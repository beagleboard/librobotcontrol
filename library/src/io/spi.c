/**
 * @file spi.c
 *
 * @author     James Strawson
 * @date       1/19/2018
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>	// for memset
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>

#include <rc/model.h>
#include <rc/gpio.h>
#include <rc/pinmux.h>
#include <rc/spi.h>

#define MAX_BUS 5 // reasonable max spi bus should cover most platforms

#define SS_MODE_NONE		0
#define SS_MODE_AUTO		1
#define SS_MODE_MANUAL		2

#define N_SS			12 // number of possible slave select lines
#define SPI_BASE_PATH		"/dev/spidev"


typedef struct rc_spi_state_t{
	unsigned char init[N_SS];
	unsigned char ss_mode[N_SS]; ///< See INIT_MODE_NONE, AUTO, MANUAL
	unsigned char chip[N_SS];
	unsigned char pin[N_SS];
	int speed[N_SS];
	int fd[N_SS];
}rc_spi_state_t;

// state of all busses
static rc_spi_state_t state[MAX_BUS+1];


// simple opening of an FD for particular bus and slave. Tries to handle the
// case where old BeagleBone kernels enumerate spi1 as spidev2.
// returns the opened file descriptor
static int __open_fd(int bus, int slave)
{
	char buf[32];
	int ret;


	snprintf(buf,sizeof(buf), SPI_BASE_PATH "%d.%d",bus, slave);
	// open file descriptor
	ret=open(buf, O_RDWR);

	// BeagleBones for a short while enumerated SPI1 as spidev2, if there
	// was an error opening spidev1, try spidev2
	if(ret==-1 && bus==1 && (slave==0 || slave==1) && rc_model_category()==CATEGORY_BEAGLEBONE){
		snprintf(buf,sizeof(buf), SPI_BASE_PATH "%d.%d", 2, slave);
		// open file descriptor
		ret=open(buf, O_RDWR);
	}

	if(ret==-1){
		perror("ERROR in rc_spi_init, failed to open /dev/spidev device");
		if(errno!=EPERM) fprintf(stderr,"likely SPI is not enabled in the device tree or kernel\n");
		return -1;
	}
	return ret;
}


int rc_spi_init_auto_slave(int bus, int slave, int bus_mode, int speed_hz)
{
	int bits = RC_SPI_BITS_PER_WORD;
	rc_model_t model = rc_model();
	rc_model_category_t category = rc_model_category();
	int fd;

	// sanity checks
	if(bus<0 || bus>MAX_BUS){
		fprintf(stderr,"ERROR in rc_spi_init_auto_slave, bus must be between 0 and %d\n", MAX_BUS);
		return -1;
	}
	if(slave<0 || slave>=N_SS){
		fprintf(stderr,"ERROR in rc_spi_init_auto_slave, slave must be between 0 and %d\n", N_SS-1);
		return -1;
	}
	if(speed_hz>RC_SPI_MAX_SPEED || speed_hz<RC_SPI_MIN_SPEED){
		fprintf(stderr,"ERROR in rc_spi_init_auto_slave, speed_hz must be between %d & %d\n", RC_SPI_MIN_SPEED, RC_SPI_MAX_SPEED);
		return -1;
	}
	if(bus_mode!=SPI_MODE_0 && bus_mode!=SPI_MODE_1 && bus_mode!=SPI_MODE_2 && bus_mode!=SPI_MODE_3){
		fprintf(stderr,"ERROR in rc_spi_init_auto_slave, bus_mode must be SPI_MODE_0, 1, 2, or 3\n");
		return -1;
	}

	// model_specific checks
	if(bus==1 && slave==1 && (model==MODEL_BB_BLACK_RC || model==MODEL_BB_BLACK_W_RC)){
		fprintf(stderr,"ERROR in rc_spi_init_auto_slave, auto slave mode not available on slave 2 with Robotics Cape\n");
		return -1;
	}
	if(bus>1 && category==CATEGORY_BEAGLEBONE){
		fprintf(stderr,"ERROR in rc_spi_init_auto_slave, can only use spi bus 0 and 1 on BeagleBones\n");
		return -1;
	}
	if(slave>1 && category==CATEGORY_BEAGLEBONE){
		fprintf(stderr,"ERROR in rc_spi_init_auto_slave, can only use slave 0 and 1 on BeagleBones\n");
		return -1;
	}

	// get file descriptor for spi1 device
	fd=__open_fd(bus, slave);
	if(fd==-1) return -1;

	// set settings
	if(ioctl(fd, SPI_IOC_WR_MODE, &bus_mode)==-1){
		perror("ERROR in rc_spi_init_auto_slave setting spi mode");
		close(fd);
		return -1;
	}
	if(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits)==-1){
		perror("ERROR in rc_spi_init_auto_slave setting bits per word");
		close(fd);
		return -1;
	}
	if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz)==-1){
		perror("ERROR in rc_spi_init_auto_slave setting max speed hz");
		close(fd);
		return -1;
	}

	// setup pinmux for BeagleBones
	if(bus==1 && slave==0 && model==MODEL_BB_BLUE){
		if(rc_pinmux_set(BLUE_SPI_PIN_6_SS1, PINMUX_SPI)){
			fprintf(stderr,"ERROR in rc_spi_init_auto_slave, failed to set slave select pinmux to SPI mode\n");
			return -1;
		}
	}
	else if(bus==1 && slave==1 && model==MODEL_BB_BLUE){
		if(rc_pinmux_set(BLUE_SPI_PIN_6_SS2, PINMUX_SPI)){
			fprintf(stderr,"ERROR in rc_spi_init_auto_slave, failed to set slave select pinmux to SPI mode\n");
			return -1;
		}
	}
	else if(bus==1 && slave==1 && (model==MODEL_BB_BLACK_RC || model==MODEL_BB_BLACK_W_RC)){
		if(rc_pinmux_set(CAPE_SPI_PIN_6_SS2, PINMUX_SPI)){
			fprintf(stderr,"ERROR in rc_spi_init_auto_slave, failed to set slave select pinmux to SPI mode\n");
			return -1;
		}
	}

	// all done, store speed and flag initialization
	state[bus].init[slave] = 1;
	state[bus].ss_mode[slave] = SS_MODE_AUTO;
	state[bus].chip[slave] = -1;
	state[bus].pin[slave] = -1;
	state[bus].fd[slave] = fd;
	state[bus].speed[slave] = speed_hz;

	return 0;
}



int rc_spi_init_manual_slave(int bus, int slave, int bus_mode, int speed_hz, int chip, int pin)
{

	int bits = RC_SPI_BITS_PER_WORD;
	rc_model_t model = rc_model();
	rc_model_category_t category = rc_model_category();
	int fd;

	// sanity checks
	if(bus<0 || bus>MAX_BUS){
		fprintf(stderr,"ERROR in rc_spi_init_manual_slave, bus must be between 0 and %d\n", MAX_BUS);
		return -1;
	}
	if(slave<0 || slave>=N_SS){
		fprintf(stderr,"ERROR in rc_spi_init_manual_slave, slave must be between 0 and %d\n", N_SS-1);
		return -1;
	}
	if(speed_hz>RC_SPI_MAX_SPEED || speed_hz<RC_SPI_MIN_SPEED){
		fprintf(stderr,"ERROR in rc_spi_init_manual_slave, speed_hz must be between %d & %d\n", RC_SPI_MIN_SPEED, RC_SPI_MAX_SPEED);
		return -1;
	}
	if(bus_mode!=SPI_MODE_0 && bus_mode!=SPI_MODE_1 && bus_mode!=SPI_MODE_2 && bus_mode!=SPI_MODE_3){
		fprintf(stderr,"ERROR in rc_spi_init_manual_slave, bus_mode must be SPI_MODE_0, 1, 2, or 3\n");
		return -1;
	}

	// model_specific checks
	if(bus==1 && slave==1 && (model==MODEL_BB_BLACK_RC || model==MODEL_BB_BLACK_W_RC)){
		fprintf(stderr,"ERROR in rc_spi_init_manual_slave, auto slave mode not available on slave 2 with Robotics Cape\n");
		return -1;
	}
	if(bus>1 && category==CATEGORY_BEAGLEBONE){
		fprintf(stderr,"ERROR in rc_spi_init_manual_slave, can only use spi bus 0 and 1 on BeagleBones\n");
		return -1;
	}
	if(slave>1 && category==CATEGORY_BEAGLEBONE){
		fprintf(stderr,"ERROR in rc_spi_init_manual_slave, can only use slave 0 and 1 on BeagleBones\n");
		return -1;
	}

	// get file descriptor for spi1 device. all manual slaves share the
	// slave 0 fd so only open it if not opened already.
	if(state[bus].fd[0]==0){
		fd=__open_fd(bus, 0);
		if(fd==-1) return -1;

		// set settings
		if(ioctl(fd, SPI_IOC_WR_MODE, &bus_mode)==-1){
			perror("ERROR in rc_spi_init_manual_slave setting spi mode");
			close(fd);
			return -1;
		}
		if(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits)==-1){
			perror("ERROR in rc_spi_init_manual_slave setting bits per word");
			close(fd);
			return -1;
		}
		if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz)==-1){
			perror("ERROR in rc_spi_init_manual_slave setting max speed hz");
			close(fd);
			return -1;
		}
	}
	// if already open, just copy it
	else	fd = state[bus].fd[0];


	// setup pinmux for BeagleBones
	if(bus==1 && slave==0 && model==MODEL_BB_BLUE){
		if(rc_pinmux_set(BLUE_SPI_PIN_6_SS1, PINMUX_GPIO)){
			fprintf(stderr,"ERROR in rc_spi_init_manual_slave, failed to set slave select pinmux to GPIO mode\n");
			return -1;
		}
	}
	else if(bus==1 && slave==1 && model==MODEL_BB_BLUE){
		if(rc_pinmux_set(BLUE_SPI_PIN_6_SS2, PINMUX_GPIO)){
			fprintf(stderr,"ERROR in rc_spi_init_manual_slave, failed to set slave select pinmux to GPIO mode\n");
			return -1;
		}
	}
	else if(bus==1 && slave==0 && (model==MODEL_BB_BLACK_RC || model==MODEL_BB_BLACK_W_RC)){
		if(rc_pinmux_set(CAPE_SPI_PIN_6_SS1, PINMUX_GPIO)){
			fprintf(stderr,"ERROR in rc_spi_init_manual_slave, failed to set slave select pinmux to GPIO mode\n");
			return -1;
		}
	}
	else if(bus==1 && slave==1 && (model==MODEL_BB_BLACK_RC || model==MODEL_BB_BLACK_W_RC)){
		if(rc_pinmux_set(CAPE_SPI_PIN_6_SS2, PINMUX_GPIO)){
			fprintf(stderr,"ERROR in rc_spi_init_manual_slave, failed to set slave select pinmux to GPIO mode\n");
			return -1;
		}
	}

	if(rc_gpio_init(chip, pin, GPIOHANDLE_REQUEST_OUTPUT)){
		fprintf(stderr,"ERROR in rc_spi_init_manual_slave failed to initialize slave select gpio pin\n");
		return -1;
	}
	// make sure slave begins deselected
	if(rc_gpio_set_value(chip, pin, 1)){
		fprintf(stderr,"ERROR in rc_spi_init_manual_slave, failed to write to gpio slave select pin\n");
		return -1;
	}

	// all done, store speed and flag initialization
	state[bus].init[slave] = 1;
	state[bus].ss_mode[slave] = SS_MODE_MANUAL;
	state[bus].chip[slave] = chip;
	state[bus].pin[slave] = pin;
	state[bus].fd[slave] = fd;
	state[bus].fd[0] = fd; // just in case we are initializing a slave other than 0 first
	state[bus].speed[slave] = speed_hz;

	return 0;
}


int rc_spi_get_fd(int bus, int slave)
{
	// sanity checks
	if(bus<0 || bus>MAX_BUS){
		fprintf(stderr,"ERROR in rc_spi_get_fd, bus must be between 0 and %d\n", MAX_BUS);
		return -1;
	}
	if(slave<0 || slave>=N_SS){
		fprintf(stderr,"ERROR in rc_spi_get_fd, slave must be between 0 and %d\n", N_SS-1);
		return -1;
	}
	if(state[bus].init[slave]==0){
		fprintf(stderr,"ERROR in rc_spi_get_fd, need to initialize first\n");
		return -1;
	}
	return state[bus].fd[slave];
}


int rc_spi_close(int bus)
{
	int i;
	// sanity checks
	if(bus<0 || bus>MAX_BUS){
		fprintf(stderr,"ERROR in rc_spi_close, bus must be between 0 and %d\n", MAX_BUS);
		return -1;
	}

	for(i=0;i<N_SS;i++){
		// cleanup for manual slave
		if(state[bus].init[i] == SS_MODE_MANUAL){
			if(rc_gpio_set_value(state[bus].chip[i], state[bus].pin[i], 1)){
				fprintf(stderr,"WARNING in rc_spi_close, failed to write to gpio slave select pin\n");
				return -1;
			}
			rc_gpio_cleanup(state[bus].chip[i], state[bus].pin[i]);
			// all manual slaves share slave 0's fd, so only close once when i=0
			if(i==0) close(state[bus].fd[i]);
		}
		// cleanup for auto slave
		if(state[bus].init[i] == SS_MODE_AUTO){
			close(state[bus].fd[i]);
		}

		// make sure struct is wiped
		state[bus].init[i] = 0;
		state[bus].ss_mode[i] = 0;
		state[bus].chip[i] = 0;
		state[bus].pin[i] = 0;
		state[bus].fd[i] = 0;
		state[bus].speed[i] = 0;
	}

	return 0;
}


int rc_spi_manual_select(int bus, int slave, int select)
{
	// sanity checks
	if(bus<0 || bus>MAX_BUS){
		fprintf(stderr,"ERROR in rc_spi_manual_select, bus must be between 0 and %d\n", MAX_BUS);
		return -1;
	}
	if(slave<0 || slave>=N_SS){
		fprintf(stderr,"ERROR in rc_spi_manual_select, slave must be between 0 and %d\n", N_SS-1);
		return -1;
	}
	if(state[bus].init[slave]==0){
		fprintf(stderr,"ERROR in rc_spi_manual_select, need to initialize first\n");
		return -1;
	}
	if(state[bus].ss_mode[slave]!=SS_MODE_MANUAL){
		fprintf(stderr,"ERROR in rc_spi_manual_select, slave not configured in manual mode\n");
		return -1;
	}

	// invert select to it's pulled low when selecting pin
	if(rc_gpio_set_value(state[bus].chip[slave], state[bus].pin[slave], !select)==-1){
		fprintf(stderr,"ERROR in rc_spi_manual_select writing to gpio pin\n");
		return -1;
	}
	return 0;
}


int rc_spi_transfer(int bus, int slave, uint8_t* tx_data, size_t tx_bytes, uint8_t* rx_data)
{
	int ret;
	struct spi_ioc_transfer xfer = {0}; // zero-initialize per docs

	// sanity checks
	if(bus<0 || bus>MAX_BUS){
		fprintf(stderr,"ERROR in rc_spi_transfer, bus must be between 0 and %d\n", MAX_BUS);
		return -1;
	}
	if(slave<0 || slave>=N_SS){
		fprintf(stderr,"ERROR in rc_spi_transfer, slave must be between 0 and %d\n", N_SS-1);
		return -1;
	}
	if(state[bus].init[slave]==0){
		fprintf(stderr,"ERROR in rc_spi_transfer, need to initialize first\n");
		return -1;
	}
	if(tx_bytes<1){
		fprintf(stderr,"ERROR: in rc_spi_transfer, tx_bytes must be >=1\n");
		return -1;
	}

	// fill in send struct
	xfer.tx_buf = (unsigned long) tx_data;
	xfer.rx_buf = (unsigned long) rx_data;
	xfer.len = tx_bytes;
	xfer.speed_hz = state[bus].speed[slave];
	xfer.delay_usecs = 0;
	xfer.bits_per_word = RC_SPI_BITS_PER_WORD;
	xfer.cs_change = 1;

	// do ioctl transfer
	ret=ioctl(state[bus].fd[slave], SPI_IOC_MESSAGE(1), &xfer);
	if(ret==-1){
		perror("ERROR in rc_spi_transfer");
		return -1;
	}
	return ret;
}


int rc_spi_write(int bus, int slave, uint8_t* data, size_t bytes)
{
	int ret;
	struct spi_ioc_transfer xfer = {0}; // zero-initialize per docs

	// sanity checks
	if(bus<0 || bus>MAX_BUS){
		fprintf(stderr,"ERROR in rc_spi_write, bus must be between 0 and %d\n", MAX_BUS);
		return -1;
	}
	if(slave<0 || slave>=N_SS){
		fprintf(stderr,"ERROR in rc_spi_write, slave must be between 0 and %d\n", N_SS-1);
		return -1;
	}
	if(state[bus].init[slave]==0){
		fprintf(stderr,"ERROR in rc_spi_write, need to initialize first\n");
		return -1;
	}
	if(bytes<1){
		fprintf(stderr,"ERROR: in rc_spi_write, bytes must be >=1\n");
		return -1;
	}

	// fill in send struct
	xfer.tx_buf = (unsigned long) data;
	xfer.rx_buf = 0;
	xfer.len = bytes;
	xfer.speed_hz = state[bus].speed[slave];
	xfer.delay_usecs = 0;
	xfer.bits_per_word = RC_SPI_BITS_PER_WORD;
	xfer.cs_change = 1;

	// send
	ret=ioctl(state[bus].fd[slave], SPI_IOC_MESSAGE(1), &xfer);
	if(ret==-1){
		perror("ERROR in rc_spi_write");
		return -1;
	}
	return ret;
}


int rc_spi_read(int bus, int slave, uint8_t* data, size_t bytes)
{
	int ret;
	struct spi_ioc_transfer xfer = {0}; // zero-initialize per docs

	// sanity checks
	if(bus<0 || bus>MAX_BUS){
		fprintf(stderr,"ERROR in rc_spi_read, bus must be between 0 and %d\n", MAX_BUS);
		return -1;
	}
	if(slave<0 || slave>=N_SS){
		fprintf(stderr,"ERROR in rc_spi_read, slave must be between 0 and %d\n", N_SS-1);
		return -1;
	}
	if(state[bus].init[slave]==0){
		fprintf(stderr,"ERROR in rc_spi_read, need to initialize first\n");
		return -1;
	}
	if(bytes<1){
		fprintf(stderr,"ERROR: in rc_spi_read, bytes must be >=1\n");
		return -1;
	}

	// fill in send struct
	xfer.tx_buf = 0;
	xfer.rx_buf = (unsigned long) data;
	xfer.len = bytes;
	xfer.speed_hz = state[bus].speed[slave];
	xfer.delay_usecs = 0;
	xfer.bits_per_word = RC_SPI_BITS_PER_WORD;
	xfer.cs_change = 1;

	// read
	ret=ioctl(state[bus].fd[slave], SPI_IOC_MESSAGE(1), &xfer);
	if(ret==-1){
		perror("ERROR in rc_spi_read");
		return -1;
	}
	return ret;
}


