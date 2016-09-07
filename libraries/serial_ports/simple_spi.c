/*******************************************************************************
* simple_spi.c
*
* Functions for interfacing with SPI1 on the beaglebone and Robotics Cape
*******************************************************************************/

#include "../robotics_cape.h"
#include "../robotics_cape_defs.h"
#include "../simple_gpio/simple_gpio.h" // for configuring gpio pins
#include "../mmap/mmap_gpio_adc.h"		// for toggling gpio pins
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h> // for memset
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define SPI1_PATH 			"/dev/spidev1.0"
#define SPI_MAX_SPEED 		24000000 	// 24mhz
#define SPI_MIN_SPEED 		1000		// 1khz
#define SPI_BITS_PER_WORD 	8
#define SPI_BUF_SIZE		2		

int fd; // file descriptor for SPI1_PATH device
int initialized; 	// set to 1 after successful initialized. 
int slave_selected; // set to 1 once a slave has been selected.

struct spi_ioc_transfer xfer[2]; // ioctl transfer structs for tx & rx
char tx_buf[SPI_BUF_SIZE];
char rx_buf[SPI_BUF_SIZE];

/*******************************************************************************
* @ int initialize_spi1(int mode, int speed_hz)
*
* Functions for interfacing with SPI1 on the beaglebone and Robotics Cape
*******************************************************************************/
int initialize_spi1(int mode, int speed_hz){
	int bits = SPI_BITS_PER_WORD;
	int mode_proper;
	
	// sanity checks
	if(speed_hz>SPI_MAX_SPEED || speed_hz<SPI_MIN_SPEED){
		printf("ERROR: SPI speed_hz must be between %d & %d\n", SPI_MIN_SPEED,\
																SPI_MAX_SPEED);
		return -1;
	}
	
	// switch 4 standard SPI modes 0-3. return error otherwise
	switch(mode){
		case 0: mode_proper = SPI_MODE_0; break;
		case 1: mode_proper = SPI_MODE_1; break;
		case 2: mode_proper = SPI_MODE_2; break;
		case 3: mode_proper = SPI_MODE_3; break;
		default: 
			printf("ERROR: SPI mode must be 0, 1, 2, or 3\n");
			printf("check your device datasheet to see which to use\n");
			return -1;
	}
	
	// set up slave select gpio pins
	if(gpio_export(SPI1_SS1_GPIO_PIN)){
		printf("ERROR: can't export gpio %d\n", SPI1_SS1_GPIO_PIN);
		return -1;
	}if(gpio_export(SPI1_SS2_GPIO_PIN)){
		printf("ERROR: can't export gpio %d\n", SPI1_SS2_GPIO_PIN);
		return -1;
	}if(gpio_set_dir(SPI1_SS1_GPIO_PIN, OUTPUT_PIN)){
		printf("ERROR: can't set direction gpio %d\n", SPI1_SS1_GPIO_PIN);
		return -1;
	}if(gpio_set_dir(SPI1_SS2_GPIO_PIN, OUTPUT_PIN)){
		printf("ERROR: can't set direction gpio %d\n", SPI1_SS2_GPIO_PIN);
		return -1;
	}if(mmap_gpio_write(SPI1_SS1_GPIO_PIN, HIGH)){
		printf("ERROR: can't write to gpio %d\n", SPI1_SS1_GPIO_PIN);
		return -1;
	}if(mmap_gpio_write(SPI1_SS2_GPIO_PIN, HIGH)){
		printf("ERROR: can't write to gpio %d\n", SPI1_SS2_GPIO_PIN);
		return -1;
	}

	// get file descriptor for spi1 device
	fd = open(SPI1_PATH, O_RDWR);
    if (fd <0) {
        printf("spidev not found in /dev/\n"); 
        return -1; 
    } 
	// set settings
	if(ioctl(fd, SPI_IOC_WR_MODE, &mode_proper)<0){
		printf("can't set spi mode");
		close(fd);
		return -1;
	}if(ioctl(fd, SPI_IOC_RD_MODE, &mode_proper)<0){
		printf("can't get spi mode");
		close(fd);
		return -1;
	}if(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits)<0){
		printf("can't set bits per word");
		close(fd);
		return -1;
	}if(ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits)<0){
		printf("can't get bits per word");
		close(fd);
		return -1;
	} if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz)<0){
		printf("can't set max speed hz");
		close(fd);
		return -1;
	} if(ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed_hz)<0){
		 printf("can't get max speed hz");
		 close(fd);
		 return -1;
	}

	// store settings
    xfer[0].cs_change = 0; // don't toggle CS
    xfer[0].delay_usecs = 0;
    xfer[0].speed_hz = speed_hz;
    xfer[0].bits_per_word = SPI_BITS_PER_WORD;
    xfer[1].cs_change = 0;
    xfer[1].delay_usecs = 0;
    xfer[1].speed_hz = speed_hz;
    xfer[1].bits_per_word = SPI_BITS_PER_WORD;
	
	// all done
	initialized = 1;
	return 0;
}

/*******************************************************************************
* int get_spi1_fd()
*
* Returns the file descriptor for spi1 once initialized.
* Use this if you want to do your own reading and writing to the bus instead
* of the basic functions defined here. If the bus has not been initialized, 
* return -1
*******************************************************************************/
int get_spi1_fd(){
	// sanity checks
	if (initialized==0){
		printf("ERROR: SPI1 not initialized yet\n");
		return -1;
	}
	return fd;
}

/*******************************************************************************
* @ int close_spi1()
*
* Closes the file descriptor and sets initialized to 0.
*******************************************************************************/
int close_spi1(){
	if(close(fd)) printf("failed to close spi1 file descriptor\n");
	initialized = 0;
	return 0;
}

/*******************************************************************************
* @ int select_spi1_slave(int slave)
*
* Selects a slave (1 or 2) by pulling the corresponding slave select pin
* to ground. It also ensures the other slave is not selected.
*******************************************************************************/
int select_spi1_slave(int slave){
	switch(slave){
		case 1:
			mmap_gpio_write(SPI1_SS2_GPIO_PIN, HIGH);
			mmap_gpio_write(SPI1_SS1_GPIO_PIN, LOW);
			break;
		case 2:
			mmap_gpio_write(SPI1_SS1_GPIO_PIN, HIGH);
			mmap_gpio_write(SPI1_SS2_GPIO_PIN, LOW);
			break;
		default:
			printf("SPI slave number must be 1 or 2\n");
			return -1;
	}
	slave_selected = 1;
	return 0;
}	

/*******************************************************************************
* @ int deselect_spi1_slave(int slave)
*
* Deselects a slave (1 or 2) by pulling the corresponding slave select pin
* to to 3.3V.
*******************************************************************************/
int deselect_spi1_slave(int slave){
	switch(slave){
		case 1:
			mmap_gpio_write(SPI1_SS1_GPIO_PIN, HIGH);
			break;
		case 2:
			mmap_gpio_write(SPI1_SS2_GPIO_PIN, HIGH);
			break;
		default:
			printf("SPI slave number must be 1 or 2\n");
			return -1;
	}
	slave_selected = 0;
	return 0;
}	

/*******************************************************************************
* int spi1_send_bytes(char* data, int bytes)
*
* Like uart_send_bytes, this lets you send any byte sequence you like.
*******************************************************************************/
int spi1_send_bytes(char* data, int bytes){
	// sanity checks
	if(initialized==0){
		printf("ERROR: SPI1 not yet initialized\n");
		return -1;
	} 
	if(bytes<1){
		printf("ERROR: spi_send_bytes, bytes to send must be >=1\n");
		return -1;
	}
  
	int ret;

	// fill in ioctl zfer struct. speed and bits were already set in initialize
	xfer[0].rx_buf = 0;
	xfer[0].tx_buf = (unsigned long) data;
	xfer[0].len = bytes;
	
	// send
	ret=ioctl(fd, SPI_IOC_MESSAGE(1), xfer);
	if(ret<0){
		printf("ERROR: SPI_IOC_MESSAGE_FAILED\n");
		return -1;
	}
	return ret;
}

/*******************************************************************************
* int spi1_read_bytes(char* data, int bytes)
*
* Like uart_read_bytes, this lets you read a byte sequence without sending.
*******************************************************************************/
int spi1_read_bytes(char* data, int bytes){
	// sanity checks
	if(initialized==0){
		printf("ERROR: SPI1 not yet initialized\n");
		return -1;
	} 
	if(bytes<1){
		printf("ERROR: spi_send_bytes, bytes to send must be >=1\n");
		return -1;
	}

	int ret;

	// fill in ioctl zfer struct. speed and bits were already set in initialize
	xfer[0].rx_buf = (unsigned long) data;;
	xfer[0].tx_buf = 0;
	xfer[0].len = bytes;
	
	// receive
	ret=ioctl(fd, SPI_IOC_MESSAGE(1), xfer);
	if(ret<0){
		printf("ERROR: SPI_IOC_MESSAGE_FAILED\n");
		return -1;
	}
	return ret;
}

/*******************************************************************************
* @ int spi1_transfer(char* tx_data, int tx_bytes, char* rx_data)
*
* This is a generic wrapper for the ioctl spi transfer function. It lets the
* user send any sequence of bytes and read the response. The return value is
* the number of bytes received or -1 on error.
*******************************************************************************/
int spi1_transfer(char* tx_data, int tx_bytes, char* rx_data){
	// sanity checks
	if(initialized==0){
		printf("ERROR: SPI1 not yet initialized\n");
		return -1;
	} if(tx_bytes<1){
		printf("ERROR: spi1_transfer, bytes must be >=1\n");
	}
	
	int ret;

	// fill in send struct 
	xfer[0].tx_buf = (unsigned long) tx_data; 
	xfer[0].rx_buf = (unsigned long) rx_data;
	xfer[0].len = tx_bytes;

	ret=ioctl(fd, SPI_IOC_MESSAGE(1), xfer);
	if(ret<0){
		printf("SPI_IOC_MESSAGE_FAILED\n");
		return -1;
	}

	return ret;
}

/*******************************************************************************
* int spi1_write_reg_byte(char reg_addr, char data)
*
* Used for writing a byte value to a register. This sends in order the address
* and byte to be written. It also sets the MSB of the register to 1 which 
* indicates a write operation on many ICs. If you do not want this particular 
* functionality, use spi1_send_bytes() to send a byte string of your choosing.
*******************************************************************************/
int spi1_write_reg_byte(char reg_addr, char data){
	// sanity checks
	if(initialized==0){
		printf("ERROR: SPI1 not yet initialized\n");
		return -1;
	} 
	
	//wipe TX buffer and fill in register address and data
	memset(tx_buf, 0, sizeof tx_buf);
	tx_buf[0] = reg_addr | 0x80; /// set MSBit = 1 to indicate it's a write
	tx_buf[1] = data;
  
	// fill in ioctl zfer struct. speed and bits were already set in initialize
	xfer[0].tx_buf = (unsigned long) tx_buf;
	xfer[0].len = 2;
	
	// send
	if(ioctl(fd, SPI_IOC_MESSAGE(1), xfer)<0){
		printf("ERROR: SPI_IOC_MESSAGE_FAILED\n");
		return -1;
	}
	return 0;
}

/*******************************************************************************
* char spi1_read_reg_byte(char reg_addr)
*
* Reads a single character located at address reg_addr. This is accomplished
* by sending the reg_addr with the MSB set to 0 indicating a read on many
* ICs. 
*******************************************************************************/
char spi1_read_reg_byte(char reg_addr){
	// sanity checks
	if(initialized==0){
		printf("ERROR: SPI1 not yet initialized\n");
		return -1;
	} 
	
	// wipe buffers
	memset(tx_buf, 0, sizeof tx_buf);
	memset(rx_buf, 0, sizeof rx_buf);
	
	// fill in xfer struct 
	tx_buf[0] = reg_addr & 0x7f; // MSBit = 0 to indicate it's a read
	xfer[0].tx_buf = (unsigned long) tx_buf; 
	xfer[0].rx_buf = (unsigned long) rx_buf;
	xfer[0].len = 1;

	if(ioctl(fd, SPI_IOC_MESSAGE(1), xfer)<0){
		printf("SPI_IOC_MESSAGE_FAILED\n");
		return -1;
	}
	return rx_buf[0];
}

/*******************************************************************************
* int spi1_read_reg_bytes(char reg_addr, char* data, int bytes)
*
* Reads multiple bytes located at address reg_addr. This is accomplished
* by sending the reg_addr with the MSB set to 0 indicating a read on many
* ICs. 
*******************************************************************************/
int spi1_read_reg_bytes(char reg_addr, char* data, int bytes){
	// sanity checks
	if(initialized==0){
		printf("ERROR: SPI1 not yet initialized\n");
		return -1;
	} if(bytes<1){
		printf("ERROR: spi1_read_reg_bytes, bytes must be >=1\n");
	}

	int ret;

	// wipe buffers
	memset(tx_buf, 0, sizeof tx_buf);
	memset(data, 0, bytes);
	
	// fill in send struct 
	tx_buf[0] = reg_addr & 0x7f; // MSBit = 0 to indicate it's a read
	xfer[0].tx_buf = (unsigned long) tx_buf; 
	xfer[0].rx_buf = 0;
	xfer[0].len = 1;
	// fill in receive struct
	xfer[1].tx_buf = 0;
	xfer[1].rx_buf = (unsigned long) data;
	xfer[1].len = bytes;

	ret=ioctl(fd, SPI_IOC_MESSAGE(2), xfer);
	if (ret<0){
		printf("SPI_IOC_MESSAGE_FAILED\n");
		return -1;
	}

	return 0;
}


