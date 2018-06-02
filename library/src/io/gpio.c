/**
 * @file rc_gpio.c
 *
 * @author     James Strawson
 * @date       03/02/2018
 */

#include <stdio.h>
#include <errno.h>
#include <unistd.h> // for read()
#include <poll.h>
#include <fcntl.h> // for open()
#include <string.h> // for memset
#include <sys/ioctl.h>

#ifdef RC_AUTOPILOT_EXT
// Not sure why #include <linux/gpio.h> did not work here after explicitly include
// the default path /usr/include, so use full path for now.
#include "/usr/include/linux/gpio.h"
#else
#include <linux/gpio.h>
#endif

#include <rc/gpio.h>

// preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)
#define likely(x)	__builtin_expect (!!(x), 1)

#define DEVICE_BASE "/dev/gpiochip"
#define CHIPS_MAX	6 // up to 6 chip chips, make larger if you want
#define MAX_BUF		64


static int chip_fd[CHIPS_MAX];
static int handle_fd[CHIPS_MAX][GPIOHANDLES_MAX];
static int event_fd[CHIPS_MAX][GPIOHANDLES_MAX];




static int __open_gpiochip(int chip)
{
	char buf[MAX_BUF];
	int temp_fd;

	snprintf(buf, sizeof(buf), DEVICE_BASE "%d", chip);
	temp_fd=open(buf,O_RDWR);
	if(temp_fd==-1){
		perror("ERROR opening gpiochip");
		return -1;
	}
	chip_fd[chip]=temp_fd;
	return 0;
}


int rc_gpio_init(int chip, int pin, int handle_flags)
{
	int ret;
	struct gpiohandle_request req;

	// sanity checks
	if(chip<0 || chip>=CHIPS_MAX){
		fprintf(stderr,"ERROR in rc_gpio_init, chip out of bounds\n");
		return -1;
	}
	if(pin<0 || pin>=GPIOHANDLES_MAX){
		fprintf(stderr,"ERROR in rc_gpio_init, pin out of bounds\n");
		return -1;
	}

	// open chip if not opened already
	if(chip_fd[chip]==0){
		if(unlikely(__open_gpiochip(chip))) return -1;
	}

	// request only one pin
	memset(&req,0,sizeof(req));
	req.lineoffsets[0] = pin;
	req.lines = 1;
	req.flags = handle_flags;
	errno=0;
	ret = ioctl(chip_fd[chip], GPIO_GET_LINEHANDLE_IOCTL, &req);
	if(unlikely(ret==-1)){
		perror("ERROR in rc_gpio_init");
		return -1;
	}
	if(req.fd==0){
		fprintf(stderr,"ERROR in rc_gpio_init, ioctl gave NULL fd\n");
		return -1;
	}
	handle_fd[chip][pin]=req.fd;
	return 0;
}


int rc_gpio_set_value(int chip, int pin, int value)
{
	int ret;
	struct gpiohandle_data data;

	// sanity checks
	// sanity checks
	if(chip<0 || chip>=CHIPS_MAX){
		fprintf(stderr,"ERROR in rc_gpio_set_value, chip out of bounds\n");
		return -1;
	}
	if(pin<0 || pin>=GPIOHANDLES_MAX){
		fprintf(stderr,"ERROR in rc_gpio_set_value, pin out of bounds\n");
		return -1;
	}
	if(unlikely(handle_fd[chip][pin]==0)){
		fprintf(stderr,"ERROR, pin %d not initialized yet\n",pin);
		return -1;
	}

	if(value) data.values[0]=1;
	else data.values[0]=0;

	ret = ioctl(handle_fd[chip][pin], GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
	if(unlikely(ret==-1)){
		perror("ERROR in rc_gpio_set_value");
		return -1;
	}

	return 0;
}


int rc_gpio_get_value(int chip, int pin)
{
	int ret;
	struct gpiohandle_data data;

	// sanity checks
	if(chip<0 || chip>=CHIPS_MAX){
		fprintf(stderr,"ERROR in rc_gpio_get_value, chip out of bounds\n");
		return -1;
	}
	if(pin<0 || pin>=GPIOHANDLES_MAX){
		fprintf(stderr,"ERROR in rc_gpio_get_value, pin out of bounds\n");
		return -1;
	}
	if(unlikely(handle_fd[chip][pin]==0)){
		fprintf(stderr,"ERROR in rc_gpio_get_value chip %d pin %d not initialized yet\n",chip, pin);
		return -1;
	}

	ret = ioctl(handle_fd[chip][pin], GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data);
	if(unlikely(ret==-1)){
		perror("ERROR in rc_gpio_get_value");
		return -1;
	}

	return data.values[0];
}



int rc_gpio_init_event(int chip, int pin, int handle_flags, int event_flags)
{
	int ret;
	struct gpioevent_request req;

	// sanity checks
	if(chip<0 || chip>=CHIPS_MAX){
		fprintf(stderr,"ERROR in rc_gpio_init_event, chip out of bounds\n");
		return -1;
	}
	if(pin<0 || pin>=GPIOHANDLES_MAX){
		fprintf(stderr,"ERROR in rc_gpio_init_event, pin out of bounds\n");
		return -1;
	}
	if(unlikely(handle_flags&GPIOHANDLE_REQUEST_OUTPUT)){
		fprintf(stderr, "ERROR in rc_gpio_init_event, can't request OUTPUT and poll input events\n");
		return -1;
	}

	// open chip if not opened already
	if(chip_fd[chip]==0){
		if(unlikely(__open_gpiochip(chip))) return -1;
	}

	req.lineoffset = pin;
	req.eventflags = event_flags;
	req.handleflags = handle_flags;
	ret=ioctl(chip_fd[chip], GPIO_GET_LINEEVENT_IOCTL, &req);
	if(unlikely(ret==-1)){
		perror("ERROR in rc_gpio_init_event");
		return -1;
	}

	event_fd[chip][pin]=req.fd;
	handle_fd[chip][pin]=req.fd; // put same fd in handle array so reads also work
	return req.fd;
}


int rc_gpio_poll(int chip, int pin, int timeout_ms, uint64_t* event_time_ns)
{
	int ret;
	struct gpioevent_data event;
	struct pollfd poll_fds[1];

	// sanity checks
	if(chip<0 || chip>=CHIPS_MAX){
		fprintf(stderr,"ERROR in rc_gpio_poll, chip out of bounds\n");
		return -1;
	}
	if(pin<0 || pin>=GPIOHANDLES_MAX){
		fprintf(stderr,"ERROR in rc_gpio_poll, pin out of bounds\n");
		return -1;
	}

	// configure the pollfd
	poll_fds[0].fd = event_fd[chip][pin];
	poll_fds[0].events = POLLIN | POLLPRI;
	poll_fds[0].revents = 0;

	// now poll
	ret = poll(poll_fds, 1, timeout_ms);
	if(unlikely(ret==-1)){
		perror("ERROR in rc_gpio_poll calling poll");
		return RC_GPIOEVENT_ERROR;
	}
	else if(ret==0) return RC_GPIOEVENT_TIMEOUT;

	// read value to see if it was rising or falling
	ret = read(event_fd[chip][pin], &event, sizeof(event));
	if(ret==-1){
		perror("ERROR in rc_gpio_poll while reading event");
		return RC_GPIOEVENT_ERROR;
	}

	if(event.id!=GPIOEVENT_EVENT_RISING_EDGE && event.id!=GPIOEVENT_EVENT_FALLING_EDGE){
		fprintf(stderr,"ERROR in rc_gpio_poll, read unknown event ID\n");
		return RC_GPIOEVENT_ERROR;
	}

	// save event time if user gave non-null pointer
	if(event_time_ns!=NULL) *event_time_ns=event.timestamp;

	// return correct direction
	if(event.id == GPIOEVENT_EVENT_RISING_EDGE)
		return RC_GPIOEVENT_RISING_EDGE;

	return RC_GPIOEVENT_FALLING_EDGE;
}


void rc_gpio_cleanup(int chip, int pin)
{
	// sanity checks
	if(chip<0 || chip>=CHIPS_MAX){
		fprintf(stderr,"ERROR in rc_gpio_cleanup, chip out of bounds\n");
		return;
	}
	if(pin<0 || pin>=GPIOHANDLES_MAX){
		fprintf(stderr,"ERROR in rc_gpio_cleanup, pin out of bounds\n");
		return;
	}
	if(handle_fd[chip][pin]!=0){
		close(handle_fd[chip][pin]);
		handle_fd[chip][pin]=0;
	}
	if(event_fd[chip][pin]!=0){
		close(event_fd[chip][pin]);
		event_fd[chip][pin]=0;
	}
	return;
}
