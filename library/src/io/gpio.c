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
// It might not be available in cross-compile environment.
// Try here: https://github.com/torvalds/linux/blob/master/include/uapi/linux/gpio.h
//
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
#define N_CHIPS		4
#define MAX_BUF		64
#define LINES_PER_CHIP	32


static int chip_fd[N_CHIPS];
static int handle_fd[N_CHIPS*LINES_PER_CHIP];
static int event_fd[N_CHIPS*LINES_PER_CHIP];



static int pin_to_chip(int pin, int* chip, int* line)
{
	// sanity checks
	if(pin<0 || pin>=(N_CHIPS*LINES_PER_CHIP)){
		fprintf(stderr, "ERROR, gpio pin out of bounds\n");
		return -1;
	}
	*chip = pin/LINES_PER_CHIP;
	*line = pin%LINES_PER_CHIP;
	return 0;
}

static int open_gpiochip(int chip)
{
	char buf[MAX_BUF];
	int temp_fd;

	// sanity checks
	if(chip<0 || chip>N_CHIPS){
		fprintf(stderr,"ERROR, chip out of bounds\n");
		return -1;
	}

	snprintf(buf, sizeof(buf), DEVICE_BASE "%d", chip);
	temp_fd=open(buf,O_RDWR);
	if(temp_fd==-1){
		perror("ERROR opening gpiochip");
		return -1;
	}
	chip_fd[chip]=temp_fd;
	return 0;
}


int rc_gpio_init(int pin, int handle_flags)
{
	int chip, line, ret;
	struct gpiohandle_request req;

	// convert pin to chip&line, this also does sanity checks
	if(unlikely(pin_to_chip(pin, &chip, &line))) return -1;

	// open chip if not opened already
	if(chip_fd[chip]==0){
		if(unlikely(open_gpiochip(chip))) return -1;
	}

	// request only one pin
	memset(&req,0,sizeof(req));
	req.lineoffsets[0] = line;
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
	handle_fd[pin]=req.fd;
	return 0;
}


int rc_gpio_set_value(int pin, int value)
{
	int ret;
	struct gpiohandle_data data;

	// sanity checks
	if(unlikely(pin<0 || pin>=(N_CHIPS*LINES_PER_CHIP))){
		fprintf(stderr, "ERROR, gpio pin out of bounds\n");
		return -1;
	}
	if(unlikely(handle_fd[pin]==0)){
		fprintf(stderr,"ERROR, pin %d not initialized yet\n",pin);
		return -1;
	}

	if(value) data.values[0]=1;
	else data.values[0]=0;

	ret = ioctl(handle_fd[pin], GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
	if(unlikely(ret==-1)){
		perror("ERROR in rc_gpio_set_value");
		return -1;
	}

	return 0;
}


int rc_gpio_get_value(int pin)
{
	int ret;
	struct gpiohandle_data data;

	// sanity checks
	if(unlikely(pin<0 || pin>=(N_CHIPS*LINES_PER_CHIP))){
		fprintf(stderr, "ERROR, gpio pin out of bounds\n");
		return -1;
	}
	if(unlikely(handle_fd[pin]==0)){
		fprintf(stderr,"ERROR, pin %d not initialized yet\n",pin);
		return -1;
	}

	ret = ioctl(handle_fd[pin], GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data);
	if(unlikely(ret==-1)){
		perror("ERROR in rc_gpio_get_value");
		return -1;
	}

	return data.values[0];
}



int rc_gpio_init_event(int pin, int handle_flags, int event_flags)
{
	int chip, line, ret;
	struct gpioevent_request req;

	// convert pin to chip&line, this also does sanity checks
	if(unlikely(pin_to_chip(pin, &chip, &line))) return -1;

	if(unlikely(handle_flags&GPIOHANDLE_REQUEST_OUTPUT)){
		fprintf(stderr, "ERROR in rc_gpio_init_event, can't request OUTPUT and poll input events\n");
		return -1;
	}

	// open chip if not opened already
	if(chip_fd[chip]==0){
		if(unlikely(open_gpiochip(chip))) return -1;
	}

	req.lineoffset = line;
	req.eventflags = event_flags;
	req.handleflags = handle_flags;
	ret=ioctl(chip_fd[chip], GPIO_GET_LINEEVENT_IOCTL, &req);
	if(unlikely(ret==-1)){
		perror("ERROR in rc_gpio_init_event");
		return -1;
	}

	event_fd[pin]=req.fd;
	handle_fd[pin]=req.fd; // put same fd in handle array so reads also work
	return req.fd;
}


int rc_gpio_poll(int pin, int timeout_ms, uint64_t* event_time_ns)
{
	int ret;
	struct gpioevent_data event;
	struct pollfd poll_fds[1];

	// sanity checks
	if(unlikely(pin<0 || pin>=(N_CHIPS*LINES_PER_CHIP))){
		fprintf(stderr, "ERROR in rc_gpio_poll, gpio pin out of bounds\n");
		return -1;
	}
	if(unlikely(event_fd[pin]==0)){
		fprintf(stderr,"ERROR in rc_gpio_poll, pin %d not initialized yet\n",pin);
		return -1;
	}

	// configure the pollfd
	poll_fds[0].fd = event_fd[pin];
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
	ret = read(event_fd[pin], &event, sizeof(event));
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


void rc_gpio_cleanup(int pin)
{
	// sanity checks
	if(unlikely(pin<0 || pin>=(N_CHIPS*LINES_PER_CHIP))){
		fprintf(stderr, "ERROR, in rc_gpio_cleanup, gpio pin out of bounds\n");
		return;
	}
	if(handle_fd[pin]!=0){
		close(handle_fd[pin]);
		handle_fd[pin]=0;
	}
	if(event_fd[pin]!=0){
		close(event_fd[pin]);
		event_fd[pin]=0;
	}
	return;
}
