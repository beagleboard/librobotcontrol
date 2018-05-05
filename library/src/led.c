/**
 * @file led.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>

#include <rc/led.h>
#include <rc/time.h>

// preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)


#define SYSFS_LED_DIR "/sys/devices/platform/leds/leds/"
#define BRIGHTNESS_FILE	"/brightness"
#define MAX_BUF 128
#define NUM_LEDS 11

static const char* paths[] = {
	"green",
	"red",
	"beaglebone:green:usr0",
	"beaglebone:green:usr1",
	"beaglebone:green:usr2",
	"beaglebone:green:usr3",
	"bat25",
	"bat50",
	"bat75",
	"bat100",
	"wifi"
};

static int fd[NUM_LEDS];
static int blinking[NUM_LEDS];
static int stop_blinking_flag[NUM_LEDS];

// initializes a single led file descriptor
static int init_led(rc_led_t led)
{
	if(fd[(int)led] != 0){
		#ifdef DEBUG
		fprintf(stderr,"WARNING, trying to initialize an LED which already has file descriptor\n");
		#endif
		return 0;
	}
	// open file descriptor for read and write
	char buf[MAX_BUF];
	int temp_fd;
	snprintf(buf,sizeof(buf),SYSFS_LED_DIR "%s" BRIGHTNESS_FILE, paths[(int)led]);
	temp_fd = open(buf, O_RDWR);
	if(temp_fd==-1){
		perror("ERROR: in init_led, failed to open LED file handle");
		return -1;
	}
	fd[(int)led]=temp_fd;
	return 0;
}


int rc_led_set(rc_led_t led, int value)
{
	if(fd[(int)led] == 0){
		#ifdef DEBUG
		fprintf(stderr,"initializing led\n");
		#endif
		if(init_led(led)<0) return -1;
	}
	int ret;
	if(value)	ret=write(fd[(int)led], "1", 2);
	else		ret=write(fd[(int)led], "0", 2);
	if(ret==-1){
		perror("ERROR in rc_led_set writing to led fd");
		return -1;
	}
	return 0;
}

void rc_led_cleanup()
{
	int i;
	for(i=0;i<NUM_LEDS;i++) close(fd[i]);
	return;
}


int rc_led_get(rc_led_t led)
{
	int ret;
	char ch;
	if(fd[(int)led] == 0){
		#ifdef DEBUG
		fprintf(stderr,"initializing led\n");
		#endif
		if(init_led(led)<0) return -1;
	}

	ret=read(fd[(int)led], &ch, 1);
	if(ret==-1){
		perror("ERROR in rc_led_get reading file descriptor");
		return -1;
	}
	if (ch == '0') return 0;
	return 1;
}


int rc_led_blink(rc_led_t led, float hz, float duration)
{
	int i;
	int toggle = 0;

	if(blinking[(int)led]){
		fprintf(stderr, "ERROR: in rc_led_blink(), led is already blinking!\n");
		return -1;
	}

	// figure out constants for later
	const int delay_us = 1000000.0f/(2.0f*hz);
	const int blinks = duration*2.0f*hz;

	// make sure global flags are set before starting
	blinking[(int)led]=1;
	stop_blinking_flag[(int)led]=0;

	// loop though blinks, exiting if necessary
	for(i=0;i<blinks;i++){
		if(stop_blinking_flag[(int)led]){
			// make sure it is left off
			rc_led_set(led, 0);
			blinking[(int)led]=0;
			return 1;
		}
		toggle = !toggle;
		if(rc_led_set(led,toggle)<0){
			blinking[(int)led]=0;
			return -1;
		}
		// wait for next blink
		rc_usleep(delay_us);
	}
	// make sure it is left off
	rc_led_set(led, 0);
	blinking[(int)led]=0;
	return 0;
}


void rc_led_stop_blink(rc_led_t led)
{
	stop_blinking_flag[(int)led]=0;
	return;
}


void rc_led_stop_blink_all()
{
	int i;
	for(i=0;i<NUM_LEDS;i++) stop_blinking_flag[i]=0;
	return;
}
