/**
 * @file rc_test_dmp_tap.c
 * @example    rc_test_dmp_tap
 *
 * @brief      tests tap detection on MPU in DMP mode
 *
 * @author     James Strawson
 * @date       1/29/2018
 */


#include <stdio.h>
#include <signal.h>
#include <getopt.h>
#include <rc/mpu.h>
#include <rc/time.h>

// bus for Robotics Cape and BeagleboneBlue is 2
// change this for your platform
#define I2C_BUS 2

static int running = 0;

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}

static void __tap_callback_func(int direction, int counter)
{
	switch(direction){
		case 1:
			printf("received tap in direction: X+");
			break;
		case 2:
			printf("received tap in direction: X-");
			break;
		case 3:
			printf("received tap in direction: Y+");
			break;
		case 4:
			printf("received tap in direction: Y-");
			break;
		case 5:
			printf("received tap in direction: Z+");
			break;
		case 6:
			printf("received tap in direction: Z-");
			break;
		default:
			printf("invalid tap direction: %d", direction);
	}
	printf(" counter: %d\n", counter);
	fflush(stdout);
	return;
}

int main()
{
	rc_mpu_data_t data; //struct to hold new data

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running = 1;

	// use defaults for now, except also enable magnetometer.
	rc_mpu_config_t conf = rc_mpu_default_config();
	conf.i2c_bus = I2C_BUS;

	if(rc_mpu_initialize_dmp(&data, conf)){
		fprintf(stderr,"rc_mpu_initialize_dmp_failed\n");
		return -1;
	}

	// attach callback
	rc_mpu_set_tap_callback(__tap_callback_func);

	//now just wait, print_data will run
	while(running){
		rc_usleep(100000);
	}

	rc_mpu_power_off();
	return 0;
}

