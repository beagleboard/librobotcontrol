#ifndef ROBOTICS_CAPE
#define ROBOTICS_CAPE

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <getopt.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>	//usleep, nanosleep
#include <math.h>	//atan2 and fabs
#include <signal.h>	//capture ctrl-c
#include <pthread.h>   // multi-threading
#include <linux/input.h> //button and interrupt events
#include "SimpleGPIO.h"
#include "mpu9150.h"

#define DEG_TO_RAD 0.0174532925199
#define RAD_TO_DEG 	  57.295779513

//// Initialization functions ///
int initialize_cape();
void cleanup(int signo);

//// Cleanup and Shutdown ///
int null_func();
void ctrl_c(int signo);
int cleanup_cape();

//// Program Flow and State Control ////
enum state_t {UNINITIALIZED,RUNNING,PAUSED,EXITING};
enum state_t state = UNINITIALIZED;
enum state_t get_state();
int set_state(enum state_t);

//// Motor, ESC, PWM ///
int set_motor(int motor, float duty);
int set_esc(int esc, float normalized_duty);
int kill_pwm();
int set_all_esc(float duty);
int set_pwm_period_ns(int period);

//// Encoders ////
long int get_encoder(int encoder);

/// Buttons LEDS BLFNAR interrupt functions///
int setGRN(PIN_VALUE i);
int setRED(PIN_VALUE i);
int set_imu_interrupt_func(int (*func)(void));
int set_start_pressed_func(int (*func)(void));
int set_start_unpressed_func(int (*func)(void));
int set_select_pressed_func(int (*func)(void));
int set_select_unpressed_func(int (*func)(void));
int get_start_button();
int get_select_button();
void* read_events(void* ptr); //background thread for polling inputs

//// Battery ////
float getBattVoltage();

//// MPU9150 IMU DMP ////
int initialize_imu(int sample_rate);

//// Spektrum DSM2 RC Radio ////
float spektrum_input_scaled(int channel);
int spektrum_input_raw(int channel);
int calibrate_spektrum();
int initialize_spektrum();
float get_rc_channel(int ch);
int get_rc_new_flag();
#define RC_CHANNELS 9
#define SPEKTRUM_CAL_FILE "/home/root/cape_calibration/spektrum.cal"
#define DEFAULT_MIN_PULSE	 1000000
#define DEFAULT_MAX_PULSE	 2000000
//void* spektrum_read(void* ptr); 
void* uart4_checker(void *ptr); //background thread for reading uart4




//// General use Functions ///
const char *byte_to_binary(int x);
typedef struct timespec	timespec;
timespec diff(timespec start, timespec end);

#endif

