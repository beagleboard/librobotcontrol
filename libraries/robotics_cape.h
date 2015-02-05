/*
Copyright (c) 2014, James Strawson
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*/


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
#include <time.h>		// usleep, nanosleep
#include <math.h>		// atan2 and fabs
#include <signal.h>		// capture ctrl-c
#include <pthread.h>    // multi-threading
#include <linux/input.h>// buttons
#include <poll.h> 		// interrupt events
#include <sys/mman.h>	// mmap for accessing eQep
#include <sys/socket.h>	// mavlink udp socket	
#include <netinet/in.h> // mavlink udp socket	
#include <sys/time.h>
#include <arpa/inet.h>  // mavlink udp socket	
#include <ctype.h>		// for isprint()

#include "SimpleGPIO.h"
#include "c_i2c.h"		// i2c lib
#include "mpu9150.h"	// general DMP library
#include "MPU6050.h" 	// gyro offset registers
#include "tipwmss.h"	// pwmss and eqep registers
#include "mavlink/mavlink.h"
#include "prussdrv.h"
#include "pruss_intc_mapping.h"

#define DEG_TO_RAD 		0.0174532925199
#define RAD_TO_DEG 	 	57.295779513
#define PI				3.141592653
#define MAX_BUF 64


//// MPU9150 IMU
#define ORIENTATION_UPRIGHT {1,0,0, 0,0,-1, 0,1,0}; // Ethernet pointing up for BeagleMIP
#define ORIENTATION_FLAT    {1,0,0, 0,1,0, 0,0,1} // BB flat on table for BealgeQuad
#define GYRO_FSR			2000				  // default full scale range (deg/s)
#define ACCEL_FSR			2					  // default full scale range  (g)

//// Spektrum DSM2 RC Radio
#define RC_CHANNELS 9

// Calibration File Locations
#define CONFIG_DIRECTORY "/root/robot_config/"
#define DSM2_CAL_FILE	"dsm2.cal"
#define GYRO_CAL_FILE 	"gyro.cal"
#define IMU_CAL_FILE	"imu.cal"

// log file location
#define LOG_DIRECTORY "/root/robot_logs/"

// lock file location
// file created to indicate running process
// contains pid of current process
#define LOCKFILE "/tmp/robotics.lock"

//// Mavlink UDP input buffer size
#define MAV_BUF_LEN 512 

//// PRU Servo Control
#define SERVO_CHANNELS			8
#define SERVO_MIN_US 			800	// min pulse to send to servos	in microseconds
#define SERVO_MAX_US 			2200	// max pulse to send to servos in microseconds

#define PRESSED 1
#define UNPRESSED 0
	

//// Initialization function must call at beginning of main()
int initialize_cape();

//// Program Flow and State Control ////
typedef enum state_t {
	UNINITIALIZED,
	RUNNING,
	PAUSED,
	EXITING
} state_t;

enum state_t get_state();
int set_state(enum state_t);

//// Motor, ESC, PWM ///
int set_motor(int motor, float duty);
int set_esc(int esc, float normalized_duty);
int kill_pwm();
int set_all_esc(float duty);
int set_all_esc(float duty);
int enable_motors();
int disable_motors();

//// eQep encoder counter
long int get_encoder_pos(int ch);
int set_encoder_pos(int ch, long value);
 
//// Buttons LEDS interrupt functions///
int setGRN(PIN_VALUE i);
int setRED(PIN_VALUE i);
int getGRN();
int getRED();
int set_pause_pressed_func(int (*func)(void));
int set_pause_unpressed_func(int (*func)(void));
int set_mode_pressed_func(int (*func)(void));
int set_mode_unpressed_func(int (*func)(void));
int get_pause_button_state();
int get_mode_button_state();
void* read_events(void* ptr); //background thread for polling inputs

//// Battery & power
float getBattVoltage();
float getJackVoltage();

//// MPU9150 IMU DMP
mpudata_t mpu; //struct to read IMU data into
int initialize_imu(int sample_rate, signed char orientation[9]);
int setXGyroOffset(int16_t offset);
int setYGyroOffset(int16_t offset);
int setZGyroOffset(int16_t offset);
int loadGyroCalibration();
void* imu_interrupt_handler(void* ptr);
int set_imu_interrupt_func(int (*func)(void));

//// DSM2 Spektrum RC radio functions
int initialize_dsm2();
float get_dsm2_ch_normalized(int channel);
int get_dsm2_ch_raw(int channel);
int is_new_dsm2_data();
void* uart4_checker(void *ptr); //background thread

//// Mavlink 
#define DEFAULT_MAV_ADDRESS "192.168.7.1"
int sock;
struct sockaddr_in gcAddr;
struct sockaddr_in initialize_mavlink_udp(char gc_ip_addr[],  int *udp_sock);

//// SPI1  use ioctl.h
// returns a file descriptor to spi device
int initialize_spi1();
int select_spi1_slave(int slave);
int deselect_spi1_slave(int slave);	

//// PRU Servo Control Functions
int initialize_pru_servos();
int send_servo_pulse_us(int ch, float us);
int send_servo_pulse_normalized(int ch, float input);

//// General use Functions
int null_func();	// good for making interrupt handlers do nothing
char *byte_to_binary(unsigned char x); // for diagnostic prints
typedef struct timespec	timespec;
timespec diff(timespec start, timespec end); // subtract timespec structs for nanosleep()
uint64_t microsSinceEpoch();

//// Cleanup and Shutdown
void ctrl_c(int signo); // signal catcher
int cleanup_cape();		// call at the very end of main()
#endif