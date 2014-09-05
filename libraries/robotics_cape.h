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
#include <arpa/inet.h> // mavlink udp socket	

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

//// Button pins
// gpio # for gpio_a.b = (32*a)+b
#define START_BTN 67	//gpio2.3 P8_8
#define SELECT_BTN 69	//gpio2.6 P8_9

//// motor direction and led output pins
#define MDIR1A    20	//gpio0.20
#define MDIR1B    112	//gpio3.16
#define MDIR2A    113	//gpio3.17
#define MDIR2B    61	//gpio1.29
#define MDIR3A    49	//gpio1.17
#define MDIR3B    48	//gpio1.16
#define MDIR4A    65	//gpio2.1 
#define MDIR4B    27	//gpio0.27 
#define MDIR5A    26	//gpio0.26
#define MDIR5B    68	//gpio1.28
#define MDIR6A    68	//gpio2.4
#define MDIR6B    66	//gpio2.2
#define GRN_LED 47	// gpio1.15 "P8_15"
#define RED_LED 46	// gpio1.14 "P8_16"

//// Spektrum UART4 RX must be remuxed to gpio output temporarily for pairing
#define PAIRING_PIN 30 //P9.11 gpio0.30
#define NUM_OUT_PINS 15

//// eQep and pwmss registers, more in tipwmss.h
#define PWM0_BASE   0x48300000
#define PWM1_BASE   0x48302000
#define PWM2_BASE   0x48304000
#define EQEP_OFFSET  0x180

//// MPU9150 IMU
#define ORIENTATION_UPRIGHT {0,0,1, 0,1,0,-1,0,0} // Ethernet pointing up for BeagleMIP
#define ORIENTATION_FLAT    {1,0,0, 0,1,0, 0,0,1} // BB flat on table for BealgeQuad
#define MPU_ADDR 0x68
#define GYRO_CAL_FILE "/root/cape_calibration/gyro.cal"
#define POLL_TIMEOUT (3 * 1000) /* 3 seconds */
#define INTERRUPT_PIN 117  //gpio3.21 P9.25


//// Spektrum DSM2 RC Radio
#define RC_CHANNELS 9
#define UART4_PATH "/dev/ttyO4"
#define DSM2_CAL_FILE "/root/cape_calibration/DSM2.cal"

//// Mavlink UDP
#define MAV_BUF_LEN 512 

//// SPI0
#define SPI0_SS0_GPIO_PIN 5   // P9.17
#define ARRAY_SIZE(array) sizeof(array)/sizeof(array[0]) 

//// PRU Servo Control
#define PRU_NUM 	 1
#define PRUSS0_SHARED_DATARAM   4
#define PRU_BIN_LOCATION "/usr/bin/pru_servo.bin"
#define SERVO_CHANNELS			8
#define SERVO_MIN_US 			1000	// min pulse to send to servos	in microseconds
#define SERVO_MAX_US 			2000	// max pulse to send to servos in microseconds
#define PRU_LOOP_INSTRUCTIONS	48		// instructions per PRU servo timer loop

//// Initialization function must call at beginning of main()
int initialize_cape();

//// Program Flow and State Control ////
enum state_t {UNINITIALIZED,RUNNING,PAUSED,EXITING};
enum state_t get_state();
int set_state(enum state_t);

//// Motor, ESC, PWM ///
int set_motor(int motor, float duty);
int set_esc(int esc, float normalized_duty);
int kill_pwm();
int set_all_esc(float duty);
int set_pwm_period_ns(int period);

//// eQep encoder counter
long int get_encoder_pos(int ch);
int set_encoder_pos(int ch, long value);

/// Buttons LEDS BLFNAR interrupt functions///
int setGRN(PIN_VALUE i);
int setRED(PIN_VALUE i);
int set_start_pressed_func(int (*func)(void));
int set_start_unpressed_func(int (*func)(void));
int set_select_pressed_func(int (*func)(void));
int set_select_unpressed_func(int (*func)(void));
int get_start_button();
int get_select_button();
void* read_events(void* ptr); //background thread for polling inputs

//// Battery
float getBattVoltage();

//// MPU9150 IMU DMP
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

//// General use Functions
int null_func();	// good for making interrupt handlers do nothing
char *byte_to_binary(unsigned char x); // for diagnostic prints
typedef struct timespec	timespec;
timespec diff(timespec start, timespec end); // subtract timespec structs for nanosleep()
uint64_t microsSinceEpoch();

//// Mavlink easy setup on udp port
struct sockaddr_in initialize_mavlink_udp(char gc_ip_addr[],  int *udp_sock);

//// SPI0  use ioctl.h
// returns a file descriptor to spi device
int initialize_spi0();
int select_spi0_slave(int slave);
int deselect_spi0_slave(int slave);	

//// Cleanup and Shutdown
void ctrl_c(int signo); // signal catcher
int cleanup_cape();		// call at the very end of main()

#endif