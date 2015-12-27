// collection of definitions for the robotics cape library
// for functions see robotics_cape.h

#ifndef ROBOTICS_CAPE_DEFS
#define ROBOTICS_CAPE_DEFS

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

// Cape name
#define CAPE_NAME 	"RoboticsCape"

// log file location
#define LOG_DIRECTORY "/root/robot_logs/"

// lock file location
// file created to indicate running process
// contains pid of current process
#define PID_FILE "/run/robotics_cape.pid"

//// Mavlink UDP input buffer size
#define MAV_BUF_LEN 512 

//// PRU Servo Control
#define SERVO_CHANNELS			8
// Most servos will keep moving out to 600-2400	
#define SERVO_EXTENDED_RANGE	1800
// normal range is from 900 to 2100 for 120 degree servos
#define SERVO_NORMAL_RANGE		1200 
// servo center at 1500us
#define SERVO_MID_US			1500 

#define MOTOR_CHANNELS	4

#define PRESSED 1
#define UNPRESSED 0

//// Button pins
// gpio # for gpio_a.b = (32*a)+b
#define PAUSE_BTN 69 	//gpio2.5 P8.9
#define MODE_BTN  68	//gpio2.4 P8.10

//// gpio output pins 
#define RED_LED 	66	//gpio2.2	P8.7
#define GRN_LED 	67	//gpio2.3	P8.8
#define MDIR1A    	60	//gpio1.28  P9.12
#define MDIR1B    	31	//gpio0.31	P9.13
#define MDIR2A    	48	//gpio1.16  P9.15
#define MDIR2B    	79	//gpio2.15  P8.38
#define MDIR4A    	70	//gpio2.6   P8.45
#define MDIR4B    	71	//gpio2.7   P8.46
#define MDIR3B    	72	//gpio2.8   P8.43
#define MDIR3A    	73	//gpio2.9   P8.44
#define MOT_STBY  	20	//gpio0.20  P9.41
#define PAIRING_PIN 		30 	//gpio0.30 	P9.11
#define SPI1_SS1_GPIO_PIN 	113 //gpio3.17	P9.28 
#define SPI1_SS2_GPIO_PIN 	49  //gpio1.17	P9.23 

//// MPU-9150 defs
#define MPU_ADDR 0x68

// 
#define POLL_TIMEOUT (3 * 1000) /* 3 seconds */
#define INTERRUPT_PIN 117  //gpio3.21 P9.25

#define UART4_PATH "/dev/ttyO4"

// PRU Servo Control shared memory pointer
#define SERVO_PRU_NUM 	 1
#define ENCODER_PRU_NUM 	 0
#define PRU_SERVO_BIN "/usr/bin/pru_1_servo.bin"
#define PRU_ENCODER_BIN "/usr/bin/pru_0_encoder.bin"
#define PRU_LOOP_INSTRUCTIONS	48		// instructions per PRU servo timer loop


#define ARRAY_SIZE(array) sizeof(array)/sizeof(array[0]) 

/*********************************
* clock control registers
*********************************/
#ifndef CM_PER
	#define CM_PER 0x44E00000 //base of Clock Module Peripheral control
	#define CM_PER_PAGE_SIZE 1024 //1kb
#endif

#define CM_PER_PRU_ICSS_CLKCTRL 0xE8 //16 bit register
#define CM_PER_PRU_ICSS_CLKSTCTRL 0x140	
#define CM_PER_UART5_CLKCTRL 0x38
#define CM_PER_I2C2_CLKCTRL 0x44
#define CM_PER_I2C1_CLKCTRL 0x48
#define CM_PER_SPI0_CLKCTRL 0x4C
#define CM_PER_SPI1_CLKCTRL 0x50
#define CM_PER_GPIO1_CLKCTRL 0xAC
#define CM_PER_GPIO2_CLKCTRL 0xB0
#define CM_PER_GPIO3_CLKCTRL 0xB4


#define MODULEMODE_DISABLED 0x0
#define MODULEMODE_ENABLE 	0x2

#endif
