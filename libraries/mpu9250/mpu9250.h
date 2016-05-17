/*
Functions for using the MPU9250 9-axis IMU on the Robotics Cape
James Strawson 2015
*/

#ifndef MPU9250
#define MPU9250

#include "simple_i2c.h"
//#include "inv_mpu_dmp_motion_driver.h"
#include <math.h>
#include <pthread.h>
#include "mpu9250_defs.h"
#include "dmp_firmware.h"

//I2C bus and address definitions for Robotics Cape
#define IMU_ADDR 0x68
#define IMU_BUS 2

// internal DMP sample rate limits
#define DMP_MAX_RATE 200
#define DMP_MIN_RATE 5

/*******************************************************************
* struct imu_data_t
*
* This is the container for holding the sensor data from the imu.
* The user is intended to keep make their own instance of this 
* struct and pass it's pointer to 
*******************************************************************/
typedef struct imu_data_t {
	
	// last read sensor values in real units
	float accel[3]; // units of m/s^2
	float gyro[3];	// units of degrees/s
	float mag[3];	// units of uT
	float temp;		// units of degrees celcius
	
	// 16 bit raw adc readings from each sensor
	int16_t raw_gyro[3];	
	int16_t raw_accel[3];
	
	// FSR-derived ratios from raw to real units
	float accel_to_ms2; // to m/s^2
	float gyro_to_degs; // to degrees/s

	// quaternion_t DMPQuat; 	// unitless
	// quaternion_t fusedQuat; // unitless
	// vector3d_t fusedEuler;  // units of degrees

	// complementary filter constants used
	// by imu_read_dmp
	float lastDMPYaw;
	float lastYaw;
	
	// steady state offsets loaded from calibration file
	uint16_t rawGyroBias[3];
	uint16_t rawAccelBias[3];
	
	// magnetometer factory sensitivity adjustment values
	float mag_adjust[3];
} imu_data_t;
 
/*******************************************************************
* Configuration struct passed to initialize_imu
* best to start with DEFAULT_IMU_CONFIG and work from there
*******************************************************************/
typedef struct imu_config_t {
	// full scale ranges for sensors
	accel_fsr_t accel_fsr; // AFS_2G, AFS_4G, AFS_8G, AFS_16G
	gyro_fsr_t gyro_fsr;  // GFS_250,GFS_500,GFS_1000,GFS_2000
	
	// internal low pass filter constants
	gyro_dlpf_t gyro_dlpf; // 
	accel_dlpf_t accel_dlpf;
	
	// magnetometer use is optional 
	int enable_magnetometer; // 0 or 1
	
	// DMP settings, only used with DMP interrupt
	int dmp_sample_rate;
	signed char orientation[9]; //orientation matrix
	int dmp_interrupt_priority; // scheduler priority for handler
} imu_config_t;

/*******************************************************************
* return default config struct
* best to start with this and work from there
*******************************************************************/
imu_config_t get_default_imu_config();
		

/*******************************************************************
* int initialize_imu(imu_config_t conf)
*
* Set up the imu for basic one-shot sampling of sensor data
*******************************************************************/
int initialize_imu(imu_data_t *data, imu_config_t conf);

/*******************************************************************
* one-shot sampling mode functions
*
* These makes a direct read of the most recent sensor value over 
* I2C without using the FIFO buffer, DMP, or interrupt
*******************************************************************/
int read_accel_data(imu_data_t *imu);
int read_gyro_data(imu_data_t *imu);
int read_mag_data(imu_data_t *imu);
int read_imu_temp(imu_data_t* data);

/*******************************************************************
* interrupt-driven sampling mode
*
* set up the imu for dmp-accelerated orientation estimate, fixed
* sample rate, and interrupt-driven sampling. Your own interrupt
* function should be assigned to handle the new data. This function
* will typically be a filter or discrete feedback controller.
*******************************************************************/
int initialize_imu_dmp(imu_data_t *data, imu_config_t conf);
int set_imu_interrupt_func(int (*func)(void), imu_data_t* data);
int stop_imu_interrupt_func();



/*******************************************************************
* int reset_imu();
*
*******************************************************************/
int reset_imu();
  
void initAK8963(float * destination);



#endif
