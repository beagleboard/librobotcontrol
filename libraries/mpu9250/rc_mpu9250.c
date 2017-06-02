/*******************************************************************************
* mpu9250.c
*
* This is a collection of high-level functions to control the
* MPU9250 from a BeagleBone Black as configured on the Robotics Cape.
* Credit to Kris Winer most of the framework and register definitions.
*******************************************************************************/
#define _GNU_SOURCE
#include "../rc_defs.h"
#include "../roboticscape.h"
#include "../preprocessor_macros.h"
#include "rc_mpu9250_defs.h"
#include "dmp_firmware.h"
#include "dmpKey.h"
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <poll.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

// macros
#define ARRAY_SIZE(array) sizeof(array)/sizeof(array[0])
#define min(a, b)	((a < b) ? a : b)
#define DEG_TO_RAD		0.0174532925199
#define RAD_TO_DEG		57.295779513
#define PI				M_PI
#define TWO_PI			(2.0 * M_PI)

// there should be 28 or 35 bytes in the FIFO if the magnetometer is disabled
// or enabled.
#define FIFO_LEN_NO_MAG 28
#define FIFO_LEN_MAG	35

// error threshold checks
#define QUAT_ERROR_THRESH		(1L<<16) // very precise threshold
#define QUAT_MAG_SQ_NORMALIZED	(1L<<28)
#define QUAT_MAG_SQ_MIN			(QUAT_MAG_SQ_NORMALIZED - QUAT_ERROR_THRESH)
#define QUAT_MAG_SQ_MAX			(QUAT_MAG_SQ_NORMALIZED + QUAT_ERROR_THRESH)
#define GYRO_CAL_THRESH			50
#define GYRO_OFFSET_THRESH		500

// Thread control
pthread_mutex_t rc_imu_read_mutex     = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  rc_imu_read_condition = PTHREAD_COND_INITIALIZER;

/*******************************************************************************
*	Local variables
*******************************************************************************/
rc_imu_config_t config;
int bypass_en;  
int dmp_en;
int packet_len;
pthread_t imu_interrupt_thread;
int thread_running_flag;
struct sched_param params;
void (*imu_interrupt_func)(); // pointer to user's interrupt function
int interrupt_func_set;
float mag_factory_adjust[3];
float mag_offsets[3];
float mag_scales[3];
int last_read_successful;
uint64_t last_interrupt_timestamp_nanos;
rc_imu_data_t* data_ptr;
int shutdown_interrupt_thread = 0;
// for magnetometer Yaw filtering
rc_filter_t low_pass, high_pass;

/*******************************************************************************
*	config functions for internal use only
*******************************************************************************/
int reset_mpu9250();
int set_gyro_fsr(rc_gyro_fsr_t fsr, rc_imu_data_t* data);
int set_accel_fsr(rc_accel_fsr_t, rc_imu_data_t* data);
int set_gyro_dlpf(rc_gyro_dlpf_t);
int set_accel_dlpf(rc_accel_dlpf_t);
int initialize_magnetometer();
int power_down_magnetometer();
int mpu_set_bypass(unsigned char bypass_on);
int mpu_write_mem(unsigned short mem_addr, unsigned short length,\
												unsigned char *data);
int mpu_read_mem(unsigned short mem_addr, unsigned short length,\
												unsigned char *data);
int dmp_load_motion_driver_firmware();
int dmp_set_orientation(unsigned short orient);
int dmp_enable_gyro_cal(unsigned char enable);
int dmp_enable_lp_quat(unsigned char enable);
int dmp_enable_6x_lp_quat(unsigned char enable);
int mpu_reset_fifo(void);
int mpu_set_sample_rate(int rate);
int dmp_set_fifo_rate(unsigned short rate);
int dmp_enable_feature(unsigned short mask);
int mpu_set_dmp_state(unsigned char enable);
int set_int_enable(unsigned char enable);
int dmp_set_interrupt_mode(unsigned char mode);
int read_dmp_fifo(rc_imu_data_t* data);
int data_fusion(rc_imu_data_t* data);
int load_gyro_offets();
int load_mag_calibration();
int write_mag_cal_to_disk(float offsets[3], float scale[3]);
void* imu_interrupt_handler(void* ptr);
int check_quaternion_validity(unsigned char* raw, int i);


/*******************************************************************************
* rc_imu_config_t rc_default_imu_config()
*
* returns reasonable default configuration values
*******************************************************************************/
rc_imu_config_t rc_default_imu_config(){
	rc_imu_config_t conf;
	
	// general stuff
	conf.accel_fsr	= A_FSR_4G;
	conf.gyro_fsr	= G_FSR_1000DPS;
	conf.gyro_dlpf	= GYRO_DLPF_92;
	conf.accel_dlpf	= ACCEL_DLPF_92;
	conf.enable_magnetometer = 0;
	
	// DMP stuff
	conf.dmp_sample_rate = 100;
	conf.orientation = ORIENTATION_Z_UP;
	conf.compass_time_constant = 5.0;
	conf.dmp_interrupt_priority = sched_get_priority_max(SCHED_FIFO)-1;
	conf.show_warnings = 0;
	return conf;
}

/*******************************************************************************
* int rc_set_imu_config_to_defaults(*rc_imu_config_t);
*
* resets an rc_imu_config_t struct to default values
*******************************************************************************/
int rc_set_imu_config_to_defaults(rc_imu_config_t *conf){
	*conf = rc_default_imu_config();
	return 0;
}

/*******************************************************************************
* int rc_initialize_imu(rc_imu_config_t conf)
*
* Set up the imu for one-shot sampling of sensor data by user
*******************************************************************************/
int rc_initialize_imu(rc_imu_data_t *data, rc_imu_config_t conf){  
	uint8_t c;
	
	// make sure the bus is not currently in use by another thread
	// do not proceed to prevent interfering with that process
	if(rc_i2c_get_in_use_state(IMU_BUS)){
		printf("i2c bus claimed by another process\n");
		printf("Continuing with rc_initialize_imu() anyway.\n");
	}
	
	// if it is not claimed, start the i2c bus
	if(rc_i2c_init(IMU_BUS, IMU_ADDR)<0){
		fprintf(stderr,"failed to initialize i2c bus\n");
		return -1;
	}
	// claiming the bus does no guarantee other code will not interfere 
	// with this process, but best to claim it so other code can check
	// like we did above
	rc_i2c_claim_bus(IMU_BUS);
	
	// update local copy of config struct with new values
	config=conf;
	
	// restart the device so we start with clean registers
	if(reset_mpu9250()<0){
		fprintf(stderr,"ERROR: failed to reset_mpu9250\n");
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
	
	//check the who am i register to make sure the chip is alive
	if(rc_i2c_read_byte(IMU_BUS, WHO_AM_I_MPU9250, &c)<0){
		fprintf(stderr,"Reading WHO_AM_I_MPU9250 register failed\n");
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
	if(c!=0x71){
		fprintf(stderr,"mpu9250 WHO AM I register should return 0x71\n");
		fprintf(stderr,"WHO AM I returned: 0x%x\n", c);
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
 
	// load in gyro calibration offsets from disk
	if(load_gyro_offets()<0){
		fprintf(stderr,"ERROR: failed to load gyro calibration offsets\n");
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
	
	// Set sample rate = 1000/(1 + SMPLRT_DIV)
	// here we use a divider of 0 for 1khz sample
	if(rc_i2c_write_byte(IMU_BUS, SMPLRT_DIV, 0x00)){
		fprintf(stderr,"I2C bus write error\n");
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
	
	// set full scale ranges and filter constants
	if(set_gyro_fsr(conf.gyro_fsr, data)){
		fprintf(stderr,"failed to set gyro fsr\n");
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
	if(set_accel_fsr(conf.accel_fsr, data)){
		fprintf(stderr,"failed to set accel fsr\n");
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
	if(set_gyro_dlpf(conf.gyro_dlpf)){
		fprintf(stderr,"failed to set gyro dlpf\n");
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
	if(set_accel_dlpf(conf.accel_dlpf)){
		fprintf(stderr,"failed to set accel_dlpf\n");
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
	
	// initialize the magnetometer too if requested in config
	if(conf.enable_magnetometer){
		if(initialize_magnetometer()){
			fprintf(stderr,"failed to initialize magnetometer\n");
			rc_i2c_release_bus(IMU_BUS);
			return -1;
		}
	}
	else power_down_magnetometer();
	
	// all done!!
	rc_i2c_release_bus(IMU_BUS);
	return 0;
}

/*******************************************************************************
* int rc_read_accel_data(rc_imu_data_t* data)
* 
* Always reads in latest accelerometer values. The sensor 
* self-samples at 1khz and this retrieves the latest data.
*******************************************************************************/
int rc_read_accel_data(rc_imu_data_t *data){
	// new register data stored here
	uint8_t raw[6];  
	// set the device address
	rc_i2c_set_device_address(IMU_BUS, IMU_ADDR);
	 // Read the six raw data registers into data array
	if(rc_i2c_read_bytes(IMU_BUS, ACCEL_XOUT_H, 6, &raw[0])<0){
		return -1;
	}
	// Turn the MSB and LSB into a signed 16-bit value
	data->raw_accel[0] = (int16_t)(((uint16_t)raw[0]<<8)|raw[1]);
	data->raw_accel[1] = (int16_t)(((uint16_t)raw[2]<<8)|raw[3]);
	data->raw_accel[2] = (int16_t)(((uint16_t)raw[4]<<8)|raw[5]);
	// Fill in real unit values
	data->accel[0] = data->raw_accel[0] * data->accel_to_ms2;
	data->accel[1] = data->raw_accel[1] * data->accel_to_ms2;
	data->accel[2] = data->raw_accel[2] * data->accel_to_ms2;
	return 0;
}

/*******************************************************************************
* int rc_read_gyro_data(rc_imu_data_t* data)
*
* Always reads in latest gyroscope values. The sensor self-samples
* at 1khz and this retrieves the latest data.
*******************************************************************************/
int rc_read_gyro_data(rc_imu_data_t *data){
	// new register data stored here
	uint8_t raw[6];
	// set the device address
	rc_i2c_set_device_address(IMU_BUS, IMU_ADDR);
	// Read the six raw data registers into data array
	if(rc_i2c_read_bytes(IMU_BUS, GYRO_XOUT_H, 6, &raw[0])<0){
		return -1;
	}
	// Turn the MSB and LSB into a signed 16-bit value
	data->raw_gyro[0] = (int16_t)(((int16_t)raw[0]<<8)|raw[1]);
	data->raw_gyro[1] = (int16_t)(((int16_t)raw[2]<<8)|raw[3]);
	data->raw_gyro[2] = (int16_t)(((int16_t)raw[4]<<8)|raw[5]);
	// Fill in real unit values
	data->gyro[0] = data->raw_gyro[0] * data->gyro_to_degs;
	data->gyro[1] = data->raw_gyro[1] * data->gyro_to_degs;
	data->gyro[2] = data->raw_gyro[2] * data->gyro_to_degs;
	return 0;
}

/*******************************************************************************
* int rc_read_mag_data(rc_imu_data_t* data)
*
* Checks if there is new magnetometer data and reads it in if true.
* Magnetometer only updates at 100hz, if there is no new data then
* the values in rc_imu_data_t struct are left alone.
*******************************************************************************/
int rc_read_mag_data(rc_imu_data_t* data){
	uint8_t st1;
	uint8_t raw[7];
	int16_t adc[3];
	float factory_cal_data[3];
	if(config.enable_magnetometer==0){
		fprintf(stderr,"ERROR: can't read magnetometer unless it is enabled in \n");
		fprintf(stderr,"rc_imu_config_t struct before calling rc_initialize_imu\n");
		return -1;
	}
	// magnetometer is actually a separate device with its
	// own address inside the mpu9250
	// MPU9250 was put into passthrough mode 
	rc_i2c_set_device_address(IMU_BUS, AK8963_ADDR);
	// read the data ready bit to see if there is new data
	if(rc_i2c_read_byte(IMU_BUS, AK8963_ST1, &st1)<0){
		fprintf(stderr,"ERROR reading Magnetometer, i2c_bypass is probably not set\n");
		return -1;
	}
	#ifdef DEBUG
	printf("st1: %d", st1);
	#endif
	if(!(st1&MAG_DATA_READY)){ 
		#ifdef DEBUG
		printf("no new data\n");
		#endif
		return 0;
	}
	// Read the six raw data regs into data array	
	if(rc_i2c_read_bytes(IMU_BUS,AK8963_XOUT_L,7,&raw[0])<0){
		printf("rc_read_mag_data failed\n");
		return -1;
	}
	// check if the readings saturated such as because
	// of a local field source, discard data if so
	if(raw[6]&MAGNETOMETER_SATURATION){
		fprintf(stderr,"ERROR: magnetometer saturated\n");
		return -1;
	}
	// Turn the MSB and LSB into a signed 16-bit value
	// Data stored as little Endian
	adc[0] = (int16_t)(((int16_t)raw[1]<<8) | raw[0]);  
	adc[1] = (int16_t)(((int16_t)raw[3]<<8) | raw[2]);  
	adc[2] = (int16_t)(((int16_t)raw[5]<<8) | raw[4]); 
	#ifdef DEBUG
	printf("raw mag:%d %d %d\n", adc[0], adc[1], adc[2]);
	#endif

	// multiply by the sensitivity adjustment and convert to units of uT micro
	// Teslas. Also correct the coordinate system as someone in invensense 
	// thought it would be bright idea to have the magnetometer coordiate
	// system aligned differently than the accelerometer and gyro.... -__-
	factory_cal_data[0] = adc[1] * mag_factory_adjust[1] * MAG_RAW_TO_uT;
	factory_cal_data[1] = adc[0] * mag_factory_adjust[0] * MAG_RAW_TO_uT;
	factory_cal_data[2] = -adc[2] * mag_factory_adjust[2] * MAG_RAW_TO_uT;

	// now apply out own calibration, but first make sure we don't accidentally
	// multiply by zero in case of uninitialized scale factors
	if(mag_scales[0]==0.0) mag_scales[0]=1.0;
	if(mag_scales[1]==0.0) mag_scales[1]=1.0;
	if(mag_scales[2]==0.0) mag_scales[2]=1.0;
	data->mag[0] = (factory_cal_data[0]-mag_offsets[0])*mag_scales[0];
	data->mag[1] = (factory_cal_data[1]-mag_offsets[1])*mag_scales[1];
	data->mag[2] = (factory_cal_data[2]-mag_offsets[2])*mag_scales[2];

	return 0;
}

/*******************************************************************************
* int rc_read_imu_temp(rc_imu_data_t* data)
*
* reads the latest temperature of the imu. 
*******************************************************************************/
int rc_read_imu_temp(rc_imu_data_t* data){
	uint16_t adc;
	// set device address
	rc_i2c_set_device_address(IMU_BUS, IMU_ADDR);
	// Read the two raw data registers
	if(rc_i2c_read_word(IMU_BUS, TEMP_OUT_H, &adc)<0){
		fprintf(stderr,"failed to read IMU temperature registers\n");
		return -1;
	}
	// convert to real units
	data->temp = 21.0 + adc/TEMP_SENSITIVITY;
	return 0;
}
 
/*******************************************************************************
* int reset_mpu9250()
*
* sets the reset bit in the power management register which restores
* the device to defualt settings. a 0.1 second wait is also included
* to let the device compelete the reset process.
*******************************************************************************/
int reset_mpu9250(){
	// disable the interrupt to prevent it from doing things while we reset
	shutdown_interrupt_thread = 1;
	// set the device address
	rc_i2c_set_device_address(IMU_BUS, IMU_ADDR);
	// write the reset bit
	if(rc_i2c_write_byte(IMU_BUS, PWR_MGMT_1, H_RESET)){
		// wait and try again
		rc_usleep(10000);
			if(rc_i2c_write_byte(IMU_BUS, PWR_MGMT_1, H_RESET)){
				fprintf(stderr,"I2C write to MPU9250 Failed\n");
			return -1;
		}
	}
	// make sure all other power management features are off
	if(rc_i2c_write_byte(IMU_BUS, PWR_MGMT_1, 0)){
		// wait and try again
		rc_usleep(10000);
		if(rc_i2c_write_byte(IMU_BUS, PWR_MGMT_1, 0)){
			fprintf(stderr,"I2C write to MPU9250 Failed\n");
		return -1;
		}
	}
	rc_usleep(100000);
	return 0;
}

/*******************************************************************************
* int set_gyro_fsr(rc_gyro_fsr_t fsr, rc_imu_data_t* data)
* 
* set gyro full scale range and update conversion ratio
*******************************************************************************/
int set_gyro_fsr(rc_gyro_fsr_t fsr, rc_imu_data_t* data){
	uint8_t c;
	switch(fsr){
	case G_FSR_250DPS:
		c = GYRO_FSR_CFG_250 | FCHOICE_B_DLPF_EN;
		data->gyro_to_degs = 250.0/32768.0;
		break;
	case G_FSR_500DPS:
		c = GYRO_FSR_CFG_500 | FCHOICE_B_DLPF_EN;
		data->gyro_to_degs = 500.0/32768.0;
		break;
	case G_FSR_1000DPS:
		c = GYRO_FSR_CFG_1000 | FCHOICE_B_DLPF_EN;
		data->gyro_to_degs = 1000.0/32768.0;
		break;
	case G_FSR_2000DPS:
		c = GYRO_FSR_CFG_2000 | FCHOICE_B_DLPF_EN;
		data->gyro_to_degs = 2000.0/32768.0;
		break;
	default:
		fprintf(stderr,"invalid gyro fsr\n");
		return -1;
	}
	return rc_i2c_write_byte(IMU_BUS, GYRO_CONFIG, c);
}

/*******************************************************************************
* int set_accel_fsr(rc_accel_fsr_t fsr, rc_imu_data_t* data)
* 
* set accelerometer full scale range and update conversion ratio
*******************************************************************************/
int set_accel_fsr(rc_accel_fsr_t fsr, rc_imu_data_t* data){
	uint8_t c;
	switch(fsr){
	case A_FSR_2G:
		c = ACCEL_FSR_CFG_2G;
		data->accel_to_ms2 = 9.80665*2.0/32768.0;
		break;
	case A_FSR_4G:
		c = ACCEL_FSR_CFG_4G;
		data->accel_to_ms2 = 9.80665*4.0/32768.0;
		break;
	case A_FSR_8G:
		c = ACCEL_FSR_CFG_8G;
		data->accel_to_ms2 = 9.80665*8.0/32768.0;
		break;
	case A_FSR_16G:
		c = ACCEL_FSR_CFG_16G;
		data->accel_to_ms2 = 9.80665*16.0/32768.0;
		break;
	default:
		fprintf(stderr,"invalid accel fsr\n");
		return -1;
	}
	return rc_i2c_write_byte(IMU_BUS, ACCEL_CONFIG, c);
}

/*******************************************************************************
* int set_gyro_dlpf(rc_gyro_dlpf_t dlpf)
*
* Set GYRO low pass filter constants. This is the same register as
* the fifo overflow mode so we set it to keep the newest data too.
*******************************************************************************/
int set_gyro_dlpf(rc_gyro_dlpf_t dlpf){ 
	uint8_t c = FIFO_MODE_REPLACE_OLD;
	switch(dlpf){
	case GYRO_DLPF_OFF:
		c |= 1;
		break;
	case GYRO_DLPF_184:
		c |= 1;
		break;
	case GYRO_DLPF_92:
		c |= 2;
		break;
	case GYRO_DLPF_41:
		c |= 3;
		break;
	case GYRO_DLPF_20:
		c |= 4;
		break;
	case GYRO_DLPF_10:
		c |= 5;
		break;
	case GYRO_DLPF_5:
		c |= 6;
		break;
	default:
		fprintf(stderr,"invalid gyro_dlpf\n");
		return -1;
	}
	return rc_i2c_write_byte(IMU_BUS, CONFIG, c); 
}

/*******************************************************************************
* int set_accel_dlpf(rc_accel_dlpf_t dlpf)
*
* Set accel low pass filter constants. This is the same register as
* the sample rate. We set it at 1khz as 4khz is unnecessary.
*******************************************************************************/
int set_accel_dlpf(rc_accel_dlpf_t dlpf){
	uint8_t c = ACCEL_FCHOICE_1KHZ | BIT_FIFO_SIZE_1024;
	switch(dlpf){
	case ACCEL_DLPF_OFF:
		c |= 7;
		break;
	case ACCEL_DLPF_184:
		c |= 1;
		break;
	case ACCEL_DLPF_92:
		c |= 2;
		break;
	case ACCEL_DLPF_41:
		c |= 3;
		break;
	case ACCEL_DLPF_20:
		c |= 4;
		break;
	case ACCEL_DLPF_10:
		c |= 5;
		break;
	case ACCEL_DLPF_5:
		c |= 6;
		break;
	default:
		fprintf(stderr,"invalid gyro_dlpf\n");
		return -1;
	}
	return rc_i2c_write_byte(IMU_BUS, ACCEL_CONFIG_2, c);
}

/*******************************************************************************
* int initialize_magnetometer()
*
* configure the magnetometer for 100hz reads, also reads in the factory
* sensitivity values into the global variables;
*******************************************************************************/
int initialize_magnetometer(){
	uint8_t raw[3];  // calibration data stored here
	
	rc_i2c_set_device_address(IMU_BUS, IMU_ADDR);
	// Enable i2c bypass to allow talking to magnetometer
	if(mpu_set_bypass(1)){
		fprintf(stderr,"failed to set mpu9250 into bypass i2c mode\n");
		return -1;
	}
	// magnetometer is actually a separate device with its
	// own address inside the mpu9250
	rc_i2c_set_device_address(IMU_BUS, AK8963_ADDR);
	// Power down magnetometer  
	rc_i2c_write_byte(IMU_BUS, AK8963_CNTL, MAG_POWER_DN); 
	rc_usleep(1000);
	// Enter Fuse ROM access mode
	rc_i2c_write_byte(IMU_BUS, AK8963_CNTL, MAG_FUSE_ROM); 
	rc_usleep(1000);
	// Read the xyz sensitivity adjustment values
	if(rc_i2c_read_bytes(IMU_BUS, AK8963_ASAX, 3, &raw[0])<0){
		fprintf(stderr,"failed to read magnetometer adjustment register\n");
		rc_i2c_set_device_address(IMU_BUS, IMU_ADDR);
		mpu_set_bypass(0);
		return -1;
	}
	// Return sensitivity adjustment values
	mag_factory_adjust[0] = (raw[0]-128)/256.0 + 1.0;   
	mag_factory_adjust[1] = (raw[1]-128)/256.0 + 1.0;  
	mag_factory_adjust[2] = (raw[2]-128)/256.0 + 1.0; 
	// Power down magnetometer again
	rc_i2c_write_byte(IMU_BUS, AK8963_CNTL, MAG_POWER_DN); 
	rc_usleep(100);
	// Configure the magnetometer for 16 bit resolution 
	// and continuous sampling mode 2 (100hz)
	uint8_t c = MSCALE_16|MAG_CONT_MES_2;
	rc_i2c_write_byte(IMU_BUS, AK8963_CNTL, c);
	rc_usleep(100);
	// go back to configuring the IMU, leave bypass on
	rc_i2c_set_device_address(IMU_BUS,IMU_ADDR);
	// load in magnetometer calibration
	load_mag_calibration();
	return 0;
}

/*******************************************************************************
* int power_down_magnetometer()
*
* Make sure the magnetometer is off.
*******************************************************************************/
int power_down_magnetometer(){
	rc_i2c_set_device_address(IMU_BUS, IMU_ADDR);
	// Enable i2c bypass to allow talking to magnetometer
	if(mpu_set_bypass(1)){
		fprintf(stderr,"failed to set mpu9250 into bypass i2c mode\n");
		return -1;
	}
	// magnetometer is actually a separate device with its
	// own address inside the mpu9250
	rc_i2c_set_device_address(IMU_BUS, AK8963_ADDR);
	// Power down magnetometer  
	if(rc_i2c_write_byte(IMU_BUS, AK8963_CNTL, MAG_POWER_DN)<0){
		fprintf(stderr,"failed to write to magnetometer\n");
		return -1;
	}
	rc_i2c_set_device_address(IMU_BUS, IMU_ADDR);
	// Enable i2c bypass to allow talking to magnetometer
	if(mpu_set_bypass(0)){
		fprintf(stderr,"failed to set mpu9250 into bypass i2c mode\n");
		return -1;
	}
	return 0;
}

/*******************************************************************************
*	Power down the IMU
*******************************************************************************/
int rc_power_off_imu(){
	shutdown_interrupt_thread = 1;
	// set the device address
	rc_i2c_set_device_address(IMU_BUS, IMU_ADDR);
	// write the reset bit
	if(rc_i2c_write_byte(IMU_BUS, PWR_MGMT_1, H_RESET)){
		//wait and try again
		rc_usleep(1000);
		if(rc_i2c_write_byte(IMU_BUS, PWR_MGMT_1, H_RESET)){
			fprintf(stderr,"I2C write to MPU9250 Failed\n");
			return -1;
		}
	}
	// write the sleep bit
	if(rc_i2c_write_byte(IMU_BUS, PWR_MGMT_1, MPU_SLEEP)){
		//wait and try again
		rc_usleep(1000);
		if(rc_i2c_write_byte(IMU_BUS, PWR_MGMT_1, MPU_SLEEP)){
			fprintf(stderr,"I2C write to MPU9250 Failed\n");
			return -1;
		}
	}
	// wait for the interrupt thread to exit if it hasn't already
	//allow up to 1 second for thread cleanup
	if(thread_running_flag){
		struct timespec thread_timeout;
		clock_gettime(CLOCK_REALTIME, &thread_timeout);
		thread_timeout.tv_sec += 1;
		int thread_err = 0;
		thread_err = pthread_timedjoin_np(imu_interrupt_thread, NULL, \
															&thread_timeout);
		if(thread_err == ETIMEDOUT){
			fprintf(stderr,"WARNING: imu_interrupt_thread exit timeout\n");
		}
	}
	return 0;
}

/*******************************************************************************
*	Set up the IMU for DMP accelerated filtering and interrupts
*******************************************************************************/
int rc_initialize_imu_dmp(rc_imu_data_t *data, rc_imu_config_t conf){
	uint8_t c;
	// range check
	if(conf.dmp_sample_rate>DMP_MAX_RATE || conf.dmp_sample_rate<DMP_MIN_RATE){
		fprintf(stderr,"ERROR:dmp_sample_rate must be between %d & %d\n", \
												DMP_MIN_RATE, DMP_MAX_RATE);
		return -1;
	}
	// make sure the sample rate is a divisor so we can find a neat rate divider
	if(DMP_MAX_RATE%conf.dmp_sample_rate != 0){
		fprintf(stderr,"DMP sample rate must be a divisor of 200\n");
		fprintf(stderr,"acceptable values: 200,100,50,40,25,20,10,8,5,4 (HZ)\n");
		return -1;
	}
	// make sure the compass filter time constant is valid
	if(conf.enable_magnetometer && conf.compass_time_constant<=0.1){
		fprintf(stderr,"ERROR: compass time constant must be greater than 0.1\n");
		return -1;
	}
	// make sure the bus is not currently in use by another thread
	// do not proceed to prevent interfering with that process
	if(rc_i2c_get_in_use_state(IMU_BUS)){
		fprintf(stderr,"WARNING: i2c bus claimed by another process\n");
		fprintf(stderr,"Continuing with rc_initialize_imu_dmp() anyway\n");
	}
	// start the i2c bus
	if(rc_i2c_init(IMU_BUS, IMU_ADDR)){
		fprintf(stderr,"rc_initialize_imu_dmp failed at rc_i2c_init\n");
		return -1;
	}
	// configure the gpio interrupt pin
	if(rc_gpio_export(IMU_INTERRUPT_PIN)<0){
		fprintf(stderr,"ERROR: failed to export GPIO %d", IMU_INTERRUPT_PIN);
		return -1;
	}
	if(rc_gpio_set_dir(IMU_INTERRUPT_PIN, INPUT_PIN)<0){
		fprintf(stderr,"ERROR: failed to configure GPIO %d", IMU_INTERRUPT_PIN);
		return -1;
	}
	if(rc_gpio_set_edge(IMU_INTERRUPT_PIN, EDGE_FALLING)<0){
		fprintf(stderr,"ERROR: failed to configure GPIO %d", IMU_INTERRUPT_PIN);
		return -1;
	}
	// claiming the bus does no guarantee other code will not interfere 
	// with this process, but best to claim it so other code can check
	// like we did above
	rc_i2c_claim_bus(IMU_BUS);
	// restart the device so we start with clean registers
	if(reset_mpu9250()<0){
		fprintf(stderr,"failed to reset_mpu9250()\n");
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
	//check the who am i register to make sure the chip is alive
	if(rc_i2c_read_byte(IMU_BUS, WHO_AM_I_MPU9250, &c)<0){
		fprintf(stderr,"i2c_read_byte failed reading who_am_i register\n");
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	} if(c!=0x71){
		fprintf(stderr,"mpu9250 WHO AM I register should return 0x71\n");
		fprintf(stderr,"WHO AM I returned: 0x%x\n", c);
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
	// load in gyro calibration offsets from disk
	if(load_gyro_offets()<0){
		fprintf(stderr,"ERROR: failed to load gyro calibration offsets\n");
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
	// log locally that the dmp will be running
	dmp_en = 1;
	// update local copy of config and data struct with new values
	config = conf;
	data_ptr = data;
	// Set sensor sample rate to 200hz which is max the dmp can do.
	// DMP will divide this frequency down further itself
	if(mpu_set_sample_rate(200)<0){
		fprintf(stderr,"ERROR: setting IMU sample rate\n");
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
	// initialize the magnetometer too if requested in config
	if(conf.enable_magnetometer){
		if(initialize_magnetometer()){
			fprintf(stderr,"ERROR: failed to initialize_magnetometer\n");
			rc_i2c_release_bus(IMU_BUS);
			return -1;
		}
	}
	else power_down_magnetometer();
	// set full scale ranges. It seems the DMP only scales the gyro properly
	// at 2000DPS. I'll assume the same is true for accel and use 2G like their
	// example
	set_gyro_fsr(G_FSR_2000DPS, data_ptr);
	set_accel_fsr(A_FSR_2G, data_ptr);
	// set the user-configurable DLPF
	set_gyro_dlpf(config.gyro_dlpf);
	set_accel_dlpf(config.accel_dlpf);
	// set up the DMP
	if(dmp_load_motion_driver_firmware()<0){
		fprintf(stderr,"failed to load DMP motion driver\n");
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
	if(dmp_set_fifo_rate(config.dmp_sample_rate)<0){
		fprintf(stderr,"ERROR: failed to set DMP fifo rate\n");
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
	// Set fifo/sensor sample rate. Will have to set the DMP sample
	// rate to match this shortly.
	if(dmp_set_orientation((unsigned short)conf.orientation)<0){
		fprintf(stderr,"ERROR: failed to set dmp orientation\n");
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
	if(dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_SEND_RAW_ACCEL| \
												DMP_FEATURE_SEND_RAW_GYRO)<0){
		fprintf(stderr,"ERROR: failed to enable DMP features\n");
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
	if(dmp_set_interrupt_mode(DMP_INT_CONTINUOUS)<0){
		fprintf(stderr,"ERROR: failed to set DMP interrupt mode to continuous\n");
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
	if (mpu_set_dmp_state(1)<0) {
		fprintf(stderr,"ERROR: mpu_set_dmp_state(1) failed\n");
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
	// set up the IMU to put magnetometer data in the fifo too if enabled
	if(conf.enable_magnetometer){
		// enable slave 0 (mag) in fifo
		rc_i2c_write_byte(IMU_BUS,FIFO_EN, FIFO_SLV0_EN);	
		// enable master, and clock speed
		rc_i2c_write_byte(IMU_BUS,I2C_MST_CTRL,	0x8D);
		// set slave 0 address to magnetometer address
		rc_i2c_write_byte(IMU_BUS,I2C_SLV0_ADDR,	0X8C);
		// set mag data register to read from
		rc_i2c_write_byte(IMU_BUS,I2C_SLV0_REG,	AK8963_XOUT_L);
		// set slave 0 to read 7 bytes
		rc_i2c_write_byte(IMU_BUS,I2C_SLV0_CTRL,	0x87);
		packet_len += 7; // add 7 more bytes to the fifo reads
	}
	// done with I2C for now
	rc_i2c_release_bus(IMU_BUS);
	#ifdef DEBUG
	printf("packet_len: %d\n", packet_len);
	#endif
	// start the interrupt handler thread
	interrupt_func_set = 1;
	shutdown_interrupt_thread = 0;
	rc_set_imu_interrupt_func(&rc_null_func);
	pthread_create(&imu_interrupt_thread, NULL, \
					imu_interrupt_handler, (void*) NULL);
	params.sched_priority = config.dmp_interrupt_priority;
	pthread_setschedparam(imu_interrupt_thread, SCHED_FIFO, &params);
	thread_running_flag = 1;
	rc_usleep(1000);
	#ifdef DEBUG
	int policy;
	struct sched_param params_tmp;
	pthread_getschedparam(imu_interrupt_thread, &policy, &params_tmp);
	printf("new policy: %d, fifo: %d, prio: %d\n", policy, SCHED_FIFO, params_tmp.sched_priority);
	#endif
	return 0;
}

/*******************************************************************************
 *  @brief      Write to the DMP memory.
 *  This function prevents I2C writes past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to write.
 *  @param[in]  data        Bytes to write to memory.
 *  @return     0 if successful.
*******************************************************************************/
int mpu_write_mem(unsigned short mem_addr, unsigned short length,\
												unsigned char *data){
	unsigned char tmp[2];
	if (!data){
		fprintf(stderr,"ERROR: in mpu_write_mem, NULL pointer\n");
		return -1;
	}
	tmp[0] = (unsigned char)(mem_addr >> 8);
	tmp[1] = (unsigned char)(mem_addr & 0xFF);
	// Check bank boundaries.
	if (tmp[1] + length > MPU6500_BANK_SIZE){
		fprintf(stderr,"mpu_write_mem exceeds bank size\n");
		return -1;
	}
	if (rc_i2c_write_bytes(IMU_BUS,MPU6500_BANK_SEL, 2, tmp))
		return -1;
	if (rc_i2c_write_bytes(IMU_BUS,MPU6500_MEM_R_W, length, data))
		return -1;
	return 0;
}

/*******************************************************************************
 *  @brief      Read from the DMP memory.
 *  This function prevents I2C reads past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Bytes read from memory.
 *  @return     0 if successful.
*******************************************************************************/
int mpu_read_mem(unsigned short mem_addr, unsigned short length,\
												unsigned char *data){
	unsigned char tmp[2];
	if (!data){
		fprintf(stderr,"ERROR: in mpu_write_mem, NULL pointer\n");
		return -1;
	}
	tmp[0] = (unsigned char)(mem_addr >> 8);
	tmp[1] = (unsigned char)(mem_addr & 0xFF);
	// Check bank boundaries.
	if (tmp[1] + length > MPU6500_BANK_SIZE){
		printf("mpu_read_mem exceeds bank size\n");
		return -1;
	}
	if (rc_i2c_write_bytes(IMU_BUS,MPU6500_BANK_SEL, 2, tmp))
		return -1;
	if (rc_i2c_read_bytes(IMU_BUS,MPU6500_MEM_R_W, length, data)!=length)
		return -1;
	return 0;
}

/*******************************************************************************
* int dmp_load_motion_driver_firmware()
*
* loads pre-compiled firmware binary from invensense onto dmp
*******************************************************************************/
int dmp_load_motion_driver_firmware(){
	unsigned short ii;
	unsigned short this_write;
	// Must divide evenly into st.hw->bank_size to avoid bank crossings.
	unsigned char cur[DMP_LOAD_CHUNK], tmp[2];
	// make sure the address is set correctly
	rc_i2c_set_device_address(IMU_BUS, IMU_ADDR);
	// loop through 16 bytes at a time and check each write for corruption
	for (ii=0; ii<DMP_CODE_SIZE; ii+=this_write) {
		this_write = min(DMP_LOAD_CHUNK, DMP_CODE_SIZE - ii);
		if (mpu_write_mem(ii, this_write, (uint8_t*)&dmp_firmware[ii])){
			fprintf(stderr,"dmp firmware write failed\n");
			return -1;
		}
		if (mpu_read_mem(ii, this_write, cur)){
			fprintf(stderr,"dmp firmware read failed\n");
			return -1;
		}
		if (memcmp(dmp_firmware+ii, cur, this_write)){
			fprintf(stderr,"dmp firmware write corrupted\n");
			return -2;
		}
	}
	// Set program start address.
	tmp[0] = dmp_start_addr >> 8;
	tmp[1] = dmp_start_addr & 0xFF;
	if (rc_i2c_write_bytes(IMU_BUS, MPU6500_PRGM_START_H, 2, tmp)){
		fprintf(stderr,"ERROR writing to MPU6500_PRGM_START register\n");
		return -1;
	}
	return 0;
}

/*******************************************************************************
 *  @brief      Push gyro and accel orientation to the DMP.
 *  The orientation is represented here as the output of
 *  @e inv_orientation_matrix_to_scalar.
 *  @param[in]  orient  Gyro and accel orientation in body frame.
 *  @return     0 if successful.
*******************************************************************************/
int dmp_set_orientation(unsigned short orient){
	unsigned char gyro_regs[3], accel_regs[3];
	const unsigned char gyro_axes[3] = {DINA4C, DINACD, DINA6C};
	const unsigned char accel_axes[3] = {DINA0C, DINAC9, DINA2C};
	const unsigned char gyro_sign[3] = {DINA36, DINA56, DINA76};
	const unsigned char accel_sign[3] = {DINA26, DINA46, DINA66};
	// populate fata to be written
	gyro_regs[0] = gyro_axes[orient & 3];
	gyro_regs[1] = gyro_axes[(orient >> 3) & 3];
	gyro_regs[2] = gyro_axes[(orient >> 6) & 3];
	accel_regs[0] = accel_axes[orient & 3];
	accel_regs[1] = accel_axes[(orient >> 3) & 3];
	accel_regs[2] = accel_axes[(orient >> 6) & 3];
	// Chip-to-body, axes only.
	if (mpu_write_mem(FCFG_1, 3, gyro_regs)){
		fprintf(stderr, "ERROR: in dmp_set_orientation, failed to write dmp mem\n");
		return -1;
	}
	if (mpu_write_mem(FCFG_2, 3, accel_regs)){
		fprintf(stderr, "ERROR: in dmp_set_orientation, failed to write dmp mem\n");
		return -1;
	}
	memcpy(gyro_regs, gyro_sign, 3);
	memcpy(accel_regs, accel_sign, 3);
	if (orient & 4) {
		gyro_regs[0] |= 1;
		accel_regs[0] |= 1;
	}
	if (orient & 0x20) {
		gyro_regs[1] |= 1;
		accel_regs[1] |= 1;
	}
	if (orient & 0x100) {
		gyro_regs[2] |= 1;
		accel_regs[2] |= 1;
	}
	// Chip-to-body, sign only.
	if(mpu_write_mem(FCFG_3, 3, gyro_regs)){
		fprintf(stderr, "ERROR: in dmp_set_orientation, failed to write dmp mem\n");
		return -1;
	}
	if(mpu_write_mem(FCFG_7, 3, accel_regs)){
		fprintf(stderr, "ERROR: in dmp_set_orientation, failed to write dmp mem\n");
		return -1;
	}
	return 0;
}

/*******************************************************************************
 *  @brief      Set DMP output rate.
 *  Only used when DMP is on.
 *  @param[in]  rate    Desired fifo rate (Hz).
 *  @return     0 if successful.
*******************************************************************************/
int dmp_set_fifo_rate(unsigned short rate){
	const unsigned char regs_end[12] = {DINAFE, DINAF2, DINAAB,
		0xc4, DINAAA, DINAF1, DINADF, DINADF, 0xBB, 0xAF, DINADF, DINADF};
	unsigned short div;
	unsigned char tmp[8];
	if (rate > DMP_MAX_RATE){
		return -1;
	}
	// set the DMP scaling factors
	div = DMP_MAX_RATE / rate - 1;
	tmp[0] = (unsigned char)((div >> 8) & 0xFF);
	tmp[1] = (unsigned char)(div & 0xFF);
	if (mpu_write_mem(D_0_22, 2, tmp)){
		fprintf(stderr,"ERROR: writing dmp sample rate reg");
		return -1;
	}
	if (mpu_write_mem(CFG_6, 12, (unsigned char*)regs_end)){
		fprintf(stderr,"ERROR: writing dmp regs_end");
		return -1;
	}
	return 0;
}

/*******************************************************************************
* int mpu_set_bypass(unsigned char bypass_on)
* 
* configures the USER_CTRL and INT_PIN_CFG registers to turn on and off the
* i2c bypass mode for talking to the magnetometer. In random read mode this
* is used to turn on the bypass and left as is. In DMP mode bypass is turned
* off after configuration and the MPU fetches magnetometer data automatically.
* USER_CTRL - based on global variable dsp_en
* INT_PIN_CFG based on requested bypass state
*******************************************************************************/
int mpu_set_bypass(uint8_t bypass_on){
	uint8_t tmp = 0;
	// set up USER_CTRL first
	if(dmp_en){
		tmp |= FIFO_EN_BIT; // enable fifo for dsp mode
	}
	if(!bypass_on){
		tmp |= I2C_MST_EN; // i2c master mode when not in bypass
	}
	if (rc_i2c_write_byte(IMU_BUS, USER_CTRL, tmp)){
		fprintf(stderr,"ERROR in mpu_set_bypass, failed to write USER_CTRL register\n");
		return -1;
	}
	rc_usleep(3000);
	// INT_PIN_CFG settings
	tmp = LATCH_INT_EN | INT_ANYRD_CLEAR | ACTL_ACTIVE_LOW;
	tmp =  ACTL_ACTIVE_LOW;
	if(bypass_on)
		tmp |= BYPASS_EN;
	if (rc_i2c_write_byte(IMU_BUS, INT_PIN_CFG, tmp)){
		fprintf(stderr,"ERROR in mpu_set_bypass, failed to write INT_PIN_CFG register\n");
		return -1;
	}
	if(bypass_on){
		bypass_en = 1;
	}
	else{
		bypass_en = 0;
	}
	return 0;
}

/*******************************************************************************
* int dmp_enable_feature(unsigned short mask)
*
* This is mostly taken from the Invensense DMP code and serves to turn on and
* off DMP features based on the feature mask. We modified to remove some 
* irrelevant features and set our own fifo-length variable. This probably
* isn't necessary to remain in its current form as rc_initialize_imu_dmp uses
* a fixed set of features but we keep it as is since it works fine.
*******************************************************************************/
int dmp_enable_feature(unsigned short mask){
	unsigned char tmp[10];
	// Set integration scale factor.
	tmp[0] = (unsigned char)((GYRO_SF >> 24) & 0xFF);
	tmp[1] = (unsigned char)((GYRO_SF >> 16) & 0xFF);
	tmp[2] = (unsigned char)((GYRO_SF >> 8) & 0xFF);
	tmp[3] = (unsigned char)(GYRO_SF & 0xFF);
	if(mpu_write_mem(D_0_104, 4, tmp)<0){
		fprintf(stderr, "ERROR: in dmp_enable_feature, failed to write mpu mem\n");
		return -1;
	}
	// Send sensor data to the FIFO.
	tmp[0] = 0xA3;
	if (mask & DMP_FEATURE_SEND_RAW_ACCEL) {
		tmp[1] = 0xC0;
		tmp[2] = 0xC8;
		tmp[3] = 0xC2;
	} else {
		tmp[1] = 0xA3;
		tmp[2] = 0xA3;
		tmp[3] = 0xA3;
	}
	if (mask & DMP_FEATURE_SEND_ANY_GYRO) {
		tmp[4] = 0xC4;
		tmp[5] = 0xCC;
		tmp[6] = 0xC6;
	} else {
		tmp[4] = 0xA3;
		tmp[5] = 0xA3;
		tmp[6] = 0xA3;
	}
	tmp[7] = 0xA3;
	tmp[8] = 0xA3;
	tmp[9] = 0xA3;
	if(mpu_write_mem(CFG_15,10,tmp)<0){
		fprintf(stderr, "ERROR: in dmp_enable_feature, failed to write mpu mem\n");
		return -1;
	}
	// Send gesture data to the FIFO.
	if (mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT)){
		tmp[0] = DINA20;
	}
	else{
		tmp[0] = 0xD8;
	}
	if(mpu_write_mem(CFG_27,1,tmp)){
		fprintf(stderr, "ERROR: in dmp_enable_feature, failed to write mpu mem\n");
		return -1;
	}
	if(mask & DMP_FEATURE_GYRO_CAL){
		dmp_enable_gyro_cal(1);
	}
	else{
		dmp_enable_gyro_cal(0);
	}
	if (mask & DMP_FEATURE_SEND_ANY_GYRO) {
		if (mask & DMP_FEATURE_SEND_CAL_GYRO) {
			tmp[0] = 0xB2;
			tmp[1] = 0x8B;
			tmp[2] = 0xB6;
			tmp[3] = 0x9B;
		} else {
			tmp[0] = DINAC0;
			tmp[1] = DINA80;
			tmp[2] = DINAC2;
			tmp[3] = DINA90;
		}
		mpu_write_mem(CFG_GYRO_RAW_DATA, 4, tmp);
	}
	// disable tap feature
	tmp[0] = 0xD8;
	mpu_write_mem(CFG_20, 1, tmp);
	// disable orientation feature
	tmp[0] = 0xD8;
	mpu_write_mem(CFG_ANDROID_ORIENT_INT, 1, tmp);
	if (mask & DMP_FEATURE_LP_QUAT){
		dmp_enable_lp_quat(1);
	}
	else{
		dmp_enable_lp_quat(0);
	}
	if (mask & DMP_FEATURE_6X_LP_QUAT){
		dmp_enable_6x_lp_quat(1);
	}
	else{
		dmp_enable_6x_lp_quat(0);
	}
	mpu_reset_fifo();
	packet_len = 0;
	if(mask & DMP_FEATURE_SEND_RAW_ACCEL){
		packet_len += 6;
	}
	if(mask & DMP_FEATURE_SEND_ANY_GYRO){
		packet_len += 6;
	}
	if(mask & (DMP_FEATURE_LP_QUAT | DMP_FEATURE_6X_LP_QUAT)){
		packet_len += 16;
	}
	return 0;
}

/*******************************************************************************
* int dmp_enable_gyro_cal(unsigned char enable)
*
* Taken straight from the Invensense DMP code. This enabled the automatic gyro
* calibration feature in the DMP. This this feature is fine for cell phones
* but annoying in control systems we do not use it here and instead ask users
* to run our own gyro_calibration routine.
*******************************************************************************/
int dmp_enable_gyro_cal(unsigned char enable){
	if(enable){
		unsigned char regs[9] = {0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d};
		return mpu_write_mem(CFG_MOTION_BIAS, 9, regs);
	}
	else{
		unsigned char regs[9] = {0xb8, 0xaa, 0xaa, 0xaa, 0xb0, 0x88, 0xc3, 0xc5, 0xc7};
		return mpu_write_mem(CFG_MOTION_BIAS, 9, regs);
	}
}

/*******************************************************************************
* int dmp_enable_6x_lp_quat(unsigned char enable)
*
* Taken straight from the Invensense DMP code. This enabled quaternion filtering
* with accelerometer and gyro filtering.
*******************************************************************************/
int dmp_enable_6x_lp_quat(unsigned char enable){
	unsigned char regs[4];
	if(enable){
		regs[0] = DINA20;
		regs[1] = DINA28;
		regs[2] = DINA30;
		regs[3] = DINA38;
	}
	else{
		memset(regs, 0xA3, 4);
	}
	mpu_write_mem(CFG_8, 4, regs);
	return 0;
}

/*******************************************************************************
* int dmp_enable_lp_quat(unsigned char enable)
*
* sets the DMP to do gyro-only quaternion filtering. This is not actually used
* here but remains as a vestige of the Invensense DMP code.
*******************************************************************************/
int dmp_enable_lp_quat(unsigned char enable){
	unsigned char regs[4];
	if(enable){
		regs[0] = DINBC0;
		regs[1] = DINBC2;
		regs[2] = DINBC4;
		regs[3] = DINBC6;
	}
	else{
		memset(regs, 0x8B, 4);
	}
	mpu_write_mem(CFG_LP_QUAT, 4, regs);
	return 0;
}

/*******************************************************************************
* int mpu_reset_fifo()
*
* This is mostly from the Invensense open source codebase but modified to also
* allow magnetometer data to come in through the FIFO. This just turns off the
* interrupt, resets fifo and DMP, then starts them again. Used once while 
* initializing (probably no necessary) then again if the fifo gets too full.
*******************************************************************************/
int mpu_reset_fifo(void){
	uint8_t data;
	// make sure the i2c address is set correctly. 
	// this shouldn't take any time at all if already set
	rc_i2c_set_device_address(IMU_BUS, IMU_ADDR);
	data = 0;
	if (rc_i2c_write_byte(IMU_BUS, INT_ENABLE, data)) return -1;
	if (rc_i2c_write_byte(IMU_BUS, FIFO_EN, data)) return -1;
	//if (rc_i2c_write_byte(IMU_BUS, USER_CTRL, data)) return -1;
	data = BIT_FIFO_RST | BIT_DMP_RST;
	if (rc_i2c_write_byte(IMU_BUS, USER_CTRL, data)) return -1;
	rc_usleep(1000);
	data = BIT_DMP_EN | BIT_FIFO_EN;
	if(config.enable_magnetometer){
		data |= I2C_MST_EN;
	}
	if(rc_i2c_write_byte(IMU_BUS, USER_CTRL, data)){
		return -1;
	}
	if(config.enable_magnetometer){
		rc_i2c_write_byte(IMU_BUS, FIFO_EN, FIFO_SLV0_EN);
	}
	else{
		rc_i2c_write_byte(IMU_BUS, FIFO_EN, 0);
	}
	if(dmp_en){
		rc_i2c_write_byte(IMU_BUS, INT_ENABLE, BIT_DMP_INT_EN);
	}
	else{
		rc_i2c_write_byte(IMU_BUS, INT_ENABLE, 0);
	}
	return 0;
}

/*******************************************************************************
* int dmp_set_interrupt_mode(unsigned char mode)
* 
* This is from the Invensense open source DMP code. It configures the DMP
* to trigger an interrupt either every sample or only on gestures. Here we
* only ever configure for continuous sampling.
*******************************************************************************/
int dmp_set_interrupt_mode(unsigned char mode){
	const unsigned char regs_continuous[11] =
		{0xd8, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0x09, 0xb4, 0xd9};
	const unsigned char regs_gesture[11] =
		{0xda, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0xda, 0xb4, 0xda};
	switch(mode){
	case DMP_INT_CONTINUOUS:
		return mpu_write_mem(CFG_FIFO_ON_EVENT, 11, (unsigned char*)regs_continuous);
	case DMP_INT_GESTURE:
		return mpu_write_mem(CFG_FIFO_ON_EVENT, 11, (unsigned char*)regs_gesture);
	default:
		return -1;
	}
}

/*******************************************************************************
* int set_int_enable(unsigned char enable)
* 
* This is a vestige of the invensense mpu open source code and is probably
* not necessary but remains here anyway.
*******************************************************************************/
int set_int_enable(unsigned char enable){
	unsigned char tmp;
	if (enable){
		tmp = BIT_DMP_INT_EN;
	}
	else{
		tmp = 0x00;
	}
	if(rc_i2c_write_byte(IMU_BUS, INT_ENABLE, tmp)){
		fprintf(stderr, "ERROR: in set_int_enable, failed to write INT_ENABLE register\n");
		return -1;
	}
	// disable all other FIFO features leaving just DMP
	if (rc_i2c_write_byte(IMU_BUS, FIFO_EN, 0)){
		fprintf(stderr, "ERROR: in set_int_enable, failed to write FIFO_EN register\n");
		return -1;
	}
	return 0;
}

/*******************************************************************************
int mpu_set_sample_rate(int rate)

Sets the clock rate divider for sensor sampling
*******************************************************************************/
int mpu_set_sample_rate(int rate){
	if(rate>1000 || rate<4){
		fprintf(stderr,"ERROR: sample rate must be between 4 & 1000\n");
		return -1;
	}
	/* Keep constant sample rate, FIFO rate controlled by DMP. */
	uint8_t div = (1000/rate) - 1;
	#ifdef DEBUG
	printf("setting divider to %d\n", div);
	#endif
	if(rc_i2c_write_byte(IMU_BUS, SMPLRT_DIV, div)){
		fprintf(stderr,"ERROR: in mpu_set_sample_rate, failed to write SMPLRT_DIV register\n");
		return -1;
	}
	return 0;
}

/*******************************************************************************
*  int mpu_set_dmp_state(unsigned char enable)
* 
* This turns on and off the DMP interrupt and resets the FIFO. This probably
* isn't necessary as rc_initialize_imu_dmp sets these registers but it remains 
* here as a vestige of the invensense open source dmp code.
*******************************************************************************/
int mpu_set_dmp_state(unsigned char enable){
	if (enable) {
		// Disable data ready interrupt.
		set_int_enable(0);
		// Disable bypass mode.
		mpu_set_bypass(0);
		// Remove FIFO elements.
		rc_i2c_write_byte(IMU_BUS, FIFO_EN , 0);
		// Enable DMP interrupt.
		set_int_enable(1);
		mpu_reset_fifo();
	}
	else {
		// Disable DMP interrupt.
		set_int_enable(0);
		// Restore FIFO settings.
		rc_i2c_write_byte(IMU_BUS, FIFO_EN , 0);
		mpu_reset_fifo();
	}
	return 0;
}

/*******************************************************************************
* void* imu_interrupt_handler(void* ptr)
*
* Here is where the magic happens. This function runs as its own thread and 
* monitors the gpio pin IMU_INTERRUPT_PIN with the blocking function call 
* poll(). If a valid interrupt is received from the IMU then mark the timestamp,
* read in the IMU data, and call the user-defined interrupt function if set.
*******************************************************************************/
void* imu_interrupt_handler( __unused void* ptr){ 
	struct pollfd fdset[1];
	int ret;
	char buf[64];
	int first_run = 1;
	int imu_gpio_fd = rc_gpio_fd_open(IMU_INTERRUPT_PIN);
	if(imu_gpio_fd == -1){
		fprintf(stderr,"ERROR: can't open IMU_INTERRUPT_PIN gpio fd\n");
		fprintf(stderr,"aborting imu_interrupt_handler\n");
		return NULL;
	}
	fdset[0].fd = imu_gpio_fd;
	fdset[0].events = POLLPRI;
	// keep running until the program closes
	mpu_reset_fifo();
	while(rc_get_state()!=EXITING && shutdown_interrupt_thread!=1) {
		// system hangs here until IMU FIFO interrupt
		poll(fdset, 1, IMU_POLL_TIMEOUT); 
		if(rc_get_state()==EXITING || shutdown_interrupt_thread==1){
			break;
		}
		else if (fdset[0].revents & POLLPRI) {
			lseek(fdset[0].fd, 0, SEEK_SET);  
			read(fdset[0].fd, buf, 64);
			// interrupt received, mark the timestamp
			last_interrupt_timestamp_nanos = rc_nanos_since_epoch();
			// try to load fifo no matter the claim bus state
			if(rc_i2c_get_in_use_state(IMU_BUS)){
				fprintf(stderr,"WARNING: Something has claimed the I2C bus when an\n");
				fprintf(stderr,"IMU interrupt was received. Reading IMU anyway.\n");
			}

			// aquires bus
			rc_i2c_claim_bus(IMU_BUS);

			// aquires mutex
			pthread_mutex_lock( &rc_imu_read_mutex );

			// read data
			ret = read_dmp_fifo(data_ptr);

			// record if it was successful or not
			if (ret==0) {
			  last_read_successful=1;
			  // signals that a measurement is available
			  pthread_cond_broadcast( &rc_imu_read_condition );
			}
			else
			  last_read_successful=0;
  
			// releases mutex
			pthread_mutex_unlock( &rc_imu_read_mutex );

			// releases bus
			rc_i2c_release_bus(IMU_BUS);
			
			// call the user function if not the first run
			if(first_run == 1){
				first_run = 0;
			}
			else if(interrupt_func_set && last_read_successful){
				imu_interrupt_func(); 
			}
		}
	}
	
	// aquires mutex
	pthread_mutex_lock( &rc_imu_read_mutex );
	// /releases other threads
	pthread_cond_broadcast( &rc_imu_read_condition );
	// releases mutex
	pthread_mutex_unlock( &rc_imu_read_mutex );

	rc_gpio_fd_close(imu_gpio_fd);
	thread_running_flag = 0;
	return 0;
}

/*******************************************************************************
* int rc_set_imu_interrupt_func(void (*func)(void))
*
* sets a user function to be called when new data is read
*******************************************************************************/
int rc_set_imu_interrupt_func(void (*func)(void)){
	if(func==NULL){
		fprintf(stderr,"ERROR: trying to assign NULL pointer to imu_interrupt_func\n");
		return -1;
	}
	imu_interrupt_func = func;
	interrupt_func_set = 1;
	return 0;
}

/*******************************************************************************
* int rc_stop_imu_interrupt_func()
*
* stops the user function from being called when new data is available
*******************************************************************************/
int rc_stop_imu_interrupt_func(){
	interrupt_func_set = 0;
	return 0;
}

/*******************************************************************************
* int read_dmp_fifo(rc_imu_data_t* data)
*
* Reads the FIFO buffer and populates the data struct. Here is where we see 
* bad/empty/double packets due to i2c bus errors and the IMU failing to have
* data ready in time. enabling warnings in the config struct will let this
* function print out warnings when these conditions are detected. If write
* errors are detected then this function tries some i2c transfers a second time.
*******************************************************************************/
int read_dmp_fifo(rc_imu_data_t* data){
	unsigned char raw[MAX_FIFO_BUFFER];
	long quat[4];
	int16_t mag_adc[3];
	uint16_t fifo_count;
	int ret, mag_data_available, dmp_data_available;
	int i = 0; // position of beginning of mag data
	int j = 0; // position of beginning of dmp data
	static int first_run = 1; // set to 0 after first call
	float factory_cal_data[3]; // just temp holder for mag data
	double q_tmp[4];
	double sum,qlen;
	
	if(!dmp_en){
		printf("only use mpu_read_fifo in dmp mode\n");
		return -1;
	}

	// if the fifo packet_len variable not set up yet, this function must
	// have been called prematurely
	if(packet_len!=FIFO_LEN_NO_MAG && packet_len!=FIFO_LEN_MAG){
		fprintf(stderr,"ERROR: packet_len is set incorrectly for read_dmp_fifo\n");
		return -1;
	}
	
	// make sure the i2c address is set correctly. 
	// this shouldn't take any time at all if already set
	rc_i2c_set_device_address(IMU_BUS, IMU_ADDR);
	int is_new_dmp_data = 0;

	// check fifo count register to make sure new data is there
	if(rc_i2c_read_word(IMU_BUS, FIFO_COUNTH, &fifo_count)<0){
		if(config.show_warnings){
			printf("fifo_count i2c error: %s\n",strerror(errno));
		}
		return -1;
	}
	#ifdef DEBUG
	printf("fifo_count: %d\n", fifo_count);
	#endif

	/***************************************************************************
	* now that we see how many values are in the buffer, we have a long list of
	* checks to determine what should be done
	***************************************************************************/
	mag_data_available=0;
	dmp_data_available=0;

	// empty FIFO, just return, nothing else to do
	if(fifo_count==0){
		// if(config.show_warnings&& first_run!=1){
		// 	printf("WARNING: empty fifo\n");
		// }
		return -1;
	}

	// one packet, perfect!
	if(fifo_count==FIFO_LEN_NO_MAG){
		i = 0; // set offset to 0
		mag_data_available=0;
		dmp_data_available=1;
		goto READ_FIFO;
	}
	if(fifo_count==FIFO_LEN_MAG){
		i = 0; // set offset to 0
		mag_data_available=1;
		dmp_data_available=1;
		goto READ_FIFO;
	}

	// these numbers pop up under high stress and represent uneven 
	// combinations of magnetometer and DMP data
	if(fifo_count==42){
		if(config.show_warnings&& first_run!=1){
			printf("warning: packet count 42\n");
		}
		i = 7; // set offset to 7
		mag_data_available=0;
		dmp_data_available=1;
		goto READ_FIFO;
	}
	if(fifo_count==63){
		if(config.show_warnings&& first_run!=1){
			printf("warning: packet count 63\n");
		}
		i = 28; // set offset to 7
		mag_data_available=0;
		dmp_data_available=1;
		goto READ_FIFO;
	}
	if(fifo_count==77){
		if(config.show_warnings&& first_run!=1){
			printf("warning: packet count 77\n");
		}
		i = 42; // set offset to 7
		mag_data_available=0;
		dmp_data_available=1;
		goto READ_FIFO;
	}

	// if exactly 2 packets are there we just missed one (whoops)
	// read both in and set the offset i to one packet length
	// the last packet data will be read normally
	if(fifo_count==2*FIFO_LEN_NO_MAG){
		if(config.show_warnings&& first_run!=1){
			printf("warning: imu fifo contains two packets\n");
		}
		i = FIFO_LEN_NO_MAG; // set offset to beginning of second packet
		mag_data_available=0;
		dmp_data_available=1;
		goto READ_FIFO;
	}
	if(fifo_count==2*FIFO_LEN_MAG){
		if(config.show_warnings&& first_run!=1){
			printf("warning: imu fifo contains two packets\n");
		}
		i = FIFO_LEN_MAG; // set offset to beginning of second packet
		mag_data_available=1;
		dmp_data_available=1;
		goto READ_FIFO;
	}

	// if there are exactly 7 14, or 21 bytes available, there must be magnetometer
	// data but nothing else. read it in anyway
	if(fifo_count==7 || fifo_count==14 || fifo_count==21){
		i = fifo_count - 7;
		mag_data_available = 1;
		dmp_data_available = 0;
		goto READ_FIFO;
	}

	// finally, if we got a weird packet length, reset the fifo
	if(config.show_warnings&& first_run!=1){
		printf("warning: %d bytes in FIFO, expected %d\n", fifo_count,packet_len);
	}
	mpu_reset_fifo();
	return -1;

	/***************************************************************************
	* now we get to the reading section. Here we also have logic to determine
	* the order of the data in the packet as magnetometer data may come before
	* or after the DMP data
	***************************************************************************/
READ_FIFO:
	memset(raw,0,MAX_FIFO_BUFFER);
	// read it in!
	ret = rc_i2c_read_bytes(IMU_BUS, FIFO_R_W, fifo_count, &raw[0]);
	if(ret<0){
		// if i2c_read returned -1 there was an error, try again
		ret = rc_i2c_read_bytes(IMU_BUS, FIFO_R_W, fifo_count, &raw[0]);
	}
	if(ret!=fifo_count){
		if(config.show_warnings){
			fprintf(stderr,"ERROR: failed to read fifo buffer register\n");
			printf("read %d bytes, expected %d\n", ret, packet_len);
		}
		return -1;
	}

	// if dmp data is available we must figure out if it's before or 
	// after the magnetometer data. Usually before.
	if(dmp_data_available){
		if(config.enable_magnetometer && check_quaternion_validity(raw,i+7)){
			j=i+7; // 7 mag bytes before dmp data
		}
		else if(check_quaternion_validity(raw,i)){
			// printf("after\n");
			j=i; // 7 mag bytes after dmp data
			i=i+FIFO_LEN_NO_MAG; // update mag data offset
		}
		else{
			if(config.show_warnings){
				printf("warning: Quaternion out of bounds\n");
				printf("fifo_count: %d\n", fifo_count);
			}
			mpu_reset_fifo();
			return -1;
		}
		// now we can read the quaternion
		// parse the quaternion data from the buffer
		quat[0] = ((long)raw[j+0] << 24) | ((long)raw[j+1] << 16) |
			((long)raw[j+2] << 8) | raw[j+3];
		quat[1] = ((long)raw[j+4] << 24) | ((long)raw[j+5] << 16) |
			((long)raw[j+6] << 8) | raw[j+7];
		quat[2] = ((long)raw[j+8] << 24) | ((long)raw[j+9] << 16) |
			((long)raw[j+10] << 8) | raw[j+11];
		quat[3] = ((long)raw[j+12] << 24) | ((long)raw[j+13] << 16) |
			((long)raw[j+14] << 8) | raw[j+15];
		
		// do double-precision quaternion normalization since the numbers
		// in raw format are huge
		for(i=0;i<4;i++) q_tmp[i]=(double)quat[i];
		sum = 0.0;
		for(i=0;i<4;i++) sum+=q_tmp[i]*q_tmp[i];
		qlen=sqrt(sum);
		for(i=0;i<4;i++) q_tmp[i]/=qlen;
		// make floating point and put in output
		for(i=0;i<4;i++) data->dmp_quat[i]=(float)q_tmp[i];

		// fill in tait-bryan angles to the data struct
		rc_quaternion_to_tb_array(data->dmp_quat, data->dmp_TaitBryan);
		
		j+=16; // increase offset by 16 which was the quaternion size
		
		// Read Accel values and load into imu_data struct
		// Turn the MSB and LSB into a signed 16-bit value
		data->raw_accel[0] = (int16_t)(((uint16_t)raw[j+0]<<8)|raw[j+1]);
		data->raw_accel[1] = (int16_t)(((uint16_t)raw[j+2]<<8)|raw[j+3]);
		data->raw_accel[2] = (int16_t)(((uint16_t)raw[j+4]<<8)|raw[j+5]);
		
		// Fill in real unit values
		data->accel[0] = data->raw_accel[0] * data->accel_to_ms2;
		data->accel[1] = data->raw_accel[1] * data->accel_to_ms2;
		data->accel[2] = data->raw_accel[2] * data->accel_to_ms2;
		j+=6;
		
		// Read gyro values and load into imu_data struct
		// Turn the MSB and LSB into a signed 16-bit value
		data->raw_gyro[0] = (int16_t)(((int16_t)raw[0+j]<<8)|raw[1+j]);
		data->raw_gyro[1] = (int16_t)(((int16_t)raw[2+j]<<8)|raw[3+j]);
		data->raw_gyro[2] = (int16_t)(((int16_t)raw[4+j]<<8)|raw[5+j]);
		// Fill in real unit values
		data->gyro[0] = data->raw_gyro[0] * data->gyro_to_degs;
		data->gyro[1] = data->raw_gyro[1] * data->gyro_to_degs;
		data->gyro[2] = data->raw_gyro[2] * data->gyro_to_degs;

		is_new_dmp_data = 1;
	}

	// if there was magnetometer data try to read it
	if(mag_data_available){
		// Turn the MSB and LSB into a signed 16-bit value
		// Data stored as little Endian
		mag_adc[0] = (int16_t)(((int16_t)raw[i+1]<<8) | raw[i+0]);  
		mag_adc[1] = (int16_t)(((int16_t)raw[i+3]<<8) | raw[i+2]);  
		mag_adc[2] = (int16_t)(((int16_t)raw[i+5]<<8) | raw[i+4]); 
		
		// if the data is non-zero, save it
		// multiply by the sensitivity adjustment and convert to units of 
		// uT micro Teslas. Also correct the coordinate system as someone 
		// in invensense thought it would be bright idea to have the 
		// magnetometer coordiate system aligned differently than the 
		// accelerometer and gyro.... -__-
		if(mag_adc[0]!=0 || mag_adc[1]!=0 || mag_adc[2]!=0){
			// multiply by the sensitivity adjustment and convert to units of uT
			// Also correct the coordinate system as someone in invensense 
			// thought it would be a bright idea to have the magnetometer coordiate
			// system aligned differently than the accelerometer and gyro.... -__-
			factory_cal_data[0] = mag_adc[1]*mag_factory_adjust[1] * MAG_RAW_TO_uT;
			factory_cal_data[1] = mag_adc[0]*mag_factory_adjust[0] * MAG_RAW_TO_uT;
			factory_cal_data[2] = -mag_adc[2]*mag_factory_adjust[2] * MAG_RAW_TO_uT;
		
			// now apply out own calibration, but first make sure we don't 
			// accidentally multiply by zero in case of uninitialized scale factors
			if(mag_scales[0]==0.0) mag_scales[0]=1.0;
			if(mag_scales[1]==0.0) mag_scales[1]=1.0;
			if(mag_scales[2]==0.0) mag_scales[2]=1.0;
			data->mag[0] = (factory_cal_data[0]-mag_offsets[0])*mag_scales[0];
			data->mag[1] = (factory_cal_data[1]-mag_offsets[1])*mag_scales[1];
			data->mag[2] = (factory_cal_data[2]-mag_offsets[2])*mag_scales[2];
		}
	}

	
	
	// run data_fusion to filter yaw with compass if new mag data came in
	if(is_new_dmp_data && config.enable_magnetometer){
		#ifdef DEBUG
		printf("running data_fusion\n");
		#endif
		data_fusion(data);
	}

	// if we finally got dmp data, turn off the first run flag
	if(is_new_dmp_data) first_run=0;

	// finally, our return value is based on the presence of DMP data only
	// even if new magnetometer data was read, the expected timing must come
	// from the DMP samples only
	if(is_new_dmp_data) return 0;
	else return -1;
}

/*******************************************************************************
* We can detect a corrupted FIFO by monitoring the quaternion data and
* ensuring that the magnitude is always normalized to one. This
* shouldn't happen in normal operation, but if an I2C error occurs,
* the FIFO reads might become misaligned.
*
* Let's start by scaling down the quaternion data to avoid long long
* math.
*******************************************************************************/
int check_quaternion_validity(unsigned char* raw, int i){
	long quat_q14[4], quat[4], quat_mag_sq;
	// parse the quaternion data from the buffer
	quat[0] = ((long)raw[i+0] << 24) | ((long)raw[i+1] << 16) |
		((long)raw[i+2] << 8) | raw[i+3];
	quat[1] = ((long)raw[i+4] << 24) | ((long)raw[i+5] << 16) |
		((long)raw[i+6] << 8) | raw[i+7];
	quat[2] = ((long)raw[i+8] << 24) | ((long)raw[i+9] << 16) |
		((long)raw[i+10] << 8) | raw[i+11];
	quat[3] = ((long)raw[i+12] << 24) | ((long)raw[i+13] << 16) |
		((long)raw[i+14] << 8) | raw[i+15];

	
	quat_q14[0] = quat[0] >> 16;
	quat_q14[1] = quat[1] >> 16;
	quat_q14[2] = quat[2] >> 16;
	quat_q14[3] = quat[3] >> 16;
	quat_mag_sq = quat_q14[0] * quat_q14[0] + quat_q14[1] * quat_q14[1] +
		quat_q14[2] * quat_q14[2] + quat_q14[3] * quat_q14[3];
	if ((quat_mag_sq < QUAT_MAG_SQ_MIN) ||(quat_mag_sq > QUAT_MAG_SQ_MAX)){
		return 0;
	}
	if ((quat_mag_sq < QUAT_MAG_SQ_MIN) ||(quat_mag_sq > QUAT_MAG_SQ_MAX)){
		return 0;
	}
	return 1;
}

/*******************************************************************************
* int data_fusion(rc_imu_data_t* data)
*
* This fuses the magnetometer data with the quaternion straight from the DMP
* to correct the yaw heading to a compass heading. Much thanks to Pansenti for
* open sourcing this routine. In addition to the Pansenti implementation I also
* correct the magnetometer data for DMP orientation, initialize yaw with the
* magnetometer to prevent initial rise time, and correct the yaw_mixing_factor
* with the sample rate so the filter rise time remains constant with different
* sample rates.
*******************************************************************************/
int data_fusion(rc_imu_data_t* data){
	float tilt_tb[3], tilt_q[4], mag_vec[3];
	static float newMagYaw = 0;
	static float newDMPYaw = 0;
	float lastDMPYaw, lastMagYaw, newYaw; 
	static int dmp_spin_counter = 0;
	static int mag_spin_counter = 0;
	static int first_run = 1; // set to 0 after first call to this function
	
	
	// start by filling in the roll/pitch components of the fused euler
	// angles from the DMP generated angles. Ignore yaw for now, we have to
	// filter that later. 
	tilt_tb[0] = data->dmp_TaitBryan[TB_PITCH_X];
	tilt_tb[1] = data->dmp_TaitBryan[TB_ROLL_Y];
	tilt_tb[2] = 0.0f;

	// generate a quaternion rotation of just roll/pitch
	rc_tb_to_quaternion_array(tilt_tb,tilt_q);

	// create a quaternion vector from the current magnetic field vector
	// in IMU body coordinate frame. Since the DMP quaternion is aligned with
	// a particular orientation, we must be careful to orient the magnetometer
	// data to match.
	switch(config.orientation){
	case ORIENTATION_Z_UP:
		mag_vec[0] = data->mag[TB_PITCH_X];
		mag_vec[1] = data->mag[TB_ROLL_Y];
		mag_vec[2] = data->mag[TB_YAW_Z];
		break;
	case ORIENTATION_Z_DOWN:
		mag_vec[0] = -data->mag[TB_PITCH_X];
		mag_vec[1] = data->mag[TB_ROLL_Y];
		mag_vec[2] = -data->mag[TB_YAW_Z];
		break;
	case ORIENTATION_X_UP:
		mag_vec[0] = data->mag[TB_YAW_Z];
		mag_vec[1] = data->mag[TB_ROLL_Y];
		mag_vec[2] = data->mag[TB_PITCH_X];
		break;
	case ORIENTATION_X_DOWN:
		mag_vec[0] = -data->mag[TB_YAW_Z];
		mag_vec[1] = data->mag[TB_ROLL_Y];
		mag_vec[2] = -data->mag[TB_PITCH_X];
		break;
	case ORIENTATION_Y_UP:
		mag_vec[0] = data->mag[TB_PITCH_X];
		mag_vec[1] = -data->mag[TB_YAW_Z];
		mag_vec[2] = data->mag[TB_ROLL_Y];
		break;
	case ORIENTATION_Y_DOWN:
		mag_vec[0] = data->mag[TB_PITCH_X];
		mag_vec[1] = data->mag[TB_YAW_Z];
		mag_vec[2] = -data->mag[TB_ROLL_Y];
		break;
	case ORIENTATION_X_FORWARD:
		mag_vec[0] = data->mag[TB_ROLL_Y];
		mag_vec[1] = -data->mag[TB_PITCH_X];
		mag_vec[2] = data->mag[TB_YAW_Z];
		break;
	case ORIENTATION_X_BACK:
		mag_vec[0] = -data->mag[TB_ROLL_Y];
		mag_vec[1] = data->mag[TB_PITCH_X];
		mag_vec[2] = data->mag[TB_YAW_Z];
		break;
	default:
		fprintf(stderr,"ERROR: invalid orientation\n");
		return -1;
	}
	// tilt that vector by the roll/pitch of the IMU to align magnetic field
	// vector such that Z points vertically
	rc_quaternion_rotate_vector_array(mag_vec,tilt_q);
	// from the aligned magnetic field vector, find a yaw heading
	// check for validity and make sure the heading is positive
	lastMagYaw = newMagYaw; // save from last loop
	newMagYaw = -atan2(mag_vec[1], mag_vec[0]);
	if (newMagYaw != newMagYaw) {
		#ifdef WARNINGS
		printf("newMagYaw NAN\n");
		#endif
		return -1;
	}
	data->compass_heading_raw = newMagYaw;
	// save DMP last from time and record newDMPYaw for this time
	lastDMPYaw = newDMPYaw;
	newDMPYaw = data->dmp_TaitBryan[TB_YAW_Z];
	
	// the outputs from atan2 and dmp are between -PI and PI.
	// for our filters to run smoothly, we can't have them jump between -PI
	// to PI when doing a complete spin. Therefore we check for a skip and 
	// increment or decrement the spin counter
	if(newMagYaw-lastMagYaw < -PI) mag_spin_counter++;
	else if (newMagYaw-lastMagYaw > PI) mag_spin_counter--;
	if(newDMPYaw-lastDMPYaw < -PI) dmp_spin_counter++;
	else if (newDMPYaw-lastDMPYaw > PI) dmp_spin_counter--;
	
	// if this is the first run, set up filters
	if(first_run){
		lastMagYaw = newMagYaw;
		lastDMPYaw = newDMPYaw;
		mag_spin_counter = 0;
		dmp_spin_counter = 0;
		// generate complementary filters
		float dt = 1.0/config.dmp_sample_rate;
		rc_first_order_lowpass(&low_pass,dt,config.compass_time_constant);
		rc_first_order_highpass(&high_pass,dt,config.compass_time_constant);
		rc_prefill_filter_inputs(&low_pass,newMagYaw);
		rc_prefill_filter_outputs(&low_pass,newMagYaw);
		rc_prefill_filter_inputs(&high_pass,newDMPYaw);
		rc_prefill_filter_outputs(&high_pass,0);
		first_run = 0;
	}
	
	// new Yaw is the sum of low and high pass complementary filters.
	newYaw = rc_march_filter(&low_pass,newMagYaw+(TWO_PI*mag_spin_counter)) \
			+ rc_march_filter(&high_pass,newDMPYaw+(TWO_PI*dmp_spin_counter));
			
	newYaw = fmod(newYaw,TWO_PI); // remove the effect of the spins
	if (newYaw > PI) newYaw -= TWO_PI; // bound between +- PI
	else if (newYaw < -PI) newYaw += TWO_PI; // bound between +- PI

	// TB angles expect a yaw between -pi to pi so slide it again and
	// store in the user-accessible fused tb angle
	data->compass_heading = newYaw;
	data->fused_TaitBryan[2] = newYaw;
	data->fused_TaitBryan[0] = data->dmp_TaitBryan[0];
	data->fused_TaitBryan[1] = data->dmp_TaitBryan[1];

	// Also generate a new quaternion from the filtered tb angles
	rc_tb_to_quaternion_array(data->fused_TaitBryan, data->fused_quat);
	return 0;
}

/*******************************************************************************
* int write_gyro_offsets_to_disk(int16_t offsets[3])
*
* Reads steady state gyro offsets from the disk and puts them in the IMU's 
* gyro offset register. If no calibration file exists then make a new one.
*******************************************************************************/
int write_gyro_offets_to_disk(int16_t offsets[3]){
	FILE *cal;
	char file_path[100];

	// construct a new file path string and open for writing
	strcpy(file_path, CONFIG_DIRECTORY);
	strcat(file_path, GYRO_CAL_FILE);
	cal = fopen(file_path, "w+");
	// if opening for writing failed, the directory may not exist yet
	if (cal == 0) {
		mkdir(CONFIG_DIRECTORY, 0777);
		cal = fopen(file_path, "w+");
		if (cal == 0){
			fprintf(stderr,"could not open config directory\n");
			fprintf(stderr,CONFIG_DIRECTORY);
			fprintf(stderr,"\n");
			return -1;
		}
	}
	// write to the file, close, and exit
	if(fprintf(cal,"%d\n%d\n%d\n", offsets[0],offsets[1],offsets[2])<0){
		printf("Failed to write gyro offsets to file\n");
		fclose(cal);
		return -1;
	}
	fclose(cal);
	return 0;
}

/*******************************************************************************
* int load_gyro_offsets()
*
* Loads steady state gyro offsets from the disk and puts them in the IMU's 
* gyro offset register. If no calibration file exists then make a new one.
*******************************************************************************/
int load_gyro_offets(){
	FILE *cal;
	char file_path[100];
	uint8_t data[6];
	int x,y,z;
	
	// construct a new file path string and open for reading
	strcpy (file_path, CONFIG_DIRECTORY);
	strcat (file_path, GYRO_CAL_FILE);
	cal = fopen(file_path, "r");
	
	if (cal == 0) {
		// calibration file doesn't exist yet
		fprintf(stderr,"WARNING: no gyro calibration data found\n");
		fprintf(stderr,"Please run rc_calibrate_gyro\n\n");
		// use zero offsets
		x = 0;
		y = 0;
		z = 0;
	}
	else {
		// read in data
		fscanf(cal,"%d\n%d\n%d\n", &x,&y,&z);
		fclose(cal);
	}

	#ifdef DEBUG
	printf("offsets: %d %d %d\n", x, y, z);
	#endif

	// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input 
	// format. also make negative since we wish to subtract out the steady 
	// state offset
	data[0] = (-x/4  >> 8) & 0xFF; 
	data[1] = (-x/4)       & 0xFF; 
	data[2] = (-y/4  >> 8) & 0xFF;
	data[3] = (-y/4)       & 0xFF;
	data[4] = (-z/4  >> 8) & 0xFF;
	data[5] = (-z/4)       & 0xFF;

	// Push gyro biases to hardware registers
	if(rc_i2c_write_bytes(IMU_BUS, XG_OFFSET_H, 6, &data[0])){
		fprintf(stderr,"ERROR: failed to load gyro offsets into IMU register\n");
		return -1;
	}
	return 0;
}

/*******************************************************************************
* int rc_calibrate_gyro_routine()
*
* Initializes the IMU and samples the gyro for a short period to get steady
* state gyro offsets. These offsets are then saved to disk for later use.
*******************************************************************************/
int rc_calibrate_gyro_routine(){
	uint8_t c, data[6];
	int32_t gyro_sum[3] = {0, 0, 0};
	int16_t offsets[3];
	int was_last_steady = 1;
	
	// make sure the bus is not currently in use by another thread
	// do not proceed to prevent interfering with that process
	if(rc_i2c_get_in_use_state(IMU_BUS)){
		fprintf(stderr,"i2c bus claimed by another process\n");
		fprintf(stderr,"aborting gyro calibration()\n");
		return -1;
	}
	
	// if it is not claimed, start the i2c bus
	if(rc_i2c_init(IMU_BUS, IMU_ADDR)){
		fprintf(stderr,"rc_initialize_imu_dmp failed at rc_i2c_init\n");
		return -1;
	}
	
	// claiming the bus does no guarantee other code will not interfere 
	// with this process, but best to claim it so other code can check
	// like we did above
	rc_i2c_claim_bus(IMU_BUS);
	
	// reset device, reset all registers
	if(reset_mpu9250()<0){
		fprintf(stderr,"ERROR: failed to reset MPU9250\n");
		return -1;
	}

	// set up the IMU specifically for calibration. 
	rc_i2c_write_byte(IMU_BUS, PWR_MGMT_1, 0x01);  
	rc_i2c_write_byte(IMU_BUS, PWR_MGMT_2, 0x00); 
	rc_usleep(200000);
	
	// // set bias registers to 0
	// // Push gyro biases to hardware registers
	// uint8_t zeros[] = {0,0,0,0,0,0};
	// if(rc_i2c_write_bytes(IMU_BUS, XG_OFFSET_H, 6, zeros)){
		// fprintf(stderr,"ERROR: failed to load gyro offsets into IMU register\n");
		// return -1;
	// }

	rc_i2c_write_byte(IMU_BUS, INT_ENABLE, 0x00);  // Disable all interrupts
	rc_i2c_write_byte(IMU_BUS, FIFO_EN, 0x00);     // Disable FIFO
	rc_i2c_write_byte(IMU_BUS, PWR_MGMT_1, 0x00);  // Turn on internal clock source
	rc_i2c_write_byte(IMU_BUS, I2C_MST_CTRL, 0x00);// Disable I2C master
	rc_i2c_write_byte(IMU_BUS, USER_CTRL, 0x00);   // Disable FIFO and I2C master
	rc_i2c_write_byte(IMU_BUS, USER_CTRL, 0x0C);   // Reset FIFO and DMP
	rc_usleep(15000);

	// Configure MPU9250 gyro and accelerometer for bias calculation
	rc_i2c_write_byte(IMU_BUS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	rc_i2c_write_byte(IMU_BUS, SMPLRT_DIV, 0x04);  // Set sample rate to 200hz
	// Set gyro full-scale to 250 degrees per second, maximum sensitivity
	rc_i2c_write_byte(IMU_BUS, GYRO_CONFIG, 0x00); 
	// Set accelerometer full-scale to 2 g, maximum sensitivity	
	rc_i2c_write_byte(IMU_BUS, ACCEL_CONFIG, 0x00); 

COLLECT_DATA:

	if(rc_get_state()==EXITING){
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}

	// Configure FIFO to capture gyro data for bias calculation
	rc_i2c_write_byte(IMU_BUS, USER_CTRL, 0x40);   // Enable FIFO  
	// Enable gyro sensors for FIFO (max size 512 bytes in MPU-9250)
	c = FIFO_GYRO_X_EN|FIFO_GYRO_Y_EN|FIFO_GYRO_Z_EN;
	rc_i2c_write_byte(IMU_BUS, FIFO_EN, c); 
	// 6 bytes per sample. 200hz. wait 0.4 seconds
	rc_usleep(400000);

	// At end of sample accumulation, turn off FIFO sensor read
	rc_i2c_write_byte(IMU_BUS, FIFO_EN, 0x00);   
	// read FIFO sample count and log number of samples
	rc_i2c_read_bytes(IMU_BUS, FIFO_COUNTH, 2, &data[0]); 
	int16_t fifo_count = ((uint16_t)data[0] << 8) | data[1];
	int samples = fifo_count/6;

	#ifdef DEBUG
	printf("calibration samples: %d\n", samples);
	#endif
	
	int i;
	int16_t x,y,z;
	rc_vector_t vx = rc_empty_vector();
	rc_vector_t vy = rc_empty_vector();
	rc_vector_t vz = rc_empty_vector();
	rc_alloc_vector(&vx,samples);
	rc_alloc_vector(&vy,samples);
	rc_alloc_vector(&vz,samples);
	float dev_x, dev_y, dev_z;
	gyro_sum[0] = 0;
	gyro_sum[1] = 0;
	gyro_sum[2] = 0;
	for (i=0; i<samples; i++) {
		// read data for averaging
		if(rc_i2c_read_bytes(IMU_BUS, FIFO_R_W, 6, data)<0){
			fprintf(stderr,"ERROR: failed to read FIFO\n");
			return -1;
		}
		x = (int16_t)(((int16_t)data[0] << 8) | data[1]) ;
		y = (int16_t)(((int16_t)data[2] << 8) | data[3]) ;
		z = (int16_t)(((int16_t)data[4] << 8) | data[5]) ;
		gyro_sum[0]  += (int32_t) x;
		gyro_sum[1]  += (int32_t) y;
		gyro_sum[2]  += (int32_t) z;
		vx.d[i] = (float)x;
		vy.d[i] = (float)y;
		vz.d[i] = (float)z;
	}
	dev_x = rc_std_dev(vx);
	dev_y = rc_std_dev(vy);
	dev_z = rc_std_dev(vz);
	rc_free_vector(&vx);
	rc_free_vector(&vy);
	rc_free_vector(&vz);

	#ifdef DEBUG
	printf("gyro sums: %d %d %d\n", gyro_sum[0], gyro_sum[1], gyro_sum[2]);
	printf("std_deviation: %6.2f %6.2f %6.2f\n", dev_x, dev_y, dev_z);
	#endif

	// try again is standard deviation is too high
	if(dev_x>GYRO_CAL_THRESH||dev_y>GYRO_CAL_THRESH||dev_z>GYRO_CAL_THRESH){
		printf("Gyro data too noisy, put me down on a solid surface!\n");
		printf("trying again\n");
		was_last_steady = 0;
		goto COLLECT_DATA;
	}
	// this skips the first steady reading after a noisy reading
	// to make sure IMU has settled after being picked up.
	if(was_last_steady == 0){
		was_last_steady = 1;
		goto COLLECT_DATA;
	}
	// average out the samples
	offsets[0] = (int16_t) (gyro_sum[0]/(int32_t)samples);
	offsets[1] = (int16_t) (gyro_sum[1]/(int32_t)samples);
	offsets[2] = (int16_t) (gyro_sum[2]/(int32_t)samples);

	// also check for values that are way out of bounds
	if(abs(offsets[0])>GYRO_OFFSET_THRESH || abs(offsets[1])>GYRO_OFFSET_THRESH \
										|| abs(offsets[2])>GYRO_OFFSET_THRESH){
		printf("Gyro data out of bounds, put me down on a solid surface!\n");
		printf("trying again\n");
		goto COLLECT_DATA;
	}
	// done with I2C for now
	rc_i2c_release_bus(IMU_BUS);
	#ifdef DEBUG
	printf("offsets: %d %d %d\n", offsets[0], offsets[1], offsets[2]);
	#endif
	// write to disk
	if(write_gyro_offets_to_disk(offsets)<0){
		fprintf(stderr,"ERROR in rc_calibrate_gyro_routine, failed to write to disk\n");
		return -1;
	}
	return 0;
}

/*******************************************************************************
* unsigned short inv_row_2_scale(signed char row[])
*
* takes a single row on a rotation matrix and returns the associated scalar
* for use by inv_orientation_matrix_to_scalar.
*******************************************************************************/
unsigned short inv_row_2_scale(signed char row[]){
	unsigned short b;

	if (row[0] > 0)
		b = 0;
	else if (row[0] < 0)
		b = 4;
	else if (row[1] > 0)
		b = 1;
	else if (row[1] < 0)
		b = 5;
	else if (row[2] > 0)
		b = 2;
	else if (row[2] < 0)
		b = 6;
	else
		b = 7;      // error
	return b;
}

/*******************************************************************************
* unsigned short inv_orientation_matrix_to_scalar(signed char mtx[])
*
* This take in a rotation matrix and returns the corresponding 16 bit short
* which is sent to the DMP to set the orientation. This function is actually
* not used in normal operation and only served to retrieve the orientation
* scalars once to populate the rc_imu_orientation_t enum during development.
*******************************************************************************/
unsigned short inv_orientation_matrix_to_scalar(signed char mtx[]){
	unsigned short scalar;

	scalar = inv_row_2_scale(mtx);
	scalar |= inv_row_2_scale(mtx + 3) << 3;
	scalar |= inv_row_2_scale(mtx + 6) << 6;
	return scalar;
}

/*******************************************************************************
* void print_orientation_info()
*
* this function purely serves to print out orientation values and rotation
* matrices which form the rc_imu_orientation_t enum. This is not called inside
* this C file and is not exposed to the user.
*******************************************************************************/
void print_orientation_info(){
	printf("\n");
	//char mtx[9];
	unsigned short orient;
	
	// Z-UP (identity matrix)
	signed char zup[] = {1,0,0, 0,1,0, 0,0,1};
	orient = inv_orientation_matrix_to_scalar(zup);
	printf("Z-UP: %d\n", orient);
	
	// Z-down
	signed char zdown[] = {-1,0,0, 0,1,0, 0,0,-1};
	orient = inv_orientation_matrix_to_scalar(zdown);
	printf("Z-down: %d\n", orient);
	
	// X-up
	signed char xup[] = {0,0,-1, 0,1,0, 1,0,0};
	orient = inv_orientation_matrix_to_scalar(xup);
	printf("x-up: %d\n", orient);
	
	// X-down
	signed char xdown[] = {0,0,1, 0,1,0, -1,0,0};
	orient = inv_orientation_matrix_to_scalar(xdown);
	printf("x-down: %d\n", orient);
	
	// Y-up
	signed char yup[] = {1,0,0, 0,0,-1, 0,1,0};
	orient = inv_orientation_matrix_to_scalar(yup);
	printf("y-up: %d\n", orient);
	
	// Y-down
	signed char ydown[] = {1,0,0, 0,0,1, 0,-1,0};
	orient = inv_orientation_matrix_to_scalar(ydown);
	printf("y-down: %d\n", orient);

	// X-forward
	signed char xforward[] = {0,-1,0, 1,0,0, 0,0,1};
	orient = inv_orientation_matrix_to_scalar(xforward);
	printf("x-forward: %d\n", orient);

	// X-back
	signed char xback[] = {0,1,0, -1,0,0, 0,0,1};
	orient = inv_orientation_matrix_to_scalar(xback);
	printf("yx-back: %d\n", orient);
}

/*******************************************************************************
* int rc_was_last_imu_read_successful()
*
* Occasionally bad data is read from the IMU, but the user's imu interrupt 
* function is always called on every interrupt to keep discrete filters
* running at a steady clock. In the event of a bad read, old data is always
* available in the user's rc_imu_data_t struct and the user can call 
* rc_was_last_imu_read_successful() to see if the data was updated or not.
*******************************************************************************/
int rc_was_last_imu_read_successful(){
	return last_read_successful;
}

/*******************************************************************************
* uint64_t rc_nanos_since_last_imu_interrupt()
*
* Immediately after the IMU triggers an interrupt saying new data is ready,
* a timestamp is logged in microseconds. The user's imu_interrupt_function
* will be called after all data has been read in through the I2C bus and 
* the user's rc_imu_data_t struct has been populated. If the user wishes to see
* how long it has been since that interrupt was received they may use this
* function.
*******************************************************************************/
uint64_t rc_nanos_since_last_imu_interrupt(){
	return rc_nanos_since_epoch() - last_interrupt_timestamp_nanos;
}

/*******************************************************************************
* int write_mag_cal_to_disk(float offsets[3], float scale[3])
*
* Reads steady state gyro offsets from the disk and puts them in the IMU's 
* gyro offset register. If no calibration file exists then make a new one.
*******************************************************************************/
int write_mag_cal_to_disk(float offsets[3], float scale[3]){
	FILE *cal;
	char file_path[100];
	int ret;
	
	// construct a new file path string and open for writing
	strcpy(file_path, CONFIG_DIRECTORY);
	strcat(file_path, MAG_CAL_FILE);
	cal = fopen(file_path, "w+");
	// if opening for writing failed, the directory may not exist yet
	if (cal == 0) {
		mkdir(CONFIG_DIRECTORY, 0777);
		cal = fopen(file_path, "w+");
		if (cal == 0){
			fprintf(stderr,"could not open config directory\n");
			fprintf(stderr, CONFIG_DIRECTORY);
			fprintf(stderr,"\n");
			return -1;
		}
	}
	
	// write to the file, close, and exit
	ret = fprintf(cal,"%f\n%f\n%f\n%f\n%f\n%f\n", 	offsets[0],\
													offsets[1],\
													offsets[2],\
													scale[0],\
													scale[1],\
													scale[2]);
	if(ret<0){
		fprintf(stderr,"Failed to write mag calibration to file\n");
		fclose(cal);
		return -1;
	}
	fclose(cal);
	return 0;
}

/*******************************************************************************
* int load_mag_calibration()
*
* Loads steady state magnetometer offsets and scale from the disk into global
* variables for correction later by read_magnetometer and FIFO read functions
*******************************************************************************/
int load_mag_calibration(){
	FILE *cal;
	char file_path[100];
	float x,y,z,sx,sy,sz;
	
	// construct a new file path string and open for reading
	strcpy (file_path, CONFIG_DIRECTORY);
	strcat (file_path, MAG_CAL_FILE);
	cal = fopen(file_path, "r");
	
	if (cal == 0) {
		// calibration file doesn't exist yet
		fprintf(stderr,"WARNING: no magnetometer calibration data found\n");
		fprintf(stderr,"Please run rc_calibrate_mag\n\n");
		mag_offsets[0]=0.0;
		mag_offsets[1]=0.0;
		mag_offsets[2]=0.0;
		mag_scales[0]=1.0;
		mag_scales[1]=1.0;
		mag_scales[2]=1.0;
		return -1;
	}
	// read in data
	fscanf(cal,"%f\n%f\n%f\n%f\n%f\n%f\n", &x,&y,&z,&sx,&sy,&sz);

	#ifdef DEBUG
	printf("magcal: %f %f %f %f %f %f\n", x,y,z,sx,sy,sz);
	#endif

	// write to global variables fo use by rc_read_mag_data
	mag_offsets[0]=x;
	mag_offsets[1]=y;
	mag_offsets[2]=z;
	mag_scales[0]=sx;
	mag_scales[1]=sy;
	mag_scales[2]=sz;

	fclose(cal);
	return 0;
}

/*******************************************************************************
* int rc_calibrate_mag_routine()
*
* Initializes the IMU and samples the magnetometer until sufficient samples
* have been collected from each octant. From there, fit an ellipse to the data 
* and save the correct offsets and scales to the disk which will later be
* applied to correct the uncalibrated magnetometer data to map calibrated
* field vectors to a sphere.
*******************************************************************************/
int rc_calibrate_mag_routine(){
	const int samples = 200;
	const int sample_rate_hz = 15;
	int i;
	uint8_t c;
	float new_scale[3];
	rc_matrix_t A = rc_empty_matrix();
	rc_vector_t center = rc_empty_vector();
	rc_vector_t lengths = rc_empty_vector();
	rc_imu_data_t imu_data; // to collect magnetometer data
	config = rc_default_imu_config();
	config.enable_magnetometer = 1;
	
	// make sure the bus is not currently in use by another thread
	// do not proceed to prevent interfering with that process
	if(rc_i2c_get_in_use_state(IMU_BUS)){
		fprintf(stderr,"i2c bus claimed by another process\n");
		fprintf(stderr,"aborting gyro calibration()\n");
		return -1;
	}
	
	// if it is not claimed, start the i2c bus
	if(rc_i2c_init(IMU_BUS, IMU_ADDR)){
		fprintf(stderr,"rc_initialize_imu_dmp failed at rc_i2c_init\n");
		return -1;
	}
	
	// claiming the bus does no guarantee other code will not interfere 
	// with this process, but best to claim it so other code can check
	// like we did above
	rc_i2c_claim_bus(IMU_BUS);
	
	// reset device, reset all registers
	if(reset_mpu9250()<0){
		fprintf(stderr,"ERROR: failed to reset MPU9250\n");
		return -1;
	}
	//check the who am i register to make sure the chip is alive
	if(rc_i2c_read_byte(IMU_BUS, WHO_AM_I_MPU9250, &c)<0){
		fprintf(stderr,"Reading WHO_AM_I_MPU9250 register failed\n");
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
	if(c!=0x71){
		fprintf(stderr,"mpu9250 WHO AM I register should return 0x71\n");
		fprintf(stderr,"WHO AM I returned: 0x%x\n", c);
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
	if(initialize_magnetometer()){
		fprintf(stderr,"ERROR: failed to initialize_magnetometer\n");
		rc_i2c_release_bus(IMU_BUS);
		return -1;
	}
	
	// set local calibration to initial values and prepare variables
	mag_offsets[0] = 0.0;
	mag_offsets[1] = 0.0;
	mag_offsets[2] = 0.0;
	mag_scales[0]  = 1.0;
	mag_scales[1]  = 1.0;
	mag_scales[2]  = 1.0;
	rc_alloc_matrix(&A,samples,3);
	i = 0;
		
	// sample data
	while(i<samples && rc_get_state()!=EXITING){
		if(rc_read_mag_data(&imu_data)<0){
			fprintf(stderr,"ERROR: failed to read magnetometer\n");
			break;
		}
		// make sure the data is non-zero
		if(imu_data.mag[0]==0 && imu_data.mag[1]==0 && imu_data.mag[2]==0){
			fprintf(stderr,"ERROR: retreived all zeros from magnetometer\n");
			break;	
		}
		// save data to matrix for ellipse fitting
		A.d[i][0] = imu_data.mag[0];
		A.d[i][1] = imu_data.mag[1];
		A.d[i][2] = imu_data.mag[2];
		i++;
		
		// print "keep going" every 4 seconds
		if(i%(sample_rate_hz*4) == sample_rate_hz*2){
			printf("keep spinning\n");
		}
		// print "you're doing great" every 4 seconds
		if(i%(sample_rate_hz*4) == 0){
			printf("you're doing great\n");
		}
		
		rc_usleep(1000000/sample_rate_hz);
	}
	// done with I2C for now
	rc_power_off_imu();
	rc_i2c_release_bus(IMU_BUS);
	
	printf("\n\nOkay Stop!\n");
	printf("Calculating calibration constants.....\n");
	fflush(stdout);
	
	// if data collection loop exited without getting enough data, warn the
	// user and return -1, otherwise keep going normally
	if(i<samples){
		printf("exiting rc_calibrate_mag_routine without saving new data\n");
		return -1;
	}
	// make empty vectors for ellipsoid fitting to populate
	if(rc_fit_ellipsoid(A,&center,&lengths)<0){
		fprintf(stderr,"failed to fit ellipsoid to magnetometer data\n");
		rc_free_matrix(&A);
		return -1;
	}
	// empty memory, we are done with A
	rc_free_matrix(&A); 
	// do some sanity checks to make sure data is reasonable
	if(fabs(center.d[0])>200 || fabs(center.d[1])>200 || \
											fabs(center.d[2])>200){
		fprintf(stderr,"ERROR: center of fitted ellipsoid out of bounds\n");
		rc_free_vector(&center);
		rc_free_vector(&lengths);
		return -1;
	}
	if( lengths.d[0]>200 || lengths.d[0]<5 || \
		lengths.d[1]>200 || lengths.d[1]<5 || \
		lengths.d[2]>200 || lengths.d[2]<5){
		fprintf(stderr,"ERROR: length of fitted ellipsoid out of bounds\n");
		//rc_free_vector(&center);
		//rc_free_vector(&lengths);
		//return -1;
	}
	// all seems well, calculate scaling factors to map ellipse lengths to
	// a sphere of radius 70uT, this scale will later be multiplied by the
	// factory corrected data
	new_scale[0] = 70.0f/lengths.d[0];
	new_scale[1] = 70.0f/lengths.d[1];
	new_scale[2] = 70.0f/lengths.d[2];
	// print results
	printf("\n");
	printf("Offsets X: %7.3f Y: %7.3f Z: %7.3f\n", 	center.d[0],\
													center.d[1],\
													center.d[2]);
	printf("Scales  X: %7.3f Y: %7.3f Z: %7.3f\n", 	new_scale[0],\
													new_scale[1],\
													new_scale[2]);
	// write to disk
	if(write_mag_cal_to_disk(center.d,new_scale)<0){
		rc_free_vector(&center);
		rc_free_vector(&lengths);
		return -1;
	}
	rc_free_vector(&center);
	rc_free_vector(&lengths);
	return 0;
}

/*******************************************************************************
* int rc_is_gyro_calibrated()
*
* return 1 is a gyro calibration file exists, otherwise 0
*******************************************************************************/
int rc_is_gyro_calibrated(){
	char file_path[100];
	strcpy (file_path, CONFIG_DIRECTORY);
	strcat (file_path, GYRO_CAL_FILE);
	if(!access(file_path, F_OK)) return 1;
	else return 0;
}

/*******************************************************************************
* int rc_is_mag_calibrated()
*
* return 1 is a magnetometer calibration file exists, otherwise 0
*******************************************************************************/
int rc_is_mag_calibrated(){
	char file_path[100];
	strcpy (file_path, CONFIG_DIRECTORY);
	strcat (file_path, MAG_CAL_FILE);
	if(!access(file_path, F_OK)) return 1;
	else return 0;
}




// Phew, that was a lot of code....
