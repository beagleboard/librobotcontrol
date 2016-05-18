/*******************************************************************************
* mpu9250.c
*
* This is a collection of high-level functions to control the
* MPU9250 from a BeagleBone Black as configured on the Robotics Cape.
* Credit to Kris Winer most of the framework and register definitions.
*******************************************************************************/

#include <robotics_cape.h>
#include <useful_includes.h>
#include "mpu9250_defs.h"
#include "dmp_firmware.h"
#include "dmpKey.h"

#define DEBUG

#define INTERRUPT_PIN 117  //gpio3.21 P9.25
#define min(a, b) 	((a < b) ? a : b)

/*******************************************************************************
*	Local variables
*******************************************************************************/
imu_config_t config;
int bypass_en;  
int dmp_en;
int fifo_packet_length;
pthread_t imu_interrupt_thread;
int (*imu_interrupt_func)();
int interrupt_running;
imu_data_t* imu_data_ptr;

/*******************************************************************************
*	config functions for internal use only
*******************************************************************************/
int reset_mpu9250();
int set_gyro_fsr(gyro_fsr_t fsr, imu_data_t* data);
int set_accel_fsr(accel_fsr_t, imu_data_t* data);
int set_gyro_dlpf(gyro_dlpf_t);
int set_accel_dlpf(accel_dlpf_t);
int initialize_magnetometer(imu_data_t* data);
int power_down_magnetometer();
int mpu_set_bypass(unsigned char bypass_on);
int mpu_write_mem(unsigned short mem_addr, unsigned short length,\
												unsigned char *data);
int mpu_read_mem(unsigned short mem_addr, unsigned short length,\
												unsigned char *data);
int dmp_load_motion_driver_firmware();
int dmp_set_orientation(unsigned short orient);
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
unsigned short inv_row_2_scale(const signed char *row);
int dmp_enable_gyro_cal(unsigned char enable);
int dmp_enable_lp_quat(unsigned char enable);
int dmp_enable_6x_lp_quat(unsigned char enable);
int mpu_reset_fifo(void);
int dmp_set_fifo_rate(unsigned short rate);
int dmp_enable_feature(unsigned short mask);
int mpu_set_dmp_state(unsigned char enable);
int set_int_enable(unsigned char enable);
// flush the FIFO buffer to retreive sensor samples
// and dmp orientation estimate. 
int read_dmp_fifo(imu_data_t* data);

void* imu_interrupt_handler(void* ptr);
int (*imu_interrupt_func)(); // pointer to user-defined function


/*******************************************************************************
* imu_config_t get_default_imu_config()
*
* returns reasonable default configuration values
*******************************************************************************/
imu_config_t get_default_imu_config(){
	imu_config_t conf;
	
	conf.accel_fsr = A_FSR_4G;
	conf.gyro_fsr = G_FSR_1000DPS;
	conf.gyro_dlpf = GYRO_DLPF_250;
	conf.accel_dlpf = ACCEL_DLPF_184;
	conf.enable_magnetometer = 1;
	conf.dmp_sample_rate = 200;
	
	// fill in orientation array
	char orient[9] = ORIENTATION_Z_UP;
	int i;
	for(i=0; i<9; i++){
		conf.orientation[i] = orient[i];
	}
	
	conf.dmp_interrupt_priority = sched_get_priority_max(SCHED_FIFO) -5;
	
	return conf;
}

/*******************************************************************************
* int initialize_imu(imu_config_t conf)
*
* Set up the imu for one-shot sampling of sensor data by user
*******************************************************************************/
int initialize_imu(imu_data_t *data, imu_config_t conf){  
	int ret;
	uint8_t c;
	
	// make sure the bus is not currently in use by another thread
	// do not proceed to prevent interfering with that process
	if(i2c_get_in_use_state(IMU_BUS)){
		printf("i2c bus claimed by another process\n");
		printf("aborting initialize_imu()\n");
		return -1;
	}
	
	// if it is not claimed, start the i2c bus
	if(i2c_init(IMU_BUS, IMU_ADDR)){
		printf("failed to initialize i2c bus\n");
		return -1;
	}
	// claiming the bus does no guarantee other code will not interfere 
	// with this process, but best to claim it so other code can check
	// like we did above
	i2c_claim_bus(IMU_BUS);
	
	// update local copy of config struct with new values
	config=conf;
	
	// restart the device so we start with clean registers
	if(reset_mpu9250()){
		return -1;
	}
	
	//check the who am i register to make sure the chip is alive
	ret = i2c_read_byte(IMU_BUS, WHO_AM_I_MPU9250, &c);
	if(ret){
		printf("i2c_read_byte failed\n");
		return -1;
	}
	if(c!=0x71){
		printf("mpu9250 WHO AM I register should return 0x71\n");
		printf("WHO AM I returned: 0x%x\n", c);
		return -1;
	}
 
	// Set sample rate = 1000/(1 + SMPLRT_DIV)
	// here we use a divider of 0
	if(i2c_write_byte(IMU_BUS, SMPLRT_DIV, 0x00)){
		printf("I2C bus write error\n");
		return -1;
	}
	
	// set full scale ranges and filter constants
	if(set_gyro_fsr(conf.gyro_fsr, data)){
		printf("failed to set gyro fsr\n");
		return -1;
	}
	if(set_accel_fsr(conf.accel_fsr, data)){
		printf("failed to set accel fsr\n");
		return -1;
	}
	if(set_gyro_dlpf(conf.gyro_dlpf)){
		printf("failed to set gyro dlpf\n");
		return -1;
	}
	if(set_accel_dlpf(conf.accel_dlpf)){
		printf("failed to set accel_dlpf\n");
		return -1;
	}

	// Enable i2c bypass to allow talking to magnetometer
	if(mpu_set_bypass(1)){
		printf("failed to set mpu9250 into bypass i2c mode\n");
		return -1;
	}
	
	// initialize the magnetometer too if requested in config
	if(conf.enable_magnetometer){
		if(initialize_magnetometer(data)){
			printf("failed to initialize magnetometer\n");
			return -1;
		}
	}
	else power_down_magnetometer();
	
	// all done!!
	i2c_release_bus(IMU_BUS);
	return 0;
}

/*******************************************************************************
* int read_accel_data(imu_data_t* data)
* 
* Always reads in latest accelerometer values. The sensor 
* self-samples at 1khz and this retrieves the latest data.
*******************************************************************************/
int read_accel_data(imu_data_t *data){
	// new register data stored here
	uint8_t raw[6];  
	
	// set the device address
	i2c_set_device_address(IMU_BUS, IMU_ADDR);
	
	 // Read the six raw data registers into data array
	if(i2c_read_bytes(IMU_BUS, ACCEL_XOUT_H, 6, &raw[0])){
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
* int read_gyro_data(imu_data_t* data)
*
* Always reads in latest gyroscope values. The sensor self-samples
* at 1khz and this retrieves the latest data.
*******************************************************************************/
int read_gyro_data(imu_data_t *data){
	// new register data stored here
	uint8_t raw[6];
	
	// set the device address
	i2c_set_device_address(IMU_BUS, IMU_ADDR);
	
	 // Read the six raw data registers into data array
	if(i2c_read_bytes(IMU_BUS, GYRO_XOUT_H, 6, &raw[0])){
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
* int read_mag_data(imu_data_t* data)
*
* Checks if there is new magnetometer data and reads it in if true.
* Magnetometer only updates at 100hz, if there is no new data then
* the values in imu_data_t struct are left alone.
*******************************************************************************/
int read_mag_data(imu_data_t* data){
	uint8_t st1;
	uint8_t raw[7];
	int16_t adc[3];
	int ret;
	
	// magnetometer is actually a separate device with its
	// own address inside the mpu9250
	// MPU9250 was put into passthrough mode 
	i2c_set_device_address(IMU_BUS, AK8963_ADDR);
	
	// read the data ready bit to see if there is new data
	i2c_read_byte(IMU_BUS, AK8963_ST1, &st1);
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
	ret=i2c_read_bytes(IMU_BUS,AK8963_XOUT_L,7,&raw[0]); 
	
	if(ret){
		printf("read_mag_data failed\n");
		return -1;
	}
	
	// check if the readings saturated such as because
	// of a local field source, discard data if so
	if(raw[6]&DATA_OVERFLOW){
		#ifdef DEBUG
		printf("magnetometer saturated\n");
		#endif
		return 0;
	}
	
	// Turn the MSB and LSB into a signed 16-bit value
	// Data stored as little Endian
	adc[0] = (int16_t)(((int16_t)raw[1]<<8) | raw[0]);  
	adc[1] = (int16_t)(((int16_t)raw[3]<<8) | raw[2]);  
	adc[2] = (int16_t)(((int16_t)raw[5]<<8) | raw[4]); 
	
	#ifdef DEBUG
	printf("raw mag:%d %d %d\n", adc[0], adc[1], adc[2]);
	#endif
	// multiply by the sensitivity adjustment and convert to
	// units of uT micro Teslas
	data->mag[0] = adc[0] * data->mag_adjust[0] * MAG_RAW_TO_uT;
	data->mag[1] = adc[1] * data->mag_adjust[1] * MAG_RAW_TO_uT;
	data->mag[2] = adc[2] * data->mag_adjust[2] * MAG_RAW_TO_uT;
	
	return 0;
}

/*******************************************************************************
* int read_imu_temp(imu_data_t* data)
*
* reads the latest temperature of the imu. 
*******************************************************************************/
int read_imu_temp(imu_data_t* data){
	uint8_t raw[2];
	int16_t adc;
	int ret;
	
	// set device address
	i2c_set_device_address(IMU_BUS, IMU_ADDR);
	
	// Read the two raw data registers
	ret = i2c_read_bytes(IMU_BUS, TEMP_OUT_H, 2, &raw[0]); 
	
	if(ret){
		printf("failed to read IMU temperature registers\n");
		return -1;
	}
	
	// Turn the MSB and LSB into a 16-bit value
	adc = (int16_t)(((int16_t)raw[0]) << 8 | raw[1]);  
	
	// convert to real units
	data->temp = ((float)(adc)/TEMP_SENSITIVITY) + 21.0;
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
	// set the device address
	i2c_set_device_address(IMU_BUS, IMU_ADDR);
	
	// write to reset bit
	if(i2c_write_byte(IMU_BUS, PWR_MGMT_1, H_RESET)){
		printf("reset_mpu9250 failed\n\n");
		return -1;
	}
	usleep(100000);
	return 0;
}

/*******************************************************************************
* int set_gyro_fsr(gyro_fsr_t fsr, imu_data_t* data)
* 
* set gyro full scale range and update conversion ratio
*******************************************************************************/
int set_gyro_fsr(gyro_fsr_t fsr, imu_data_t* data){
	uint8_t c;
	switch(fsr){
	case G_FSR_250DPS:
		c = GYRO_FSR_CFG_250 | FCHOICE_B_DLPF_EN;
		data->gyro_to_degs = (float)250/(float)32768;
		break;
	case G_FSR_500DPS:
		c = GYRO_FSR_CFG_500 | FCHOICE_B_DLPF_EN;
		data->gyro_to_degs = (float)500/(float)32768;
		break;
	case G_FSR_1000DPS:
		c = GYRO_FSR_CFG_1000 | FCHOICE_B_DLPF_EN;
		data->gyro_to_degs = (float)1000/(float)32768;
		break;
	case G_FSR_2000DPS:
		c = GYRO_FSR_CFG_2000 | FCHOICE_B_DLPF_EN;
		data->gyro_to_degs = (float)2000/(float)32768;
		break;
	default:
		printf("invalid gyro fsr\n");
		return -1;
	}
	return i2c_write_byte(IMU_BUS, GYRO_CONFIG, c);
}

/*******************************************************************************
* int set_accel_fsr(accel_fsr_t fsr, imu_data_t* data)
* 
* set accelerometer full scale range and update conversion ratio
*******************************************************************************/
int set_accel_fsr(accel_fsr_t fsr, imu_data_t* data){
	uint8_t c;
	switch(fsr){
	case A_FSR_2G:
		c = ACCEL_FSR_CFG_2G;
		data->accel_to_ms2 = 9.807*(float)2/(float)32768;
		break;
	case A_FSR_4G:
		c = ACCEL_FSR_CFG_4G;
		data->accel_to_ms2 = 9.807*(float)4/(float)32768;
		break;
	case A_FSR_8G:
		c = ACCEL_FSR_CFG_8G;
		data->accel_to_ms2 = 9.807*(float)8/(float)32768;
		break;
	case A_FSR_16G:
		c = ACCEL_FSR_CFG_16G;
		data->accel_to_ms2 = 9.807*(float)16/(float)32768;
		break;
	default:
		printf("invalid accel fsr\n");
		return -1;
		
	}
	return i2c_write_byte(IMU_BUS, ACCEL_CONFIG, c);
}

/*******************************************************************************
* int set_gyro_dlpf(gyro_dlpf_t dlpf)
*
* Set GYRO low pass filter constants. This is the same register as
* the fifo overflow mode so we set it to keep the newest data too.
*******************************************************************************/
int set_gyro_dlpf(gyro_dlpf_t dlpf){ 
	uint8_t c = FIFO_MODE_REPLACE_OLD | (uint8_t)dlpf;
	return i2c_write_byte(IMU_BUS, CONFIG, c); 
}

/*******************************************************************************
* int set_accel_dlpf(accel_dlpf_t dlpf)
*
* Set accel low pass filter constants. This is the same register as
* the sample rate. We set it at 1khz as 4khz is unnecessary.
*******************************************************************************/
int set_accel_dlpf(accel_dlpf_t dlpf){
	uint8_t c = ACCEL_FCHOICE_1KHZ | (uint8_t)dlpf;
	return i2c_write_byte(IMU_BUS, ACCEL_CONFIG_2, c);
}


/*******************************************************************************
* int initialize_magnetometer(imu_data_t* data)
*
* configure the magnetometer and load sensitivity adjustment
* values into the data struct for use by future reads
*******************************************************************************/
int initialize_magnetometer(imu_data_t* data){
	uint8_t raw[3];  // calibration data stored here
	int ret;
	
	// magnetometer is actually a separate device with its
	// own address inside the mpu9250
	i2c_set_device_address(IMU_BUS, AK8963_ADDR);
	
	// Power down magnetometer  
	i2c_write_byte(IMU_BUS, AK8963_CNTL, MAG_POWER_DN); 
	usleep(1000);
	
	// Enter Fuse ROM access mode
	i2c_write_byte(IMU_BUS, AK8963_CNTL, MAG_FUSE_ROM); 
	usleep(1000);
	
	// Read the xyz sensitivity adjustment values
	ret = i2c_read_bytes(IMU_BUS, AK8963_ASAX, 3, &raw[0]);

	// check if the adjustment value read worked
	if(ret){
		printf("failed to read magnetometer adjustment regs\n");
		return -1;
	}

	// Return sensitivity adjustment values
	data->mag_adjust[0]=(float)(raw[0]-128)/256.0f + 1.0f;   
	data->mag_adjust[1]=(float)(raw[1]-128)/256.0f + 1.0f;  
	data->mag_adjust[2]=(float)(raw[2]-128)/256.0f + 1.0f; 
	
	// Power down magnetometer again
	i2c_write_byte(IMU_BUS, AK8963_CNTL, MAG_POWER_DN); 
	usleep(100);
	
	// Configure the magnetometer for 16 bit resolution 
	// and continuous sampling mode 2 (100hz)
	uint8_t c = MSCALE_16|MAG_CONT_MES_2;
	i2c_write_byte(IMU_BUS, AK8963_CNTL, c);
	usleep(100);
	
	return 0;
}

/*******************************************************************************
* int power_down_magnetometer()
*
* Make sure the magnetometer is off.
*******************************************************************************/
int power_down_magnetometer(){
	int ret;
	
	// magnetometer is actually a separate device with its
	// own address inside the mpu9250
	i2c_set_device_address(IMU_BUS, AK8963_ADDR);
	
	// Power down magnetometer  
	ret = i2c_write_byte(IMU_BUS, AK8963_CNTL, MAG_POWER_DN); 
	
	if(ret){
		printf("failed to write to magnetometer\n");
		return -1;
	}
	return 0;
}












// /******************************************************************
// *	Set up the IMU for DMP accelerated filtering and interrupts
// *******************************************************************/
// int initialize_imu_dmp(imu_data_t *data, imu_config_t conf){
	
	// // range check
	// if(conf.dmp_sample_rate>DMP_MAX_RATE || conf.dmp_sample_rate<DMP_MIN_RATE){
		// printf("ERROR:dmp_sample_rate must be between DMP_MAX_RATE & DMP_MIN_RATE\n");
		// return -1;
	// }
	
	// //set up gpio interrupt pin connected to imu
	// if(gpio_export(INTERRUPT_PIN)){
		// printf("can't export gpio %d \n", INTERRUPT_PIN);
		// return (-1);
	// }
	// gpio_set_dir(INTERRUPT_PIN, INPUT_PIN);
	// gpio_set_edge(INTERRUPT_PIN, "rising");
	
	// // make sure the bus is not currently in use by another thread
	// // do not proceed to prevent interfering with that process
	// if(i2c_get_in_use_state(IMU_BUS)){
		// printf("i2c bus claimed by another process\n");
		// printf("aborting initialize_imu()\n");
		// return -1;
	// }
	
	// // if it is not claimed, start the i2c bus
	// if(i2c_init(IMU_BUS, IMU_ADDR)){
		// printf("initialize_imu_dmp failed at i2c_init\n");
		// return -1;
	// }
	
	// // claiming the bus does no guarantee other code will not interfere 
	// // with this process, but best to claim it so other code can check
	// // like we did above
	// i2c_claim_bus(IMU_BUS);
	
	// // update local copy of config struct with new values
	// config = conf;
	
	// // restart the device so we start with clean registers
	// if(reset_mpu9250()){
		// printf("failed to reset_mpu9250()\n");
		// return -1;
	// }
	
	// // log locally that the dmp will be running
	// dmp_en = 1;
	
	// // set full scale ranges and filter constants
	// set_gyro_fsr(conf.gyro_fsr, data);
	// set_accel_fsr(conf.accel_fsr, data);
	// set_gyro_dlpf(conf.gyro_dlpf);
	// set_accel_dlpf(conf.accel_dlpf);
	
	// // initialize the magnetometer too if requested in config
	// if(mpu_set_bypass(1)) return -1;
	// if(conf.enable_magnetometer){
		// initialize_magnetometer(data);
	// }
	// else{
		// power_down_magnetometer();
	// }
	// mpu_set_bypass(0); // turn off bypass, DMP will manage the mag from now on
	
	// dmp_load_motion_driver_firmware();

	// dmp_set_orientation(inv_orientation_matrix_to_scalar(&conf.orientation[0]));
  	// // dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL 
						// // | DMP_FEATURE_SEND_CAL_GYRO);
	// dmp_enable_feature(DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_RAW_GYRO);

	// dmp_set_fifo_rate((unsigned short)config.dmp_sample_rate);
	
	
	// #ifdef DEBUG
	 // printf("enabling dmp\n");
	// #endif
	// if (mpu_set_dmp_state(1)) {
		// printf("\nmpu_set_dmp_state(1) failed\n");
		// return -1;
	// }
	// // // if(loadGyroCalibration()){
		// // // printf("\nGyro Calibration File Doesn't Exist Yet\n");
		// // // printf("Use calibrate_gyro example to create one\n");
		// // // printf("Using 0 offset for now\n");
	// // // };
	
	// // start the interrupt handler thread
	// set_imu_interrupt_func(&null_func, NULL);
	// struct sched_param params;
	// pthread_create(&imu_interrupt_thread, NULL, imu_interrupt_handler, (void*) NULL);
	// params.sched_priority = config.dmp_interrupt_priority;
	// pthread_setschedparam(imu_interrupt_thread, SCHED_FIFO, &params);
	
	// return 0;
// }



/**
 *  @brief      Write to the DMP memory.
 *  This function prevents I2C writes past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to write.
 *  @param[in]  data        Bytes to write to memory.
 *  @return     0 if successful.
 */
int mpu_write_mem(unsigned short mem_addr, unsigned short length,\
												unsigned char *data){
    unsigned char tmp[2];

    if (!data)
        return -1;
	
    tmp[0] = (unsigned char)(mem_addr >> 8);
    tmp[1] = (unsigned char)(mem_addr & 0xFF);

    /* Check bank boundaries. */
    if (tmp[1] + length > MPU6500_BANK_SIZE){
		printf("mpu_write_mem exceeds bank size\n");
        return -1;
	}
    if (i2c_write_bytes(IMU_BUS,MPU6500_BANK_SEL, 2, tmp))
        return -1;
    if (i2c_write_bytes(IMU_BUS,MPU6500_MEM_R_W, length, data))
        return -1;
    return 0;
}

/**
 *  @brief      Read from the DMP memory.
 *  This function prevents I2C reads past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Bytes read from memory.
 *  @return     0 if successful.
 */
int mpu_read_mem(unsigned short mem_addr, unsigned short length,\
												unsigned char *data){
    unsigned char tmp[2];

    if (!data)
        return -1;

    tmp[0] = (unsigned char)(mem_addr >> 8);
    tmp[1] = (unsigned char)(mem_addr & 0xFF);

    /* Check bank boundaries. */
    if (tmp[1] + length > MPU6500_BANK_SIZE){
		printf("mpu_read_mem exceeds bank size\n");
        return -1;
	}
    if (i2c_write_bytes(IMU_BUS,MPU6500_BANK_SEL, 2, tmp))
        return -1;
    if (i2c_read_bytes(IMU_BUS,MPU6500_MEM_R_W, length, data))
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
    /* Must divide evenly into st.hw->bank_size to avoid bank crossings. */

    unsigned char cur[DMP_LOAD_CHUNK], tmp[2];

	// make sure the address is set correctly
	i2c_set_device_address(IMU_BUS, IMU_ADDR);
	
	// loop through 16 bytes at a time and check each write
	// for corruption
    for (ii=0; ii<DMP_CODE_SIZE; ii+=this_write) {
        this_write = min(DMP_LOAD_CHUNK, DMP_CODE_SIZE - ii);
        if (mpu_write_mem(ii, this_write, (uint8_t*)&dmp_firmware[ii])){
			printf("dmp firmware write failed\n");
            return -1;
		}
        if (mpu_read_mem(ii, this_write, cur)){
			printf("dmp firmware read failed\n");
            return -1;
		}
        if (memcmp(dmp_firmware+ii, cur, this_write)){
			printf("dmp firmware write corrupted\n");
            return -2;
		}
    }

    /* Set program start address. */
    tmp[0] = dmp_start_addr >> 8;
    tmp[1] = dmp_start_addr & 0xFF;
    if (i2c_write_bytes(IMU_BUS, MPU6500_PRGM_START_H, 2, tmp)){
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

    gyro_regs[0] = gyro_axes[orient & 3];
    gyro_regs[1] = gyro_axes[(orient >> 3) & 3];
    gyro_regs[2] = gyro_axes[(orient >> 6) & 3];
    accel_regs[0] = accel_axes[orient & 3];
    accel_regs[1] = accel_axes[(orient >> 3) & 3];
    accel_regs[2] = accel_axes[(orient >> 6) & 3];

    /* Chip-to-body, axes only. */
    if (mpu_write_mem(FCFG_1, 3, gyro_regs))
        return -1;
    if (mpu_write_mem(FCFG_2, 3, accel_regs))
        return -1;

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

    /* Chip-to-body, sign only. */
    if (mpu_write_mem(FCFG_3, 3, gyro_regs))
        return -1;
    if (mpu_write_mem(FCFG_7, 3, accel_regs))
        return -1;
    //dmp.orient = orient;
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

    if (rate > DMP_MAX_RATE)
        return -1;
    div = DMP_MAX_RATE / rate - 1;
    tmp[0] = (unsigned char)((div >> 8) & 0xFF);
    tmp[1] = (unsigned char)(div & 0xFF);
    if (mpu_write_mem(D_0_22, 2, tmp))
        return -1;
    if (mpu_write_mem(CFG_6, 12, (unsigned char*)regs_end))
        return -1;

    return 0;
}

/*******************************************************************************
* int mpu_set_bypass(unsigned char bypass_on)
* 
* configures the USER_CTRL and INT_PIN_CFG registers
* USER_CTRL - based on global variable dsp_en
* INT_PIN_CFG based on requested bypass state
*******************************************************************************/
int mpu_set_bypass(uint8_t bypass_on){
    uint8_t tmp = 0;

    // set up USER_CTRL first
	if(dmp_en)
		tmp |= FIFO_EN_BIT; // enable fifo for dsp mode
	if(!bypass_on)
		tmp |= I2C_MST_EN; // i2c master mode when not in bypass
	if (i2c_write_byte(IMU_BUS, USER_CTRL, tmp))
            return -1;
    usleep(3000);
	
	// INT_PIN_CFG settings
	tmp = ACTL_ACTIVE_HIGH;
	tmp |= INT_PUSH_PULL;
	tmp |= LATCH_INT_EN;
	tmp |= INT_ANYRD_CLEAR;
	if(bypass_on)
		tmp |= BYPASS_EN;
	if (i2c_write_byte(IMU_BUS, INT_PIN_CFG, tmp))
            return -1;
		
	if(bypass_on)
		bypass_en = 1;
	else
		bypass_en = 0;
	
	return 0;
}


/*******************************************************************************
* These next two functions convert the orientation matrix (see
* gyro_orientation) to a scalar representation for use by the DMP.
* NOTE: These functions are borrowed from InvenSense's MPL.
*******************************************************************************/
unsigned short inv_row_2_scale(const signed char *row){
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

unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx){
    unsigned short scalar;
    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;
    return scalar;
}


/*******************************************************************************
 *  @brief      Enable DMP features.
 *  @param[in]  mask    Mask of features to enable.
 *  @return     0 if successful.
*******************************************************************************/
int dmp_enable_feature(unsigned short mask){
    unsigned char tmp[10];

    /* Set integration scale factor. */
    tmp[0] = (unsigned char)((GYRO_SF >> 24) & 0xFF);
    tmp[1] = (unsigned char)((GYRO_SF >> 16) & 0xFF);
    tmp[2] = (unsigned char)((GYRO_SF >> 8) & 0xFF);
    tmp[3] = (unsigned char)(GYRO_SF & 0xFF);
    mpu_write_mem(D_0_104, 4, tmp);

    /* Send sensor data to the FIFO. */
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
    mpu_write_mem(CFG_15,10,tmp);

    /* Send gesture data to the FIFO. */
    if (mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT))
        tmp[0] = DINA20;
    else
        tmp[0] = 0xD8;
    mpu_write_mem(CFG_27,1,tmp);

    if (mask & DMP_FEATURE_GYRO_CAL)
        dmp_enable_gyro_cal(1);
    else
        dmp_enable_gyro_cal(0);

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

    // if (mask & DMP_FEATURE_TAP) {
        // /* Enable tap. */
        // tmp[0] = 0xF8;
        // mpu_write_mem(CFG_20, 1, tmp);
        // dmp_set_tap_thresh(TAP_XYZ, 250);
        // dmp_set_tap_axes(TAP_XYZ);
        // dmp_set_tap_count(1);
        // dmp_set_tap_time(100);
        // dmp_set_tap_time_multi(500);

        // dmp_set_shake_reject_thresh(GYRO_SF, 200);
        // dmp_set_shake_reject_time(40);
        // dmp_set_shake_reject_timeout(10);
    // } else {
        // tmp[0] = 0xD8;
        // mpu_write_mem(CFG_20, 1, tmp);
    // }
	// disable tap feature
	tmp[0] = 0xD8;
	mpu_write_mem(CFG_20, 1, tmp);

	
    // if (mask & DMP_FEATURE_ANDROID_ORIENT) {
        // tmp[0] = 0xD9;
    // } else
        // tmp[0] = 0xD8;
	
	// disable orientation feature
	tmp[0] = 0xD8;
    mpu_write_mem(CFG_ANDROID_ORIENT_INT, 1, tmp);

    if (mask & DMP_FEATURE_LP_QUAT)
        dmp_enable_lp_quat(1);
    else
        dmp_enable_lp_quat(0);

    if (mask & DMP_FEATURE_6X_LP_QUAT)
        dmp_enable_6x_lp_quat(1);
    else
        dmp_enable_6x_lp_quat(0);

    // /* Pedometer is always enabled. */
    // dmp.feature_mask = mask | DMP_FEATURE_PEDOMETER;
    mpu_reset_fifo();

    fifo_packet_length = 0;
    if (mask & DMP_FEATURE_SEND_RAW_ACCEL)
        fifo_packet_length += 6;
    if (mask & DMP_FEATURE_SEND_ANY_GYRO)
        fifo_packet_length += 6;
    if (mask & (DMP_FEATURE_LP_QUAT | DMP_FEATURE_6X_LP_QUAT))
        fifo_packet_length += 16;
    // if (mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT))
        // dmp.packet_length += 4;

    return 0;
}

/*******************************************************************************
 *  @brief      Calibrate the gyro data in the DMP.
 *  After eight seconds of no motion, the DMP will compute gyro biases and
 *  subtract them from the quaternion output. If @e dmp_enable_feature is
 *  called with @e DMP_FEATURE_SEND_CAL_GYRO, the biases will also be
 *  subtracted from the gyro output.
 *  @param[in]  enable  1 to enable gyro calibration.
 *  @return     0 if successful.
*******************************************************************************/
int dmp_enable_gyro_cal(unsigned char enable){
    if (enable) {
        unsigned char regs[9] = {0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d};
        return mpu_write_mem(CFG_MOTION_BIAS, 9, regs);
    } else {
        unsigned char regs[9] = {0xb8, 0xaa, 0xaa, 0xaa, 0xb0, 0x88, 0xc3, 0xc5, 0xc7};
        return mpu_write_mem(CFG_MOTION_BIAS, 9, regs);
    }
}

/*******************************************************************************
 *  @brief       Generate 6-axis quaternions from the DMP.
 *  In this driver, the 3-axis and 6-axis DMP quaternion features are mutually
 *  exclusive.
 *  @param[in]   enable  1 to enable 6-axis quaternion.
 *  @return      0 if successful.
*******************************************************************************/
int dmp_enable_6x_lp_quat(unsigned char enable){
    unsigned char regs[4];
    if (enable) {
        regs[0] = DINA20;
        regs[1] = DINA28;
        regs[2] = DINA30;
        regs[3] = DINA38;
    } else
        memset(regs, 0xA3, 4);

    mpu_write_mem(CFG_8, 4, regs);

    return mpu_reset_fifo();
}

/*******************************************************************************
 *  @brief      Generate 3-axis quaternions from the DMP.
 *  In this driver, the 3-axis and 6-axis DMP quaternion features are mutually
 *  exclusive.
 *  @param[in]  enable  1 to enable 3-axis quaternion.
 *  @return     0 if successful.
*******************************************************************************/
int dmp_enable_lp_quat(unsigned char enable){
    unsigned char regs[4];
    if (enable) {
        regs[0] = DINBC0;
        regs[1] = DINBC2;
        regs[2] = DINBC4;
        regs[3] = DINBC6;
    }
    else
        memset(regs, 0x8B, 4);

    mpu_write_mem(CFG_LP_QUAT, 4, regs);

    return mpu_reset_fifo();
}


/*******************************************************************************
 *  @brief  Reset FIFO read/write pointers.
 *  @return 0 if successful.
*******************************************************************************/
int mpu_reset_fifo(void){
    uint8_t data;

    data = 0;
    if (i2c_write_byte(IMU_BUS, INT_ENABLE, data))
        return -1;
    if (i2c_write_byte(IMU_BUS, FIFO_EN, data))
        return -1;
    if (i2c_write_byte(IMU_BUS, USER_CTRL, data))
        return -1;

	data = BIT_FIFO_RST | BIT_DMP_RST;
	if (i2c_write_byte(IMU_BUS, USER_CTRL, data))
		return -1;
	usleep(50000);
	data = BIT_DMP_EN | BIT_FIFO_EN;
	if (config.enable_magnetometer)
		data |= BIT_AUX_IF_EN;
	if (i2c_write_byte(IMU_BUS, USER_CTRL, data))
		return -1;

	if (i2c_write_byte(IMU_BUS, INT_ENABLE, BIT_DMP_INT_EN))
		return -1;
	data = 0;
	if (i2c_write_byte(IMU_BUS, FIFO_EN, data))
		return -1;
  
    return 0;
}


/*******************************************************************************
 *  @brief      Enable/disable DMP support.
 *  @param[in]  enable  1 to turn on the DMP.
 *  @return     0 if successful.
*******************************************************************************/
int mpu_set_dmp_state(unsigned char enable){
    unsigned char tmp;

    if (enable) {
        /* Disable data ready interrupt. */
        set_int_enable(0);
        /* Disable bypass mode. */
        mpu_set_bypass(0);
        /* Keep constant sample rate, FIFO rate controlled by DMP. */
        // Set sample rate = 1000/(1 + SMPLRT_DIV)
		int div = (1000/DMP_MAX_RATE)-1;
		i2c_write_byte(IMU_BUS, SMPLRT_DIV, (uint8_t)div);  
        /* Remove FIFO elements. */
        tmp = 0;
        i2c_write_byte(IMU_BUS, 0x23, tmp);
        /* Enable DMP interrupt. */
        set_int_enable(1);
        mpu_reset_fifo();
    } else {
        /* Disable DMP interrupt. */
        set_int_enable(0);
        /* Restore FIFO settings. */
        i2c_write_byte(IMU_BUS, 0x23, 0);
        mpu_reset_fifo();
    }
    return 0;
}

/*******************************************************************************
 *  @brief      Enable/disable data ready interrupt.
 *  If the DMP is on, the DMP interrupt is enabled. Otherwise, the data ready
 *  interrupt is used.
 *  @param[in]  enable      1 to enable interrupt.
 *  @return     0 if successful.
*******************************************************************************/
int set_int_enable(unsigned char enable){
    unsigned char tmp;

    if (dmp_en) {
		#ifdef DEBUG
		printf("setting dmp-driven interrupt to %d\n", enable);
		#endif
        if (enable){
            tmp = BIT_DMP_INT_EN;
			tmp = BIT_DATA_RDY_EN;
		}
        else
            tmp = 0x00;
        if (i2c_write_byte(IMU_BUS, INT_ENABLE, tmp))
            return -1;
    } else {
		#ifdef DEBUG
		printf("setting data-ready interrupt to %d\n", enable);
		#endif
        if (enable)
            tmp = BIT_DATA_RDY_EN;
        else
            tmp = 0x00;
        if (i2c_write_byte(IMU_BUS, INT_ENABLE, tmp))
            return -1;
    }
    return 0;
}


/*******************************************************************************
*
*******************************************************************************/
// void* imu_interrupt_handler(void* ptr){
	// struct pollfd fdset[1];
	// char buf[MAX_BUF];
	// int imu_gpio_fd = gpio_fd_open(INTERRUPT_PIN);
	// fdset[0].fd = imu_gpio_fd;
	// fdset[0].events = POLLPRI; // high-priority interrupt
	// // keep running until the program closes
	// while(interrupt_running) {
		// // system hangs here until IMU FIFO interrupt
// POLL:
		// poll(fdset, 1, POLL_TIMEOUT);        
		// if (fdset[0].revents & POLLPRI) {
			// lseek(fdset[0].fd, 0, SEEK_SET);  
			// read(fdset[0].fd, buf, MAX_BUF);
			
			// // now the magic happens
			// #ifdef DEBUG
			// printf("IMU interrupt recieved\n");
			// #endif
			// i2c_set_device_address(IMU_BUS, IMU_ADDR);
			// i2c_claim_bus(IMU_BUS);
			// // try to load fifo
			// if(read_dmp_fifo(imu_data_ptr)){
				// i2c_release_bus(IMU_BUS);
				// goto POLL;
			// }
			// // user selectable with set_imu_interrupt_func()
			// i2c_release_bus(IMU_BUS);
			// imu_interrupt_func();
		// }
	// }
	// gpio_fd_close(imu_gpio_fd);
	// return 0;
// }

// int set_imu_interrupt_func(int (*func)(void), imu_data_t* data){
	// imu_data_ptr = data;
	// imu_interrupt_func = func;
	// set_int_enable(1);
	// return 0;
// }

// int stop_imu_interrupt_func(){
	// interrupt_running = 0;
	// return set_int_enable(0);
// }


// /*******************************************************************************
// *
// *******************************************************************************/
// int read_dmp_fifo(imu_data_t* data){
    // /* Assumes maximum packet size is gyro (6) + accel (6). */
    // unsigned char raw[32];
    // unsigned short fifo_count;

    // if (!dmp_en){
		// printf("only use mpu_read_fifo in dsmp mode\n");
        // return -1;
	// }
	// i2c_set_device_address(IMU_BUS, IMU_ADDR);
	
	// // check fifo count register to make sure new data is there
    // if (i2c_read_word(IMU_BUS, FIFO_COUNTH, &fifo_count)) return -1;
	// #ifdef DEBUG
	// printf("fifo_count: %d\n", fifo_count);
	// #endif
	
	// // check that the right number of bytes are in fifo
    // if (fifo_count |= fifo_packet_length){
		// #ifdef DEBUG
		// printf("WARNING: fifo_count not as expected\n");
		// #endif
		// // check overflow bit
        // i2c_read_byte(IMU_BUS, INT_STATUS, raw);
        // if (raw[0] & BIT_FIFO_OVERFLOW) {
			// printf("WARNING: IMU FIFO OVERFLOW\n");
			// printf("Resetting FIFO\n");
            // mpu_reset_fifo();
            // return -1;
        // }
	// }

	// // read it in!
    // if (i2c_read_bytes(IMU_BUS, FIFO_R_W, fifo_packet_length, raw))
        // return -1;

	
	// // Read Accel values and load into imu_data struct
	// // Turn the MSB and LSB into a signed 16-bit value
	// data->raw_accel[0] = (int16_t)(((uint16_t)raw[0]<<8)|raw[1]);
	// data->raw_accel[1] = (int16_t)(((uint16_t)raw[2]<<8)|raw[3]);
	// data->raw_accel[2] = (int16_t)(((uint16_t)raw[4]<<8)|raw[5]);
	// // Fill in real unit values
	// data->accel[0] = data->raw_accel[0] * data->accel_to_ms2;
	// data->accel[1] = data->raw_accel[1] * data->accel_to_ms2;
	// data->accel[2] = data->raw_accel[2] * data->accel_to_ms2;
	
	// // Read gyro values and load into imu_data struct
	// // Turn the MSB and LSB into a signed 16-bit value
	// data->raw_gyro[0] = (int16_t)(((int16_t)raw[0+6]<<8)|raw[1+6]);
	// data->raw_gyro[1] = (int16_t)(((int16_t)raw[2+6]<<8)|raw[3+6]);
	// data->raw_gyro[2] = (int16_t)(((int16_t)raw[4+6]<<8)|raw[5+6]);
	// // Fill in real unit values
	// data->gyro[0] = data->raw_gyro[0] * data->gyro_to_degs;
	// data->gyro[1] = data->raw_gyro[1] * data->gyro_to_degs;
	// data->gyro[2] = data->raw_gyro[2] * data->gyro_to_degs;
	

    // return 0;
// }




// int setXGyroOffset(int16_t offset) {
	// uint16_t new = offset;
	// const unsigned char msb = (unsigned char)((new&0xff00)>>8);
	// const unsigned char lsb = (new&0x00ff);//get LSB
	// //printf("writing: 0x%x 0x%x 0x%x \n", new, msb, lsb);
	// linux_i2c_write(MPU_ADDR, MPU6050_RA_XG_OFFS_USRH, 1, &msb);
	// return linux_i2c_write(MPU_ADDR, MPU6050_RA_XG_OFFS_USRL, 1, &lsb);
// }

// int setYGyroOffset(int16_t offset) {
	// uint16_t new = offset;
	// const unsigned char msb = (unsigned char)((new&0xff00)>>8);
	// const unsigned char lsb = (new&0x00ff);//get LSB
	// //printf("writing: 0x%x 0x%x 0x%x \n", new, msb, lsb);
	// linux_i2c_write(MPU_ADDR, MPU6050_RA_YG_OFFS_USRH, 1, &msb);
	// return linux_i2c_write(MPU_ADDR, MPU6050_RA_YG_OFFS_USRL, 1, &lsb);
// }

// int setZGyroOffset(int16_t offset) {
	// uint16_t new = offset;
	// const unsigned char msb = (unsigned char)((new&0xff00)>>8);
	// const unsigned char lsb = (new&0x00ff);//get LSB
	// //printf("writing: 0x%x 0x%x 0x%x \n", new, msb, lsb);
	// linux_i2c_write(MPU_ADDR, MPU6050_RA_ZG_OFFS_USRH, 1, &msb);
	// return linux_i2c_write(MPU_ADDR, MPU6050_RA_ZG_OFFS_USRL, 1, &lsb);
// }

// int loadGyroCalibration(){
	// FILE *cal;
	// char file_path[100];

	// // construct a new file path string
	// strcpy (file_path, CONFIG_DIRECTORY);
	// strcat (file_path, GYRO_CAL_FILE);
	
	// // open for reading
	// cal = fopen(file_path, "r");
	// if (cal == 0) {
		// // calibration file doesn't exist yet
		// return -1;
	// }
	// else{
		// int xoffset, yoffset, zoffset;
		// fscanf(cal,"%d\n%d\n%d\n", &xoffset, &yoffset, &zoffset);
		// if(setXGyroOffset((int16_t)xoffset)){
			// printf("problem setting gyro offset\n");
			// return -1;
		// }
		// if(setYGyroOffset((int16_t)yoffset)){
			// printf("problem setting gyro offset\n");
			// return -1;
		// }
		// if(setZGyroOffset((int16_t)zoffset)){
			// printf("problem setting gyro offset\n");
			// return -1;
		// }
	// }
	// fclose(cal);
	// return 0;
// }

// void tilt_compensate(quaternion_t magQ, quaternion_t unfusedQ)
// {
	// quaternion_t unfusedConjugateQ;
	// quaternion_t tempQ;

	// quaternionConjugate(unfusedQ, unfusedConjugateQ);
	// quaternionMultiply(magQ, unfusedConjugateQ, tempQ);
	// quaternionMultiply(unfusedQ, tempQ, magQ);
// }

// int data_fusion(imu_data_t *imu)
// {
	// quaternion_t dmpQuat;
	// vector3d_t dmpEuler;
	// quaternion_t magQuat;
	// quaternion_t unfusedQuat;
	// float deltaDMPYaw;
	// float deltaMagYaw;
	// float newMagYaw;
	// float newYaw;
	
	// dmpQuat[QUAT_W] = (float)mpu->rawQuat[QUAT_W];
	// dmpQuat[QUAT_X] = (float)mpu->rawQuat[QUAT_X];
	// dmpQuat[QUAT_Y] = (float)mpu->rawQuat[QUAT_Y];
	// dmpQuat[QUAT_Z] = (float)mpu->rawQuat[QUAT_Z];

	// quaternionNormalize(dmpQuat);	
	// quaternionToEuler(dmpQuat, dmpEuler);

	// mpu->fusedEuler[VEC3_X] = dmpEuler[VEC3_X];
	// mpu->fusedEuler[VEC3_Y] = -dmpEuler[VEC3_Y];
	// mpu->fusedEuler[VEC3_Z] = 0;

	// eulerToQuaternion(mpu->fusedEuler, unfusedQuat);

	// deltaDMPYaw = -dmpEuler[VEC3_Z] + mpu->lastDMPYaw;
	// mpu->lastDMPYaw = dmpEuler[VEC3_Z];

	// magQuat[QUAT_W] = 0;
	// magQuat[QUAT_X] = mpu->calibratedMag[VEC3_X];
  	// magQuat[QUAT_Y] = mpu->calibratedMag[VEC3_Y];
  	// magQuat[QUAT_Z] = mpu->calibratedMag[VEC3_Z];

	// tilt_compensate(magQuat, unfusedQuat);

	// newMagYaw = -atan2f(magQuat[QUAT_Y], magQuat[QUAT_X]);

	// if (newMagYaw != newMagYaw) {
		// printf("newMagYaw NAN\n");
		// return -1;
	// }

	// if (newMagYaw < 0.0f)
		// newMagYaw = TWO_PI + newMagYaw;

	// newYaw = mpu->lastYaw + deltaDMPYaw;

	// if (newYaw > TWO_PI)
		// newYaw -= TWO_PI;
	// else if (newYaw < 0.0f)
		// newYaw += TWO_PI;
	 
	// deltaMagYaw = newMagYaw - newYaw;
	
	// if (deltaMagYaw >= (float)M_PI)
		// deltaMagYaw -= TWO_PI;
	// else if (deltaMagYaw < -(float)M_PI)
		// deltaMagYaw += TWO_PI;

	// if (yaw_mixing_factor > 0)
		// newYaw += deltaMagYaw / yaw_mixing_factor;

	// if (newYaw > TWO_PI)
		// newYaw -= TWO_PI;
	// else if (newYaw < 0.0f)
		// newYaw += TWO_PI;

	// mpu->lastYaw = newYaw;

	// if (newYaw > (float)M_PI)
		// newYaw -= TWO_PI;

	// mpu->fusedEuler[VEC3_Z] = newYaw;

	// eulerToQuaternion(mpu->fusedEuler, mpu->fusedQuat);

	// return 0;
// }


// // Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// // of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
// void calibrateMPU9250(float * dest1, float * dest2){  
  // uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  // uint16_t ii, packet_count, fifo_count;
  // int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
// // reset device, reset all registers, clear gyro and accelerometer bias registers
  // i2c_write_byte(IMU_BUS PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  // wait(0.1);  
   
// // get stable time source
// // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  // i2c_write_byte(IMU_BUS PWR_MGMT_1, 0x01);  
  // i2c_write_byte(IMU_BUS PWR_MGMT_2, 0x00); 
  // wait(0.2);
  
// // Configure device for bias calculation
  // i2c_write_byte(IMU_BUS INT_ENABLE, 0x00);   // Disable all interrupts
  // i2c_write_byte(IMU_BUS FIFO_EN, 0x00);      // Disable FIFO
  // i2c_write_byte(IMU_BUS PWR_MGMT_1, 0x00);   // Turn on internal clock source
  // i2c_write_byte(IMU_BUS I2C_MST_CTRL, 0x00); // Disable I2C master
  // i2c_write_byte(IMU_BUS USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  // i2c_write_byte(IMU_BUS USER_CTRL, 0x0C);    // Reset FIFO and DMP
  // wait(0.015);
  
// // Configure MPU9250 gyro and accelerometer for bias calculation
  // i2c_write_byte(IMU_BUS CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  // i2c_write_byte(IMU_BUS SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  // i2c_write_byte(IMU_BUS GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  // i2c_write_byte(IMU_BUS ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  // uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  // uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// // Configure FIFO to capture accelerometer and gyro data for bias calculation
  // i2c_write_byte(IMU_BUS USER_CTRL, 0x40);   // Enable FIFO  
  // i2c_write_byte(IMU_BUS FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
  // wait(0.04); // accumulate 40 samples in 80 milliseconds = 480 bytes

// // At end of sample accumulation, turn off FIFO sensor read
  // i2c_write_byte(IMU_BUS FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  // i2c_read_bytes(IMU_ADDR, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  // fifo_count = ((uint16_t)data[0] << 8) | data[1];
  // packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  // for (ii = 0; ii < packet_count; ii++) {
    // int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    // i2c_read_bytes(IMU_ADDR, FIFO_R_W, 12, &data[0]); // read data for averaging
    // accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    // accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    // accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    // gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    // gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    // gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    // accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    // accel_bias[1] += (int32_t) accel_temp[1];
    // accel_bias[2] += (int32_t) accel_temp[2];
    // gyro_bias[0]  += (int32_t) gyro_temp[0];
    // gyro_bias[1]  += (int32_t) gyro_temp[1];
    // gyro_bias[2]  += (int32_t) gyro_temp[2];
            
// }
    // accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    // accel_bias[1] /= (int32_t) packet_count;
    // accel_bias[2] /= (int32_t) packet_count;
    // gyro_bias[0]  /= (int32_t) packet_count;
    // gyro_bias[1]  /= (int32_t) packet_count;
    // gyro_bias[2]  /= (int32_t) packet_count;
    
  // if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  // else {accel_bias[2] += (int32_t) accelsensitivity;}
 
// // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  // data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  // data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  // data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  // data[3] = (-gyro_bias[1]/4)       & 0xFF;
  // data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  // data[5] = (-gyro_bias[2]/4)       & 0xFF;

// /// Push gyro biases to hardware registers
// /*  i2c_write_byte(IMU_BUS XG_OFFSET_H, data[0]);
  // i2c_write_byte(IMU_BUS XG_OFFSET_L, data[1]);
  // i2c_write_byte(IMU_BUS YG_OFFSET_H, data[2]);
  // i2c_write_byte(IMU_BUS YG_OFFSET_L, data[3]);
  // i2c_write_byte(IMU_BUS ZG_OFFSET_H, data[4]);
  // i2c_write_byte(IMU_BUS ZG_OFFSET_L, data[5]);
// */
  // dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  // dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  // dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// // the accelerometer biases calculated above must be divided by 8.

  // int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  // i2c_read_bytes(IMU_ADDR, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  // accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  // i2c_read_bytes(IMU_ADDR, YA_OFFSET_H, 2, &data[0]);
  // accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  // i2c_read_bytes(IMU_ADDR, ZA_OFFSET_H, 2, &data[0]);
  // accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  
  // uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  // uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  // for(ii = 0; ii < 3; ii++) {
    // if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  // }

  // // Construct total accelerometer bias, including calculated average accelerometer bias from above
  // accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  // accel_bias_reg[1] -= (accel_bias[1]/8);
  // accel_bias_reg[2] -= (accel_bias[2]/8);
 
  // data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  // data[1] = (accel_bias_reg[0])      & 0xFF;
  // data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  // data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  // data[3] = (accel_bias_reg[1])      & 0xFF;
  // data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  // data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  // data[5] = (accel_bias_reg[2])      & 0xFF;
  // data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// // Apparently this is not working for the acceleration biases in the MPU-9250
// // Are we handling the temperature correction bit properly?
// // Push accelerometer biases to hardware registers
// /*  i2c_write_byte(IMU_BUS XA_OFFSET_H, data[0]);
  // i2c_write_byte(IMU_BUS XA_OFFSET_L, data[1]);
  // i2c_write_byte(IMU_BUS YA_OFFSET_H, data[2]);
  // i2c_write_byte(IMU_BUS YA_OFFSET_L, data[3]);
  // i2c_write_byte(IMU_BUS ZA_OFFSET_H, data[4]);
  // i2c_write_byte(IMU_BUS ZA_OFFSET_L, data[5]);
// */
// // Output scaled accelerometer biases for manual subtraction in the main program
   // dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   // dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   // dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
// }





