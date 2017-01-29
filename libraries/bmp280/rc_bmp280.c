/*******************************************************************************
*  rc_bmp280.c
*******************************************************************************/

#include "../roboticscape.h"
#include "../rc_defs.h"
#include "rc_bmp280_defs.h"

#include <stdio.h>
#include <math.h>
#include <unistd.h>

typedef struct bmp280_cal_t{
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
	
	float sea_level_pa;

}bmp280_cal_t;


typedef struct bmp280_data_t{
	float temp;
	float alt;
	float pressure;
}bmp280_data_t;


// one global instance of each struct
bmp280_cal_t cal;
bmp280_data_t data;


/*******************************************************************************
* int rc_initialize_barometer(rc_bmp_oversample_t oversample, rc_bmp_filter_t filter)
*
* sets up the i2c bus and barometer for continuous sampling and internal
* filtering. 
*******************************************************************************/
int rc_initialize_barometer(rc_bmp_oversample_t oversample, rc_bmp_filter_t filter){
	uint8_t buf[24];
	uint8_t c;
	int i;
	// make sure the bus is not currently in use by another thread
	// do not proceed to prevent interfering with that process
	if(rc_i2c_get_in_use_state(BMP_BUS)){
		printf("i2c bus claimed by another process\n");
		printf("Continuing with rc_initialize_barometer() anyway.\n");
	}
	
	// initialize the bus
	if(rc_i2c_init(BMP_BUS,BMP_ADDR)<0){
		printf("ERROR: failed to initialize i2c bus\n");
		printf("aborting rc_initialize_barometer\n");
		return -1;
	}

	// claiming the bus does no guarantee other code will not interfere 
	// with this process, but best to claim it so other code can check
	// like we did above
	rc_i2c_claim_bus(BMP_BUS);
	
	// reset the barometer
	if(rc_i2c_write_byte(BMP_BUS, BMP280_RESET_REG, BMP280_RESET_WORD)<0){
		printf("ERROR: failed to send reset byte to barometer\n");
		printf("aborting initialize_bmp\n");
		rc_i2c_release_bus(BMP_BUS);
		return -1;
	}
	
	// check the chip ID register
	if(rc_i2c_read_byte(BMP_BUS, BMP280_CHIP_ID_REG, &c)<0){
		printf("ERROR: read chip_id byte from barometer\n");
		printf("aborting initialize_bmp\n");
		rc_i2c_release_bus(BMP_BUS);
		return -1;
	}
	if(c != BMP280_CHIP_ID){
		printf("ERROR: barometer returned wrong chip_id\n");
		printf("received: %x  expected: %x\n", c, BMP280_CHIP_ID_REG);
		printf("aborting initialize_bmp\n");
		rc_i2c_release_bus(BMP_BUS);
		return -1;
	}
		
	// set up the bmp measurement control register settings
	// no temperature oversampling,  normal continuous read mode
	c = BMP_MODE_NORMAL;
	c |= BMP_TEMP_OVERSAMPLE_1;
	c |= oversample;
	// write the measurement control register
	if(rc_i2c_write_byte(BMP_BUS,BMP280_CTRL_MEAS,c)<0){
		printf("ERROR: can't write to bmp measurement control register\n");
		printf("aborting initialize_bmp\n");
		rc_i2c_release_bus(BMP_BUS);
		return -1;
	}
	
	// set up the filter config register
	c = BMP280_TSB_0; 	// minimal sleep delay between samples
	c |= filter;		// user selectable filter coefficient
	if(rc_i2c_write_byte(BMP_BUS,BMP280_CONFIG,c)<0){
		printf("failed to write to bmp_config register\n");
		printf("aborting initialize_bmp\n");
		rc_i2c_release_bus(BMP_BUS);
		return -1;
	}
	
	
	// keep checking the status register untill the NVM calibration is ready
	// after a short wait
	i = 0;
	c = BMP280_IM_UPDATE_STATUS;
	do{
		usleep(20000);
		if(rc_i2c_read_byte(BMP_BUS, BMP280_STATUS_REG	, &c)<0){
			printf("ERROR: can't read status byte from barometer\n");
			printf("aborting initialize_bmp\n");
			rc_i2c_release_bus(BMP_BUS);
			return -1;
		}
		if(i>10){
			printf("ERROR: factory NVM calibration not available yet\n");
			printf("aborting initialize_bmp\n");
			rc_i2c_release_bus(BMP_BUS);
			return -1;
		}
		i++;
	} while(c&BMP280_IM_UPDATE_STATUS);

	// retrieve the factory NVM calibration data all in one go
	if(rc_i2c_read_bytes(BMP_BUS,BMP280_DIG_T1,24,buf)<0){
		printf("ERROR: failed to load BMP280 factory calibration registers\n");
		printf("aborting rc_initialize_barometer\n");
		rc_i2c_release_bus(BMP_BUS);
		return -1;
	}
	cal.dig_T1 = (uint16_t) ((buf[1] << 8) | buf [0]);
	cal.dig_T2 = (uint16_t) ((buf[3] << 8) | buf [2]);
	cal.dig_T3 = (uint16_t) ((buf[5] << 8) | buf [4]);
	cal.dig_P1 = (uint16_t) ((buf[7] << 8) | buf [6]);
	cal.dig_P2 = (uint16_t) ((buf[9] << 8) | buf [8]);
	cal.dig_P3 = (uint16_t) ((buf[11] << 8) | buf [10]);
	cal.dig_P4 = (uint16_t) ((buf[13] << 8) | buf [12]);
	cal.dig_P5 = (uint16_t) ((buf[15] << 8) | buf [14]);
	cal.dig_P6 = (uint16_t) ((buf[17] << 8) | buf [16]);
	cal.dig_P7 = (uint16_t) ((buf[19] << 8) | buf [18]);
	cal.dig_P8 = (uint16_t) ((buf[21] << 8) | buf [20]);
	cal.dig_P9 = (uint16_t) ((buf[23] << 8) | buf [22]);
	
	// use default for now unless use sets it otherwise
	cal.sea_level_pa = DEFAULT_SEA_LEVEL_PA; 
	
	// release control of the bus
	rc_i2c_release_bus(BMP_BUS);

	// wait for the barometer to settle and get its first internal read.
	usleep(50000);

	// read in data to it's ready for the user to access right away
	if(rc_read_barometer()<0){
		printf("ERROR: failed to read barometer during initialization\n");
		rc_i2c_release_bus(BMP_BUS);
		return -1;
	}
	return 0;
}


/*******************************************************************************
* int rc_power_off_barometer()
*
* Puts the barometer into low power standby
*******************************************************************************/
int rc_power_off_barometer(){
	// make sure the bus is not currently in use by another thread
	// do not proceed to prevent interfering with that process
	if(rc_i2c_get_in_use_state(BMP_BUS)){
		printf("i2c bus claimed by another process\n");
		printf("Continuing with rc_initialize_barometer() anyway.\n");
	}
	// set the i2c address
	if(rc_i2c_set_device_address(BMP_BUS, BMP_ADDR)<0){
		printf("ERROR: failed to set the i2c device address\n");
		rc_i2c_release_bus(BMP_BUS);
		return -1;
	}
	// write the measurement control register to go into sleep mode
	if(rc_i2c_write_byte(BMP_BUS,BMP280_CTRL_MEAS,BMP_MODE_SLEEP)<0){
		printf("ERROR: cannot write bmp_mode_register\n");
		printf("aborting rc_power_off_barometer()\n");
		rc_i2c_release_bus(BMP_BUS);
		return -1;
	}
	
	// release control of the bus
	rc_i2c_release_bus(BMP_BUS);
	return 0;
}


/*******************************************************************************
* int rc_read_barometer()
*
* Reads the newest temperature and pressure measurments from the barometer over
* the I2C bus. To access the data use the rc_bmp_get_temperature(), 
* rc_bmp_get_pressure_pa(), or rc_bmp_get_altitude_m() functions. 
* returns 0 on success, otherwise -1.
*******************************************************************************/
int rc_read_barometer(){
	int64_t var1, var2, var3, var4, t_fine, T, p;
	uint8_t raw[6];
	int32_t adc_P, adc_T;
	
	// check claim bus state to avoid stepping on IMU reads
	if(rc_i2c_get_in_use_state(BMP_BUS)){
		printf("WARNING: i2c bus is claimed, aborting rc_read_barometer\n");
		return -1;
	}
	
	// claim bus for ourselves and set the device address
	rc_i2c_claim_bus(BMP_BUS);
	if(rc_i2c_set_device_address(BMP_BUS, BMP_ADDR)<0){
		printf("ERROR: failed to set the i2c device address\n");
		rc_i2c_release_bus(BMP_BUS);
		return -1;
	}
	
	// if new data is ready, read it in
	if(rc_i2c_read_bytes(BMP_BUS,BMP280_PRESSURE_MSB,6,raw)<0){
		printf("ERROR: failed to read barometer data registers\n");
		rc_i2c_release_bus(BMP_BUS);
		return -1;
	}
	
	// run the numbers, thanks to Bosch for putting this code in their datasheet
	adc_P = (raw[0] << 12)|
			(raw[1] << 4)|(raw[2] >> 4);
	adc_T = (raw[3] << 12)|
			(raw[4] << 4)|(raw[5] >> 4);
	
	var1  = ((((adc_T>>3) - ((int32_t)cal.dig_T1 <<1))) *
			((int32_t)cal.dig_T2)) >> 11;
	var2  = (((((adc_T>>4) - ((int32_t)cal.dig_T1)) *
			((adc_T>>4) - ((int32_t)cal.dig_T1))) >> 12) *
			((int32_t)cal.dig_T3)) >> 14;
			   
	t_fine = var1 + var2;
	
	T  = (t_fine * 5 + 128) >> 8;
	data.temp =  T/100.0;

	var3 = ((int64_t)t_fine) - 128000;
	var4 = var3 * var3 * (int64_t)cal.dig_P6;
	var4 = var4 + ((var3*(int64_t)cal.dig_P5)<<17);
	var4 = var4 + (((int64_t)cal.dig_P4)<<35);
	var3 = ((var3 * var3 * (int64_t)cal.dig_P3)>>8) +
		   ((var3 * (int64_t)cal.dig_P2)<<12);
	var3 = (((((int64_t)1)<<47)+var3))*((int64_t)cal.dig_P1)>>33;

	if (var3 == 0){
		return 0;  // avoid exception caused by division by zero
	}
  
	p = 1048576 - adc_P;
	p = (((p<<31) - var4)*3125) / var3;
	var3 = (((int64_t)cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var4 = (((int64_t)cal.dig_P8) * p) >> 19;

	p = ((p + var3 + var4) >> 8) + (((int64_t)cal.dig_P7) << 4);
	data.pressure = (float)p/256;
	

	data.alt = 44330.0*(1.0 - pow((data.pressure/cal.sea_level_pa), 0.1903));

	rc_i2c_release_bus(BMP_BUS);
	return 0;
}

/*******************************************************************************
* float rc_bmp_get_temperature()
*
* This does not start an I2C transaction but simply returns the temperature in
* degrees celcius that was read by the last call to the rc_read_barometer() 
* function.
*******************************************************************************/
float rc_bmp_get_temperature(){
	return data.temp;
}

/*******************************************************************************
* float rc_bmp_get_pressure_pa()
*
* This does not start an I2C transaction but simply returns the pressure in
* pascals that was read by the last call to the rc_read_barometer() function.
*******************************************************************************/
float rc_bmp_get_pressure_pa(){
	return data.pressure;
}

/*******************************************************************************
* float rc_bmp_get_altitude_m()
*
* This does not start an I2C transaction but simply returns the altitude in 
* meters based on the pressure received by the last call to the rc_read_barometer()
* function. Assuming current pressure at sea level is the default 101325 Pa.
* Use rc_set_sea_level_pressure_pa() if you know the current sea level pressure
* and desire more accuracy. 
*******************************************************************************/
float rc_bmp_get_altitude_m(){
	return data.alt;
}

/*******************************************************************************
* int rc_set_sea_level_pressure_pa(float pa)
*
* If you know the current sea level pressure for your region and weather, you 
* can use this to correct the altititude reading. This is not necessary if you
* only care about differential altitude from a starting point.
*******************************************************************************/
int rc_set_sea_level_pressure_pa(float pa){
	if(pa<80000 || pa >120000){
		printf("ERROR: Please enter a reasonable sea level pressure\n");
		printf("between 80,000 & 120,000 pascals.\n");
		return -1;
	}
	cal.sea_level_pa = pa;
	return 0;
}


