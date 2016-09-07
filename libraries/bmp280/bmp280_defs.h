/*******************************************************************************
* bmp280_defs.h
* Matt Atlas 2016
*
* Register definitions for the BMP180 barometer
 ******************************************************************************/
#ifndef _BMP280_H
#define _BMP280_H
   
#define BMP_ADDR    		    0x76
#define DEFAULT_SEA_LEVEL_PA	101325

// temperature and pressure registers
#define BMP280_TEMPERATURE_XLSB	0xFC
#define BMP280_TEMPERATURE_LSB	0xFB
#define BMP280_TEMPERATURE_MSB	0xFA
#define BMP280_PRESSURE_XLSB	0xF9
#define BMP280_PRESSURE_LSB		0xF8
#define BMP280_PRESSURE_MSB		0xF7

// config register and settings
#define BMP280_CONFIG			(0xF5)
// time sample delay settings
#define BMP280_TSB_0			(0x00)
#define BMP280_TSB_63			(0x01<<5)
#define BMP280_TSB_125			(0x02<<5)
#define BMP280_TSB_250			(0x03<<5)
#define BMP280_TSB_500			(0x04<<5)
#define BMP280_TSB_1000			(0x05<<5)
#define BMP280_TSB_2000			(0x06<<5)
#define BMP280_TSB_4000			(0x07<<5)
// IIR filter settings
#define BMP280_FILTER_OFF		(0x00<<2)
#define BMP280_FILTER_2			(0x01<<2)
#define BMP280_FILTER_4			(0x02<<2)
#define BMP280_FILTER_8			(0x03<<2)
#define BMP280_FILTER_16		(0x04<<2)
// SPI enable
#define BMP280_SPI3W_EN			(0x01)

// control measurement register and settings
#define BMP280_CTRL_MEAS		(0xF4)
// temperature oversample
#define BMP_TEMP_OVERSAMPLE_OFF	(0x00)
#define BMP_TEMP_OVERSAMPLE_1	(0x01<<5)
#define BMP_TEMP_OVERSAMPLE_2	(0x02<<5)
#define BMP_TEMP_OVERSAMPLE_4	(0x03<<5)
#define BMP_TEMP_OVERSAMPLE_8	(0x04<<5)
#define BMP_TEMP_OVERSAMPLE_16	(0x05<<5)
// pressure oversample
#define BMP_PRES_OVERSAMPLE_1	(0x01<<2)
#define BMP_PRES_OVERSAMPLE_2	(0x02<<2)
#define BMP_PRES_OVERSAMPLE_4	(0x03<<2)
#define BMP_PRES_OVERSAMPLE_8	(0x04<<2)
#define BMP_PRES_OVERSAMPLE_16	(0x05<<2)
// mode
#define BMP_MODE_SLEEP			0x00
#define BMP_MODE_FORCED			0x01
#define BMP_MODE_NORMAL			0x03

// status register
#define BMP280_STATUS_REG		(0xF3)
#define BMP280_MEAS_STATUS 		(0x01<<3)
#define BMP280_IM_UPDATE_STATUS (0x01)


// reset register, write reset_word to reset
#define BMP280_RESET_REG 		(0xE0)
#define BMP280_RESET_WORD		(0xB6)

// chip ID, should return 0x58
#define BMP280_CHIP_ID_REG		(0xD0)
#define BMP280_CHIP_ID			(0x58)


#define BMP280_CAL26			(0xE1)	// R calibration stored in 0xE1-0xF0






// calibration constant registers
#define BMP280_DIG_T1			0x88
#define BMP280_DIG_T2			0x8A
#define BMP280_DIG_T3			0x8C
#define BMP280_DIG_P1			0x8E
#define BMP280_DIG_P2			0x90
#define BMP280_DIG_P3			0x92
#define BMP280_DIG_P4			0x94
#define BMP280_DIG_P5			0x96
#define BMP280_DIG_P6			0x98
#define BMP280_DIG_P7			0x9A
#define BMP280_DIG_P8			0x9C
#define BMP280_DIG_P9			0x9E





#endif
