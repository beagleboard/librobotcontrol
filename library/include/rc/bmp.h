/**
 * <rc/bmp.h>
 *
 * @brief      Interface to the BMP280 barometer
 *
 *
 * @author     James Strawson
 * @date       3/14/2018
 *
 * @addtogroup Barometer_BMP
 * @{
 */

#ifndef RC_BMP_H
#define RC_BMP_H

#ifdef __cplusplus
extern "C" {
#endif


/**
 * Setting given to rc_bmp_init which defines the oversampling
 * done internally to the barometer. For example, if BMP_OVERSAMPLE_16 is used
 * then the barometer will average 16 samples before updating the data
 * registers. The more oversampling used, the slower the data registers will
 * update. You should pick an oversample that provides an update rate slightly
 * slower than the rate at which you will be reading the barometer.
 */
typedef enum rc_bmp_oversample_t{
	BMP_OVERSAMPLE_1  = (0x01<<2), ///< update rate 182 HZ
	BMP_OVERSAMPLE_2  = (0x02<<2), ///< update rate 133 HZ
	BMP_OVERSAMPLE_4  = (0x03<<2), ///< update rate 87 HZ
	BMP_OVERSAMPLE_8  = (0x04<<2), ///< update rate 51 HZ
	BMP_OVERSAMPLE_16 = (0x05<<2)  ///< update rate 28 HZ
} rc_bmp_oversample_t;


/**
 * Setting given to rc_bmp_init to configure the coefficient of the internal
 * first order filter. We recommend disabling the filter with BMP_FILTER_OFF and
 * doing your own filtering with the discrete filter functions below.
 */
typedef enum rc_bmp_filter_t{
	BMP_FILTER_OFF = (0x00<<2),
	BMP_FILTER_2   = (0x01<<2),
	BMP_FILTER_4   = (0x02<<2),
	BMP_FILTER_8   = (0x03<<2),
	BMP_FILTER_16  = (0x04<<2)
}rc_bmp_filter_t;


/**
 * struct to hold the data retreived during one read of the barometer.
 */
typedef struct rc_bmp_data_t{
	double temp_c;		///< temperature in degrees celcius
	double alt_m;		///< altitude in meters
	double pressure_pa;	///< current pressure in pascals
} rc_bmp_data_t;


/**
 * @brief      powers on the barometer and initializes it with the given
 * oversample and filter settings.
 *
 * Optionally call rc_bmp_set_sea_level_pressure_pa afterwards to change the sea
 * level pressure from default.
 *
 * @param[in]  oversample  see rc_bmp_oversample_t
 * @param[in]  filter      see rc_bmp_filter_t
 *
 * @return     0 on success, otherwise -1.
 */
int rc_bmp_init(rc_bmp_oversample_t oversample, rc_bmp_filter_t filter);


/**
 * @brief      If you know the current sea level pressure for your region and
 * weather, you can use this to correct the altititude reading.
 *
 * This is not necessary if you only care about differential altitude from a
 * starting point. Must be called after rc_bmp_init to have an effect.
 *
 * @param[in]  pa    sea level pressure in pascals
 *
 * @return     0 on success, -1 on failure
 */
int rc_bmp_set_sea_level_pressure_pa(double pa);


/**
 * @brief      Puts the barometer into a low power state, should be called at
 * the end of your program before close.
 *
 * @return     0 on success, -1 on failure
 */
int rc_bmp_power_off(void);


/**
 * @brief      Reads the newest temperature and pressure measurments from the
 * barometer over the I2C bus.
 *
 * @param      data  pointer to data struct where new data will be written.
 *
 * @return     0 on success, -1 on failure
 */
int rc_bmp_read(rc_bmp_data_t* data);



#ifdef __cplusplus
}
#endif

#endif // RC_BMP_H

/** @} end group Barometer */