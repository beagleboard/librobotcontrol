/**
 * <rc/mpu.h>
 *
 * @brief      A userspace C interface for the invensense MPU6050, MPU6500,
 * MPU9150, and MPU9250.
 *
 * This API allows the user to configure this IMU in two modes: NORMAL and DMP
 *
 * ##Normal Mode
 *
 * The accelerometer, gyroscope, magnetometer, and thermometer can be
 * read directly at any time. To use this mode, call rc_mpu_initialize() with
 * your imu_config and imu_data structs as arguments as defined below. You can
 * then call rc_mpu_read_accel, rc_mpu_read_gyro, rc_mpu_read_mag, or
 * rc_mpu_read_temp at any time to get the latest sensor values.
 *
 * ##DMP Mode
 *
 * Stands for Digital Motion Processor which is a feature of the MPU9250.
 * in this mode, the DMP will sample the sensors internally and fill a FIFO
 * buffer with the data at a fixed rate. Furthermore, the DMP will also
 * calculate a filtered orientation quaternion which is placed in the same
 * buffer. When new data is ready in the buffer, the IMU sends an interrupt to
 * the BeagleBone triggering the buffer read followed by the execution of a
 * function of your choosing set with the rc_mpu_set_dmp_callback() function.
 *
 * @author     James Strawson
 * @date       1/19/2018
 *
 * @addtogroup IMU_MPU
 * @{
 */

#ifndef RC_MPU_H
#define RC_MPU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <pthread.h>

#define RC_MPU_DEFAULT_I2C_ADDR	0x68 ///< default i2c address if AD0 is left low
#define RC_MPU_ALT_I2C_ADDR	0x69 ///< alternate i2c address if AD0 pin pulled high


// defines for index location within TaitBryan and quaternion vectors
#define TB_PITCH_X	0 ///< Index of the dmp_TaitBryan[] array corresponding to the Pitch (X) axis.
#define TB_ROLL_Y	1 ///< Index of the dmp_TaitBryan[] array corresponding to the Roll (Y) axis.
#define TB_YAW_Z	2 ///< Index of the dmp_TaitBryan[] array corresponding to the Yaw (Z) axis.
#define QUAT_W		0 ///< First index of the dmp_quat[] quaternion vector
#define QUAT_X		1 ///< Second index of the dmp_quat[] quaternion vector
#define QUAT_Y		2 ///< Third index of the dmp_quat[] quaternion vector
#define QUAT_Z		3 ///< Fourth index of the dmp_quat[] quaternion vector

#define DEG_TO_RAD	0.0174532925199	///< multiply to convert degrees to radians
#define RAD_TO_DEG	57.295779513	///< multiply to convert radians to degrees
#define MS2_TO_G	0.10197162129	///< multiply to convert m/s^2 to G
#define G_TO_MS2	9.80665		///< multiply to convert G to m/s^2, standard gravity definition

/**
 * @brief      accelerometer full scale range options
 *
 * The user may choose from 4 full scale ranges of the accelerometer and
 * gyroscope. They have units of gravity (G) and degrees per second (DPS) The
 * defaults values are A_FSR_2G and G_FSR_2000DPS respectively.
 */
typedef enum rc_mpu_accel_fsr_t{
	ACCEL_FSR_2G,
	ACCEL_FSR_4G,
	ACCEL_FSR_8G,
	ACCEL_FSR_16G
} rc_mpu_accel_fsr_t;

/**
 * @brief      gyroscope full scale range options
 *
 * The user may choose from 4 full scale ranges of the accelerometer and
 * gyroscope. They have units of gravity (G) and degrees per second (DPS) The
 * defaults values are A_FSR_2G and G_FSR_2000DPS respectively.
 */
typedef enum rc_mpu_gyro_fsr_t{
	GYRO_FSR_250DPS,
	GYRO_FSR_500DPS,
	GYRO_FSR_1000DPS,
	GYRO_FSR_2000DPS
} rc_mpu_gyro_fsr_t;

/**
 * @brief      accelerometer digital low-pass filter options
 *
 * The user may choose from 7 digital low pass filter constants for the
 * accelerometer and gyroscope. The filter runs at 1kz and helps to reduce
 * sensor noise when sampling more slowly. The default values are ACCEL_DLPF_184
 * GYRO_DLPF_184. Lower cut-off frequencies incur phase-loss in measurements.
 * Number is cutoff frequency in hz.
 */
typedef enum rc_mpu_accel_dlpf_t{
	ACCEL_DLPF_OFF,
	ACCEL_DLPF_460,
	ACCEL_DLPF_184,
	ACCEL_DLPF_92,
	ACCEL_DLPF_41,
	ACCEL_DLPF_20,
	ACCEL_DLPF_10,
	ACCEL_DLPF_5
} rc_mpu_accel_dlpf_t;

/**
 * @brief      gyroscope digital low-pass filter options
 *
 * The user may choose from 7 digital low pass filter constants for the
 * accelerometer and gyroscope. The filter runs at 1kz and helps to reduce
 * sensor noise when sampling more slowly. The default values are ACCEL_DLPF_184
 * GYRO_DLPF_184. Lower cut-off frequencies incur phase-loss in measurements.
 * Number is cutoff frequency in hz.
 */
typedef enum rc_mpu_gyro_dlpf_t{
	GYRO_DLPF_OFF,
	GYRO_DLPF_250,
	GYRO_DLPF_184,
	GYRO_DLPF_92,
	GYRO_DLPF_41,
	GYRO_DLPF_20,
	GYRO_DLPF_10,
	GYRO_DLPF_5
} rc_mpu_gyro_dlpf_t;


/**
 * @brief      Orientation of the sensor.
 *
 * This is only applicable when operating in DMP mode. This is the orientation
 * that the DMP consideres neutral, aka where roll/pitch/yaw are zero.
 */
typedef enum rc_mpu_orientation_t{
	ORIENTATION_Z_UP	= 136,
	ORIENTATION_Z_DOWN	= 396,
	ORIENTATION_X_UP	= 14,
	ORIENTATION_X_DOWN	= 266,
	ORIENTATION_Y_UP	= 112,
	ORIENTATION_Y_DOWN	= 336,
	ORIENTATION_X_FORWARD	= 133,
	ORIENTATION_X_BACK	= 161
} rc_mpu_orientation_t;

/**
 * @brief      configuration of the mpu sensor
 *
 * Configuration struct passed to rc_mpu_initialize and rc_mpu_initialize_dmp.
 * It is best to get the default config with rc_mpu_default_config() function
 * first and modify from there.
 */
typedef struct rc_mpu_config_t{
	/** @name physical connection configuration */
	///@{
	int gpio_interrupt_pin_chip;	///< gpio pin, default 3 on Robotics Cape and BB Blue
	int gpio_interrupt_pin;		///< gpio pin, default 21 on Robotics Cape and BB Blue
	int i2c_bus;			///< which bus to use, default 2 on Robotics Cape and BB Blue
	uint8_t i2c_addr;		///< default is 0x68, pull pin ad0 high to make it 0x69
	int show_warnings;		///< set to 1 to print i2c_bus warnings for debug
	///@}

	/** @name accelerometer, gyroscope, and magnetometer configuration */
	///@{
	rc_mpu_accel_fsr_t accel_fsr;	///< accelerometer full scale range, default ACCEL_FSR_8G
	rc_mpu_gyro_fsr_t gyro_fsr;	///< gyroscope full scale range, default GYRO_FSR_2000DPS
	rc_mpu_accel_dlpf_t accel_dlpf;	///< internal low pass filter cutoff, default ACCEL_DLPF_184
	rc_mpu_gyro_dlpf_t gyro_dlpf;	///< internal low pass filter cutoff, default GYRO_DLPF_184
	int enable_magnetometer;	///< magnetometer use is optional, set to 1 to enable, default 0 (off)
	///@}

	/** @name DMP settings, only used with DMP mode */
	///@{
	int dmp_sample_rate;		///< sample rate in hertz, 200,100,50,40,25,20,10,8,5,4
	int dmp_fetch_accel_gyro;	///< set to 1 to optionally raw accel/gyro when reading DMP quaternion, default: 0 (off)
	int dmp_auto_calibrate_gyro;	///< set to 1 to let DMP auto calibrate the gyro while in use, default: 0 (off)
	rc_mpu_orientation_t orient;	///< DMP orientation matrix, see rc_mpu_orientation_t
	double compass_time_constant;	///< time constant (seconds) for filtering compass with gyroscope yaw value, default 25
	int dmp_interrupt_sched_policy;	///< Scheduler policy for DMP interrupt handler and user callback, default SCHED_OTHER
	int dmp_interrupt_priority;	///< scheduler priority for DMP interrupt handler and user callback, default 0
	int read_mag_after_callback;	///< reads magnetometer after DMP callback function to improve latency, default 1 (true)
	int mag_sample_rate_div;	///< magnetometer_sample_rate = dmp_sample_rate/mag_sample_rate_div, default: 4
	int tap_threshold;		///< threshold impulse for triggering a tap in units of mg/ms
	///@}

} rc_mpu_config_t;


/**
 * @brief      data struct populated with new sensor data
 *
 * This is the container for holding the sensor data. The user is intended to
 * make their own instance of this struct and pass its pointer to imu read
 * functions. new data will then be written back into the user's instance of the
 * data struct.
 */
typedef struct rc_mpu_data_t{
	/** @name base sensor readings in real units */
	///@{
	double accel[3];	///< accelerometer (XYZ) in units of m/s^2
	double gyro[3];		///< gyroscope (XYZ) in units of degrees/s
	double mag[3];		///< magnetometer (XYZ) in units of uT
	double temp;		///< thermometer, in units of degrees Celsius
	///@}

	/** @name 16 bit raw adc readings and conversion rates*/
	///@{
	int16_t raw_gyro[3];	///< raw gyroscope (XYZ)from 16-bit ADC
	int16_t raw_accel[3];	///< raw accelerometer (XYZ) from 16-bit ADC
	double accel_to_ms2;	///< conversion rate from raw accelerometer to m/s^2
	double gyro_to_degs;	///< conversion rate from raw gyroscope to degrees/s
	///@}

	/** @name DMP data */
	///@{
	double dmp_quat[4];	///< normalized quaternion from DMP based on ONLY Accel/Gyro
	double dmp_TaitBryan[3];///< Tait-Bryan angles (roll pitch yaw) in radians from DMP based on ONLY Accel/Gyro
	int tap_detected;	///< set to 1 if there was a tap detect on the last dmp sample, reset to 0 on next sample
	int last_tap_direction;	///< direction of last tap, 1-6 corresponding to X+ X- Y+ Y- Z+ Z-
	int last_tap_count;	///< current counter of rapid consecutive taps
	///@}

	/** @name fused DMP data filtered with magnetometer */
	///@{
	double fused_quat[4];		///< fused and normalized quaternion
	double fused_TaitBryan[3];	///< fused Tait-Bryan angles (roll pitch yaw) in radians
	double compass_heading;		///< fused heading filtered with gyro and accel data, same as Tait-Bryan yaw
	double compass_heading_raw;	///< unfiltered heading from magnetometer
	///@}
} rc_mpu_data_t;


/** @name common functions */
///@{

/**
 * @brief      Returns an rc_mpu_config_t struct with default settings.
 *
 * Use this as a starting point and modify as you wish.
 *
 * @return     Returns an rc_mpu_config_t struct with default settings.
 */
rc_mpu_config_t rc_mpu_default_config(void);

/**
 * @brief      Resets a config struct to defaults.
 *
 * @param[out] conf  Pointer to config struct to be overwritten
 *
 * @return     0 on success or -1 on failure.
 */
int rc_mpu_set_config_to_default(rc_mpu_config_t* conf);

/**
 * @brief      Powers off the MPU
 *
 * Only call this after powering on the MPU with rc_mpu_initialize or
 * rc_mpu_initialize_dmp. This should geenrally be called at the end of your
 * main function to make sure the MPU is put to sleep.
 *
 * @return     0 on success or -1 on failure.
 */
int rc_mpu_power_off(void);
///@} end common functions


/** @name normal one-shot sampling functions */
///@{

/**
 * @brief      Sets up the MPU in normal one-shot sampling mode.
 *
 * First create an instance of the rc_mpu_data_t struct and pass its pointer to
 * rc_mpu_initialize which will then write to. Also pass an rc_mpu_config_t
 * struct with your configruation settings.
 *
 * This function will populate the  accel_to_ms2 and gyro_to_deg fields of the
 * rc_mpu_data_t struct appropriately based on the user-configured full scale
 * ranges.
 *
 * After this, you may read sensor data at any time with the functions
 * rc_mpu_read_accel(), rc_mpu_read_gyro(), and rc_mpu_read_temp(). The magentometer
 * can also be read with rc_mpu_read_mag() if using an MPU9150 or MPU9250 and the
 * enable_magnetometer field in the rc_mpu_config_t struct has been set to 1.
 *
 * Be sure to power off the MPU at the end of your program with
 * rc_mpu_power_off().
 *
 * @param      data  pointer to user's data struct
 * @param[in]  conf  user congiguration data
 *
 * @return     0 on success or -1 on failure.
 */
int rc_mpu_initialize(rc_mpu_data_t* data, rc_mpu_config_t conf);


/**
 * @brief      Reads accelerometer data from the MPU
 *
 * @param      data  Pointer to user's data struct where new data will be
 * written
 *
 * @return     0 on success or -1 on failure.
 */
int rc_mpu_read_accel(rc_mpu_data_t* data);


/**
 * @brief      Reads gyroscope data from the MPU
 *
 * @param      data  Pointer to user's data struct where new data will be
 * written
 *
 * @return     0 on success or -1 on failure.
 */
int rc_mpu_read_gyro(rc_mpu_data_t* data);


/**
 * @brief      Reads thermometer data from the MPU
 *
 * Note this is the internal termperature of the chip, not abient temperature.
 *
 * @param      data  Pointer to user's data struct where new data will be
 * written
 *
 * @return     0 on success or -1 on failure.
 */
int rc_mpu_read_temp(rc_mpu_data_t* data);


/**
 * @brief      Reads magnetometer data from the MPU
 *
 * Note this requires use of an MPU9150 or MPU9250, the MPU6050 and MPU6500 do
 * not have magnetometers. Additionally, the enable_magnetometer flag must has
 * been set in the user's rc_mpu_config_t when it was passed to
 * rc_mpu_initialize()
 *
 * @param      data  Pointer to user's data struct where new data will be
 * written
 *
 * @return     0 on success or -1 on failure.
 */
int rc_mpu_read_mag(rc_mpu_data_t* data);
///@} end normal one-shot sampling functions


/** @name interrupt-driven DMP mode functions */
///@{

/**
 * @brief      Initializes the MPU in DMP mode, see rc_test_dmp example
 *
 * After calling this the user does not need to call the normal read functions
 * rc_mpu_read_accel(), rc_mpu_read_gyro(), or rc_mpu_read mag(). Instead the
 * data will automatically be read into the user's data struct at the
 * dmp_sample_rate set in the config struct.
 *
 * @param      data  Pointer to user's data struct where new data will be
 * written
 * @param[in]  conf  User's configuration struct
 *
 * @return     0 on success or -1 on failure.
 */
int rc_mpu_initialize_dmp(rc_mpu_data_t* data, rc_mpu_config_t conf);


/**
 * @brief      Sets the callback function that will be triggered when new DMP
 * data is ready.
 *
 * @param[in]  func  user's callback function
 *
 * @return     0 on success or -1 on failure.
 */
int rc_mpu_set_dmp_callback(void (*func)(void));


/**
 * @brief      blocking function that returns once new DMP data is available
 *
 * @return     Returns 0 once new data is available, 1 if the MPU is shutting
 * down due to rc_mpu_power_off, or -1 on error.
 */
int rc_mpu_block_until_dmp_data(void);


/**
 * @brief      calculates number of nanoseconds since the last DMP interrupt
 *
 * @return     nanoseconds since last interrupt, or -1 if no interrupt received
 * yet.
 */
int64_t rc_mpu_nanos_since_last_dmp_interrupt(void);


/**
 * @brief      sets the callback function triggered when a tap is detected
 *
 * @param[in]  func  user's callback function
 *
 * @return     0 on success or -1 on failure.
 */
int rc_mpu_set_tap_callback(void (*func)(int direction, int counter));


/**
 * @brief      blocking function that returns when a tap is detected
 *
 * @return     Returns 0 once a tap is detected, 1 if the MPU is shutting down
 * due to rc_mpu_power_off(), or -1 on error.
 */
int rc_mpu_block_until_tap(void);


/**
 * @brief      calculates nanoseconds since last tap was detected
 *
 * @return     nanoseconds since last tap, or -1 if no tap has been detected
 * yet.
 */
int64_t rc_mpu_nanos_since_last_tap(void);
///@} end interrupt-driven DMP mode functions



/** @name calibration functions */
///@{

/**
 * @brief      Runs gyroscope calibration routine
 *
 * This should generally not be used by the user unless they absolutely want to
 * calibrate the gyroscope inside their own program. Instead call the
 * rc_calibrate_gyro example program.
 *
 * @param[in]  conf  Config struct, only used to configure i2c bus and address.
 *
 * @return     0 on success, -1 on failure
 */
int rc_mpu_calibrate_gyro_routine(rc_mpu_config_t conf);


/**
 * @brief      Runs magnetometer calibration routine
 *
 * This should generally not be used by the user unless they absolutely want to
 * calibrate the magnetometer inside their own program. Instead call the
 * rc_calibrate_mag example program.
 *
 * @param[in]  conf  Config struct, only used to configure i2c bus and address.
 *
 * @return     0 on success, -1 on failure
 */
int rc_mpu_calibrate_mag_routine(rc_mpu_config_t conf);


/**
 * @brief      Runs accelerometer calibration routine
 *
 * This should generally not be used by the user unless they absolutely want to
 * calibrate the accelerometer inside their own program. Instead call the
 * rc_calibrate_accel example program.
 *
 * @param[in]  conf  Config struct, only used to configure i2c bus and address.
 *
 * @return     0 on success, -1 on failure
 */
int rc_mpu_calibrate_accel_routine(rc_mpu_config_t conf);


/**
 * @brief      Checks if a gyro calibration file is saved to disk
 *
 * generally used to warn the user that they are running a program without
 * calibration. Can also be used to decide if calibration should be done at the
 * beginning of user's program.
 *
 * @return     1 if calibrated, 0 if not
 */
int rc_mpu_is_gyro_calibrated(void);


/**
 * @brief      Checks if a magnetometer calibration file is saved to disk
 *
 * generally used to warn the user that they are running a program without
 * calibration. Can also be used to decide if calibration should be done at the
 * beginning of user's program.
 *
 * @return     1 if calibrated, 0 if not
 */
int rc_mpu_is_mag_calibrated(void);


/**
 * @brief      Checks if an accelerometer calibration file is saved to disk
 *
 * generally used to warn the user that they are running a program without
 * calibration. Can also be used to decide if calibration should be done at the
 * beginning of user's program.
 *
 * @return     1 if calibrated, 0 if not
 */
int rc_mpu_is_accel_calibrated(void);



///@} end calibration functions

#ifdef __cplusplus
}
#endif

#endif // RC_MPU_H

/** @}  end group IMU_MPU*/

