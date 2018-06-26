/**
 * @file rc_calibrate_accel.c
 * @example    rc_calibrate_accel
 *
 * @brief      runs the mpu accelerometer calibration routine
 *
 *             If the routine is successful, a new accelerometer calibration
 *             file will be saved which is loaded automatically the next time
 *             the MPU is used.
 *
 *
 * @author     James Strawson
 * @date       1/29/2018
 */


#include <stdio.h>
#include <rc/mpu.h>

// bus for Robotics Cape and BeagleboneBlue is 2
// change this for your platform
#define I2C_BUS 2

int main()
{
	printf("\nThis program will generate a new accelerometer calibration file\n");
	printf("Press any key to continue\n");
	getchar();
	printf("Starting calibration routine\n");

	rc_mpu_config_t config = rc_mpu_default_config();
	config.i2c_bus = I2C_BUS;

	if(rc_mpu_calibrate_accel_routine(config)<0){
		printf("Failed to complete accelerometer calibration\n");
		return -1;
	}

	printf("\nacceleometer calibration file written\n");
	printf("run rc_test_mpu to check performance\n");

	return 0;
}
