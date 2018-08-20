/**
 * @file rc_calibrate_mag.c
 * @example    rc_calibrate_mag
 *
 * @brief      runs the MPU magnetometer calibration routine
 *
 *             If the routine is successful, a new magnetometer calibration file
 *             will be saved which is loaded automatically the next time the MPU
 *             is used.
 *
 * @author     James Strawson
 * @date       1/29/2018
 */

#include <stdio.h>
#include <rc/mpu.h>
#include <rc/time.h>

// bus for Robotics Cape and BeagleboneBlue is 2
// change this for your platform
#define I2C_BUS 2

int main()
{
	printf("\n");
	printf("This will sample the magnetometer for the next 15 seconds\n");
	printf("Rotate the board around in the air through as many orientations\n");
	printf("as possible to collect sufficient data for calibration\n");
	printf("Press any key to continue\n");
	getchar();


	printf("spin spin spin!!!\n\n");
	// wait for the user to actually start
	rc_usleep(2000000);

	rc_mpu_config_t config = rc_mpu_default_config();
	config.i2c_bus = I2C_BUS;

	if(rc_mpu_calibrate_mag_routine(config)<0){
		printf("Failed to complete magnetometer calibration\n");
		return -1;
	}

	printf("\nmagnetometer calibration file written\n");
	printf("run rc_test_mpu to check performance\n");
	return 0;
}
