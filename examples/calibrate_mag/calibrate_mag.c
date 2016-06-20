/*******************************************************************************
* calibrate_mag.c
* James Strawson - 2016
*
* This function serves as a command line interface for the calibrate_mag
* routine. If the routine is successful, a new magnetometer calibration file
* will be saved which is loaded automatically the next time the IMU is used.
*******************************************************************************/

#include <robotics_cape.h>
#include <useful_includes.h>

int main(){
	if(initialize_cape()<0){
		printf("Failed to initialize cape, exiting\n");
		return -1;
	}
	printf("\n");
	printf("This will sample the magnetometer for the next few seconds\n");
	printf("Rotate the cape around in the air through as many orientations\n");
	printf("as possible to collect sufficient data for callibration\n");
	printf("Press ENTER to continue or anything else to quit\n");
	if(continue_or_quit()<0){
		cleanup_cape();
		return -1;
	}
	
	if(calibrate_mag_routine()<0){
		printf("Failed to complete magnetometer calibration\n");
		cleanup_cape();
		return -1;
	}
	
	printf("\nmagnetometer calibration file written\n");
	printf("run test_imu to check performance\n");
		
	cleanup_cape();
	return 0;
}
