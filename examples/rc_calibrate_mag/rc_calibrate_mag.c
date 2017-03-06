/*******************************************************************************
* calibrate_mag.c
*
* James Strawson - 2016
* This function serves as a command line interface for the calibrate_mag
* routine. If the routine is successful, a new magnetometer calibration file
* will be saved which is loaded automatically the next time the IMU is used.
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

int main(){
	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}

	printf("\n");
	printf("This will sample the magnetometer for the next 15 seconds\n");
	printf("Rotate the cape around in the air through as many orientations\n");
	printf("as possible to collect sufficient data for calibration\n");
	printf("Press ENTER to continue or anything else to quit\n");
	if(rc_continue_or_quit()<1){
		rc_cleanup();
		return -1;
	}

	printf("spin spin spin!!!\n\n");
	// wait for the user to actually start 
	sleep(2);

	if(rc_calibrate_mag_routine()<0){
		printf("Failed to complete magnetometer calibration\n");
		rc_cleanup();
		return -1;
	}

	printf("\nmagnetometer calibration file written\n");
	printf("run rc_test_imu to check performance\n");

	rc_cleanup();
	return 0;
}
