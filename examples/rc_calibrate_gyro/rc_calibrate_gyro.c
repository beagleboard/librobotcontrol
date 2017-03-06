/*******************************************************************************
* rc_calibrate_gyro.c
* James Strawson - 2016
*
* This program exists as an interface to the rc_calibrate_gyro_routine which
* manages collecting gyroscope data for averaging to find the steady state
* offsets.
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

int main(){
	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}

	printf("\nThis program will generate a new gyro calibration file\n");
	printf("keep your beaglebone very still for this procedure.\n");
	printf("Press ENTER to continue or anything else to quit\n");
	if(rc_continue_or_quit()<1){
		rc_cleanup();
		return -1;
	}

	printf("Starting calibration routine\n");
	if(rc_calibrate_gyro_routine()<0){
		printf("Failed to complete gyro calibration\n");
		return -1;
	}

	printf("\ngyro calibration file written\n");
	printf("run rc_test_imu to check performance\n");

	rc_cleanup();
	return 0;
}
