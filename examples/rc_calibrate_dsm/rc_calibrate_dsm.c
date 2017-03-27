/*******************************************************************************
* rc_calibrate_dsm2.c
*
* James Strawson 2016
* Running the calibrate_dsm2 example will print out raw data to the console and 
* record the min and max values for each channel. These limits will be saved to
* disk so future dsm2 reads will be scaled correctly.
* 
* Make sure the transmitter and receiver are paired before testing. 
* Use the pair_dsm2 example if you haven't already used a bind plug and standard 
* receiver to pair. The satellite receiver remembers which transmitter it is 
* paired to, not your BeagleBone.
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

int main(){
	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}

	printf("Please connect a DSM sattelite reciever to your Robotics Cape\n");
	printf("and make sure your transmitter is on and paired to the receiver\n");
	printf("\n");
	printf("Press ENTER to continue or anything else to quit\n");
	if(rc_continue_or_quit()<1){
		rc_cleanup();
		return -1;
	}

	// run the calibration routine
	rc_calibrate_dsm_routine();

	// cleanup and close, calibration file already saved by the routine
	rc_cleanup();
	return 0;
}
