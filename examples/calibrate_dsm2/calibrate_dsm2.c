/*******************************************************************************
* calibrate_dsm2.c
*
* run the build in calibration routine to save the limits of each channel
* transmitted by your dsm2/dsmx radio
*******************************************************************************/

#include <useful_includes.h>
#include <robotics_cape.h>

int main(){
	if(initialize_cape()<0){
		printf("ERROR: failed to initialize_cape\n");
	}
	
	printf("Please connect a DSM2 sattelite reciever to your Robotics Cape\n");
	printf("and make sure your transmitter is on and paired to the receiver\n");
	printf("\n");
	printf("Press ENTER to continue or anything else to quit\n");
	if(continue_or_quit()<0){
		cleanup_cape();
		return -1;
	}
	
	// run the calibration routine
	calibrate_dsm2_routine();
	
	// cleanup and close, calibration file already saved by the routine
	cleanup_cape();
	return 0;
}
