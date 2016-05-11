/*******************************************************************************
* calibrate_dsm2.c
*
* run the build in calibration routine to save the limits of each channel
* transmitted by your dsm2/dsmx radio
*******************************************************************************/


#include <robotics_cape.h>

int main(){
	if(initialize_cape()){
		printf("ERROR: failed to initialize_cape\n");
	}
	if(initialize_dsm2()){
		printf("ERROR: failed to initialize_dsm2\n");
	}	
	if(calibrate_dsm2()){
		printf("ERROR: failed to calibrate_dsm2\n");
	}
	cleanup_cape();
	return 0;
}