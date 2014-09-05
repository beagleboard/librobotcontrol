// prints normalized DSM2 Satellite Receiver Data
// normalized to +- 1 for bilinear joysticks
// normalized from 0-1 for throttle and 2-position switches
// James Strawson 2014

#include <robotics_cape.h>

int main(){
	initialize_cape();
	if(initialize_dsm2()){
		// if init returns -1 if there was a problem 
		// most likely no calibration file found
		printf("run calibrate_dsm2 first\n");
		return -1;
	}
	
	int i;
	while(get_state()!=EXITING){
		if(is_new_dsm2_data()){	
			//print all channels
			printf("\r");
			for(i=0;i<RC_CHANNELS;i++){
				printf("ch%d %0.2f  ", i+1, get_dsm2_ch_normalized(i+1));
			}
			fflush(stdout);
		}
		else{
			printf("\rNo New Radio Packets ");
		}
		fflush(stdout);
		usleep(100000);
	}
	cleanup_cape();
	return 0;
}