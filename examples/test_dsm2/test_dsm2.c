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
	printf("1:Thr  ");
	printf("2:Roll ");
	printf("3:Pitch ");
	printf("4:Yaw  ");
	printf("5:Kill  ");
	printf("6:Mode ");
	printf("7:Aux1 ");
	printf("8:Aux2 ");
	printf("9:Aux3");
	printf("\n");
	
	int i;
	while(get_state()!=EXITING){
		if(is_new_dsm2_data()){	
			//print all channels
			printf("\r");
			for(i=0;i<RC_CHANNELS;i++){
				printf("%0.2f   ", get_dsm2_ch_normalized(i+1));
			}
			fflush(stdout);
		}
		else{
			printf("\rNo New Radio Packets ");
		}
		fflush(stdout);
		usleep(10000);
	}
	cleanup_cape();
	return 0;
}