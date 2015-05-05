// prints raw DSM2 Satellite Receiver Data
// and sends this data out to the servos

// James Strawson 2015

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
	printf("5:Kill ");
	printf("6:Mode ");
	printf("7:Aux1 ");
	printf("8:Aux2 ");
	printf("9:Aux3");
	printf("\n");
	
	int i;
	while(get_state()!=EXITING){
		if(is_new_dsm2_data()){	
		
			// send single pulse to each servo
			for(i=0; i<SERVO_CHANNELS; i++){
				send_servo_pulse_us(i+1,get_dsm2_ch_raw(i+1));
			}
		
			//print all channels
			printf("\r");
			for(i=0;i<RC_CHANNELS;i++){
				printf(" %d   ", get_dsm2_ch_raw(i+1));
			}
			fflush(stdout);
		}
		fflush(stdout);
		usleep(20000);
	}
	cleanup_cape();
	return 0;
}