/*
DSM2 Spektrum Radio testing and Calibration Function
Running the calibrate_dsm2 executable will print out raw 
data to the terminal along with min and max recorded values. 
These limits will be saved to a calibration file in 
/home/root/calibration to be used with your projects.

James Strawson - 2013
*/


#include <robotics_cape.h>

int rc_mins[RC_CHANNELS];
int rc_maxes[RC_CHANNELS];
int listening;

void *listen_func(void *params){
	//wait for data to start
	printf("waiting for dsm2 connection");
	while(!is_new_dsm2_data()){
		if(get_state()==EXITING || listening==0){
			return 0;
		}
		usleep(5000); 
	}
	
	//start limits at current value
	int j;
	for(j=0;j<RC_CHANNELS;j++){
		rc_mins[j]=get_dsm2_ch_raw(j+1);
		rc_maxes[j]=get_dsm2_ch_raw(j+1);
	}
	
	// record limits until user presses enter
	while(listening && get_state()!=EXITING){
		printf("\r");
		if (is_new_dsm2_data()){
			for(j=0;j<RC_CHANNELS;j++){
				if(get_dsm2_ch_raw(j+1) > 0){ //record only non-zero channels
					if(get_dsm2_ch_raw(j+1)>rc_maxes[j]){
						rc_maxes[j] = get_dsm2_ch_raw(j+1);
					}
					else if(get_dsm2_ch_raw(j+1)<rc_mins[j]){
						rc_mins[j] = get_dsm2_ch_raw(j+1);
					}
					printf(" %d   ",get_dsm2_ch_raw(j+1));
				}
			}
			fflush(stdout);
		}
		usleep(10000); 
	}
	return 0;
}

int main(){
	int i;
	
	initialize_cape();
	initialize_dsm2();
		
	printf("\n\nTurn on your Transmitter and connect receiver.\n");
	printf("Move all sticks and switches through their range of motion.\n");
	printf("Raw min/max positions will display below.\n");
	printf("Press Enter to save and exit.\n");

	printf("1 Thr   ");
	printf("2 Roll  ");
	printf("3 Pitch ");
	printf("4 Yaw   ");
	printf("5 Kill  ");
	printf("6 Mode  ");
	printf("7 Aux1  ");
	printf("8 Aux2  ");
	printf("9 Aux3  ");
	printf("\n");
	
	// start listening
	listening = 1;
	pthread_t  listening_thread;
	pthread_create(&listening_thread, NULL, listen_func, (void*) NULL);
	
	// wait for user to hit enter
	while(getchar() != '\n'){
	}
	
	//stop listening
	listening=0;
	pthread_join(listening_thread, NULL);
	
	//if it looks like new data came in, write calibration file
	if((rc_mins[0]!=0)&&(rc_mins[0]!=rc_maxes[0])){ 
		int fd;
		fd = open(DSM2_CAL_FILE, O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | 
											S_IWUSR | S_IRGRP | S_IROTH);

		if (fd < 0) {
			printf("\n error opening calibration file for writing\n");
			return -1;
		}
		char buff[64];
		for(i=0;i<RC_CHANNELS;i++){
				sprintf(buff, "%d %d\n", rc_mins[i], rc_maxes[i]);
				write(fd, buff, strlen(buff));
		}
		close(fd);
		printf("\nNew calibration file written\n");
	}
	
	cleanup_cape();
	return 0;
}

