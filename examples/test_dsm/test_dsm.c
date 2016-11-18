/******************************************************************************
* test_dsm.c
*
* James Strawson 2016
* test_dsm2 will print out the normalized dsm2 values.
* Make sure the transmitter and receiver are paired before testing. 
* Use the pair_dsm2 example if you didn't already bind receiver to transmitter
* with a bind plug the old fashioned way. The satellite receiver remembers which
* transmitter it is paired to, not your BeagleBone. 
*
* If the values you read are not normalized between +-1, then you should run the
* calibrate_dsm example to save your particular transmitter's min and max 
* channel values.
*******************************************************************************/

#include "../../libraries/roboticscape-usefulincludes.h"
#include "../../libraries/roboticscape.h"

int main(){
	int i;

	if(initialize_cape()){
		printf("ERROR: failed to initialize cape\n");
		return -1;
	}
	if(initialize_dsm()){
		// if init returns -1 if there was a problem 
		// most likely no calibration file found
		printf("run calibrate_dsm first\n");
		return -1;
	}
	
	printf("\n");
	printf("Make sure transmitter and receiver are bound and on.\n");
	printf("If data is received, the normalized values will be printed\n");
	printf("here along with the bit resolution and the number of channels\n");
	printf("\n");
	printf("If connection is lost the number of seconds since last packet\n");
	printf("will be displayed\n");
	printf("\n");
	

	printf("Waiting for first packet");
	fflush(stdout);
	while(is_new_dsm_data()==0){
		if(get_state()==EXITING) return 0;
		usleep(50000);
	}

	while(get_state()!=EXITING){
		if(is_new_dsm_data()){	
			printf("\r");// keep printing on same line
			int channels = get_num_dsm_channels();
			// print framerate
			printf("%d/", get_dsm_frame_resolution());
			// print num channels in use
			printf("%d-ch ", channels);
			//print all channels
			for(i=0;i<channels;i++){
				printf("%d:% 0.2f ", i+1, get_dsm_ch_normalized(i+1));
			}
			fflush(stdout);
		}
		else{
			printf("\rSeconds since last DSM packet: ");
			printf("%0.2f ", ms_since_last_dsm_packet()/1000.0);
			printf("                             ");
		}
		fflush(stdout);
		usleep(25000);
	}
	cleanup_cape();
	return 0;
}