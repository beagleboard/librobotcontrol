/******************************************************************************
* rc_test_dsm.c
*
* Prints out the normalized dsm2 values.
* Make sure the transmitter and receiver are paired before testing. 
* Use the rc_pair_dsm example if you didn't already bind receiver to transmitter
* with a bind plug the old fashioned way. The satellite receiver remembers which
* transmitter it is paired to, not your BeagleBone. 
*
* If the values you read are not normalized between +-1, then you should run the
* calibrate_dsm example to save your particular transmitter's min and max 
* channel values.
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

int main(){
	int i;

	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}
	if(rc_initialize_dsm()){
		fprintf(stderr,"ERROR: failed to run rc_initialize_dsm()\n");
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
	while(rc_is_new_dsm_data()==0){
		if(rc_get_state()==EXITING) return 0;
		rc_usleep(50000);
	}

	while(rc_get_state()!=EXITING){
		if(rc_is_new_dsm_data()){
			printf("\r");// keep printing on same line
			int channels = rc_num_dsm_channels();
			// print framerate
			printf("%d/", rc_get_dsm_resolution());
			// print num channels in use
			printf("%d-ch ", channels);
			//print all channels
			for(i=0;i<channels;i++){
				printf("%d:% 0.2f ", i+1, rc_get_dsm_ch_normalized(i+1));
			}
			fflush(stdout);
		}
		else{
			printf("\rSeconds since last DSM packet: ");
			printf("%lld ", rc_nanos_since_last_dsm_packet()/1000000000);
			printf("                             ");
		}
		fflush(stdout);
		rc_usleep(25000);
	}
	rc_cleanup();
	return 0;
}
