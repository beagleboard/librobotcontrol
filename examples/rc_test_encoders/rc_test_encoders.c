/*******************************************************************************
* rc_test_encoders.c
*
* Prints out current encoder ticks for all 4 channels
* channels 1-3 are counted using eQEP 0-2. Channel 4 is counted by PRU0
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

	printf("\nRaw encoder positions\n");
	printf("   E1   |");
	printf("   E2   |");
	printf("   E3   |");
	printf("   E4   |");
	printf(" \n");

	while(rc_get_state() != EXITING){
		printf("\r");
		for(i=1;i<=4;i++){
			printf("%6d  |", rc_get_encoder_pos(i));
		}
		fflush(stdout);
		rc_usleep(50000);
	}

	rc_cleanup();
	return 0;
}

