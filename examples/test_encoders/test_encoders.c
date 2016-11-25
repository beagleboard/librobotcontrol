/*******************************************************************************
* test_encoders.c
*
* James Strawson 2016
* Prints out current encoder ticks for all 4 channels
* channels 1-3 are counted using eQEP 0-2. Channel 4 is counted by PRU0
*******************************************************************************/

#include "../../libraries/roboticscape-usefulincludes.h"
#include "../../libraries/roboticscape.h"

int main(){
	int i;

	if(initialize_roboticscape()<0){
		printf("ERROR: failed to initialize cape");
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
			printf("%6d  |", get_encoder_pos(i));
		}			
		fflush(stdout);
		usleep(50000);
	}
	
	cleanup_roboticscape();
	return 0;
}

