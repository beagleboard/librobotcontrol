/*******************************************************************************
* test_encoders.c
*
* Prints out current encoder ticks for all 4 channels
* channels 1-3 are counted using eQEP 0-2. Channel 4 is counted by PRU0
*******************************************************************************/

#include <robotics_cape.h>
#include <useful_includes.h>

int main(){
	initialize_cape();
	
	set_led(GREEN,ON);
	set_led(RED,ON);

	printf("\n   Raw encoder positions\n");
	printf("   ch1   ch2   ch3   ch4\n");

	while(get_state() != EXITING){
		printf("\r%5d %5d %5d %5d       ", \
						get_encoder_pos(1),\
						get_encoder_pos(2),\
						get_encoder_pos(3),\
						get_encoder_pos(4));
		fflush(stdout);
		usleep(50000);
	}
	
	cleanup_cape();
	return 0;
}

