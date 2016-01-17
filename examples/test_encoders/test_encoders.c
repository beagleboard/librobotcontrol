// Basic Program to Test Encoders
// Prints out current encoder ticks for all 4 channels
// James Strawson - 2013

#include <robotics_cape.h>

int main(){
	initialize_cape();
	
	set_led(GREEN,HIGH);
	set_led(RED,HIGH);

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

