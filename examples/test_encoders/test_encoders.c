// Basic Program to Test Encoders
// Prints out current encoder ticks for a few seconds
// James Strawson - 2013

#include <robotics_cape.h>

int main(){
	initialize_cape();
	
	setGRN(HIGH);
	setRED(HIGH);

	printf("\n\nRaw data for encoders 1,2,3,4\n");
	set_encoder_pos(4,0);
	while(get_state() != EXITING){
		printf("\r%3d ", get_encoder_pos(1));
		printf("%3d ", get_encoder_pos(2));
		printf("%3d ", get_encoder_pos(3));
		printf("%3d   ", get_encoder_pos(4));
		fflush(stdout);
		usleep(50000);
	}
	
	cleanup_cape();
	return 0;
}

