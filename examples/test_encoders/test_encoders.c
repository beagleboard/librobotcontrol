// Basic Program to Test Encoders
// Prints out current encoder ticks for a few seconds
// James Strawson - 2013

#include <robotics_cape.h>

int main(){
	initialize_cape();
	
	setGRN(HIGH);
	setRED(HIGH);

	printf("\n\nRaw data for encoders 1,2,3\n");

	while(get_state() != EXITING){
		printf("\r%3d %3d %3d  ", get_encoder_pos(1),get_encoder_pos(2),get_encoder_pos(3));
		fflush(stdout);
		usleep(50000);
	}
	
	cleanup_cape();
	return 0;
}

