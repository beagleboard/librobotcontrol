// Basic Program to Test Motors
// Moves all 6 motors forward and back
// James Strawson - 2013

#include <robotics_cape.h>

int main(){
	initialize_cape();
	
	setGRN(HIGH);
	setRED(HIGH);
	int i;
	for(i=1;i<=6;i++){
		set_motor(i,.3);
	}
	printf("\nAll Motors Forward\n");
	sleep(2);

	for(i=1;i<=6;i++){
		set_motor(i,-.3);
	}
	printf("All Motors Reverse\n");
	sleep(2);
	
	for(i=1;i<=6;i++){
		set_motor(i,0);
	}
	printf("All Motors Off\n\n");
	
	cleanup_cape();
	return 0;
}

