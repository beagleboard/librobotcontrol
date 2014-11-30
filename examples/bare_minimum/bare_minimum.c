// Bare Minimum Skeleton for Robotics Cape Project
// James Strawson - 2013

#include <robotics_cape.h>

int main(){
	initialize_cape();
	
	printf("\nHello BeagleBone\n");
	
	//Keep Running until program state changes
	while(get_state() != EXITING){
		usleep(1000000);
	}
	
	cleanup_cape();
	return 0;
}