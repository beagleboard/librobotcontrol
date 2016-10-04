/*******************************************************************************
* bare_minimum.c
*
* James Strawson 2016
* This is meant to be a skeleton program for robotics cape projects. 
*******************************************************************************/

#include <robotics_cape.h>
#include <useful_includes.h>

int main(){
	// always initialize cape library first
	initialize_cape();
	printf("\nHello BeagleBone\n");
	set_state(RUNNING);
	// Keep Running until state changes to EXITING
	while(get_state()!=EXITING){
		// handle other states
		if(get_state()==RUNNING){
			// do things
		}
		else if(get_state()==PAUSED){
			// do other things
		}
		// always sleep at some point
		usleep(100000);
	}
	
	cleanup_cape(); // exit cleanly
	return 0;
}
