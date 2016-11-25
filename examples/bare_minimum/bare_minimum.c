/*******************************************************************************
* bare_minimum.c
*
* James Strawson 2016
* This is meant to be a skeleton program for robotics cape projects. 
*******************************************************************************/

#include "../../libraries/roboticscape-usefulincludes.h"
#include "../../libraries/roboticscape.h"

int main(){
	// always initialize cape library first
	initialize_roboticscape();
	printf("\nHello BeagleBone\n");
	rc_set_state(RUNNING);
	// Keep Running until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// handle other states
		if(rc_get_state()==RUNNING){
			// do things
		}
		else if(rc_get_state()==PAUSED){
			// do other things
		}
		// always sleep at some point
		usleep(100000);
	}
	
	cleanup_roboticscape(); // exit cleanly
	return 0;
}
