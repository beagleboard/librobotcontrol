/*******************************************************************************
* rc_bare_minimum.c
*
* This is meant to be a skeleton program for robotics cape projects. Critically,
* it demonstrates correct handling of the rc flow state
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

int main(){
	// always initialize cape library first
	rc_initialize();
	printf("\nHello BeagleBone!\n");
	printf("\nPress Ctrl-C to exit\n");
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
		rc_usleep(100000);
	}
	// exit cleanly
	rc_cleanup();
	return 0;
}
