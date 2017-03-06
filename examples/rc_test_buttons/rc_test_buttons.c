/*******************************************************************************
* rc_test_buttons.c
*
* This is a more simple example than blink.c for testing button functionality.
* It simply prints to the screen when a button has been pressed or released.
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

// pause button pressed interrupt function
void on_pause_pressed(){
	rc_set_led(RED, ON);
	printf("Pause Pressed\n");
	return;
}

// pause button released interrupt function
void on_pause_released(){
	rc_set_led(RED, OFF);
	printf("Pause Released\n");
	return;
}

// mode button pressed interrupt function
void on_mode_pressed(){
	rc_set_led(GREEN, ON);
	printf("Mode Pressed\n");
	return;
}

// mode button released interrupt function
void on_mode_released(){
	rc_set_led(GREEN,OFF);
	printf("Mode Released\n");
	return;
}

// main just assigns interrupt functions and waits to exit
int main(){
	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}
	//Assign your own functions to be called when events occur
	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_pause_released_func(&on_pause_released);
	rc_set_mode_pressed_func(&on_mode_pressed);
	rc_set_mode_released_func(&on_mode_released);
	printf("Press buttons to see response\n");
	//toggle leds till the program state changes
	while(rc_get_state()!=EXITING){
		rc_usleep(10000);
	}
	rc_cleanup();
	return 0;
}
