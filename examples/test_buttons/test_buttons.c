/*******************************************************************************
* test_buttons.c
*
* James Strawson 2016
* This is a more simple example than blink.c for testing button functionality.
* It simply prints to the screen when a button has been pressed or released.
*******************************************************************************/

#include "../../libraries/roboticscape-usefulincludes.h"
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
	if(initialize_roboticscape()){
		printf("failed to initialize cape\n");
		return -1;
	}
	
	//Assign your own functions to be called when events occur
	set_pause_pressed_func(&on_pause_pressed);
	set_pause_released_func(&on_pause_released);
	set_mode_pressed_func(&on_mode_pressed);
	set_mode_released_func(&on_mode_released);
	
	printf("Press buttons to see response\n");
	
	//toggle leds till the program state changes
	while(rc_get_state()!=EXITING){
		usleep(10000);
	}
	
	cleanup_roboticscape();
	return 0;
}