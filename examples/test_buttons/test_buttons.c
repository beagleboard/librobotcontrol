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
int on_pause_pressed(){
	set_led(GREEN, ON);
	printf("Pause Pressed\n");
	return 0;
}

// pause button released interrupt function
int on_pause_released(){
	set_led(GREEN, OFF);
	printf("Pause Released\n");
	return 0;
}

// mode button pressed interrupt function
int on_mode_pressed(){
	set_led(RED, ON);
	printf("Mode Pressed\n");
	return 0;
}

// mode button released interrupt function
int on_mode_released(){
	set_led(RED,OFF);
	printf("Mode Released\n");
	return 0;
}

// main just assigns interrupt functions and waits to exit
int main(){
	if(initialize_cape()){
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
	while(get_state()!=EXITING){
		usleep(10000);
	}
	
	cleanup_cape();
	return 0;
}