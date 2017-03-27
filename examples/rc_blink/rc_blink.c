/*******************************************************************************
* rc_blink.c
*
* This is an example program to demonstrate use of LEDs and button handlers
* in the robotics cape API. Once started, blink will flash the green and red
* LEDs. Pressing the mode button will cycle through 3 blinking speeds, slow
* medium, and fast. Momentarily pressing the pause button will stop and start
* the blinking by toggling the global state between PAUSED and RUNNING. If the 
* user holds the pause button for more than 1.5 seconds then the blink program
* will flash the red LED and exit cleanly.
*
* This should be used as a reference for how to handle buttons and how to
* control program flow cleanly utilizing rc_get_state() and rc_set_state().
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

#define QUIT_TIMEOUT_US 1500000 // quit after 1.5 seconds holding pause button

// mode=0,1,2 corresponds to us_delay index for slow,medium,fast
const int us_delay[] = {400000, 170000, 100000};	
int mode;
int toggle; // toggles between 0&1 for led blink


/*******************************************************************************
* void on_pause_released() 
*	
* Make the Pause button toggle between paused and running states.
*******************************************************************************/
void on_pause_released(){
	// toggle betewen paused and running modes
	if(rc_get_state()==RUNNING)		rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return;
}

/*******************************************************************************
* void on_pause_pressed() 
*	
* If the user holds the pause button for 2 seconds, set state to exiting which 
* triggers the rest of the program to exit cleanly.
*******************************************************************************/
void on_pause_pressed(){
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds
	
	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_get_pause_button() == RELEASED) return;
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}


/*******************************************************************************
* void on_mode_released() 
*	
* If the user holds the pause button for 2 seconds, set state to exiting which 
* triggers the rest of the program to exit cleanly.
*******************************************************************************/
void on_mode_released(){
	if(mode<2)mode++;
	else mode=0;
	
	printf("setting mode: %d\n", mode);
	return;
}

/*******************************************************************************
* int main()
*
* main function sits in one while loop blinking LEDs while button handlers
* control the blink speed and program state
*******************************************************************************/
int main(){
	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}
	
	printf("\nPress mode to change blink rate\n");
	printf("hold pause button to exit\n");
	
	//Assign your own functions to be called when events occur
	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_pause_released_func(&on_pause_released);
	rc_set_mode_released_func(&on_mode_released);
	
	// start in slow mode
	mode = 0;
	rc_set_state(RUNNING);
	
	// Run the main loop untill state is EXITING which is set by hitting ctrl-c
	// or holding down the pause button for more than the quit timeout period
	while(rc_get_state()!=EXITING){
		// if the state is RUNNING (instead of PAUSED) then blink!
		if(rc_get_state()==RUNNING){
			if(toggle){
				rc_set_led(GREEN,OFF);
				rc_set_led(RED,ON);
				toggle = 0;
			}
			else{
				rc_set_led(GREEN,ON);
				rc_set_led(RED,OFF);
				toggle=1;
			}
		}
		// sleep the right delay based on current mode.
		rc_usleep(us_delay[mode]);
	}
	
	// now that the while loop has exited, clean up neatly and exit compeltely.
	rc_cleanup();
	return 0;
}
