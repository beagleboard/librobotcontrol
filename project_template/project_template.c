/*******************************************************************************
* project_template.c
*
* This is meant to be a skeleton program for robotics cape projects. 
* Change this description and file name 
*******************************************************************************/

#include <roboticscape-usefulincludes.h>
#include <roboticscape.h>


// function declarations
int on_pause_pressed();
int on_pause_released();


/*******************************************************************************
* int main() 
*	
* This template main function contains these critical components
* - call to initialize_cape
* - main while loop that checks for EXITING condition
* - cleanup_cape() at the end
*******************************************************************************/
int main(){
	// always initialize cape library first
	initialize_cape();

	// do your own initialization here
	printf("\nHello BeagleBone\n");
	set_pause_pressed_func(&on_pause_pressed);
	set_pause_released_func(&on_pause_released);

	// done initializing so set state to RUNNING
	set_state(RUNNING); 

	// Keep looping until state changes to EXITING
	while(get_state()!=EXITING){
		// handle other states
		if(get_state()==RUNNING){
			// do things
			set_led(GREEN, ON);
			set_led(RED, OFF);
		}
		else if(get_state()==PAUSED){
			// do other things
			set_led(GREEN, OFF);
			set_led(RED, ON);
		}
		// always sleep at some point
		usleep(100000);
	}
	
	// exit cleanly
	cleanup_cape(); 
	return 0;
}


/*******************************************************************************
* int on_pause_released() 
*	
* Make the Pause button toggle between paused and running states.
*******************************************************************************/
int on_pause_released(){
	// toggle betewen paused and running modes
	if(get_state()==RUNNING)   		set_state(PAUSED);
	else if(get_state()==PAUSED)	set_state(RUNNING);
	return 0;
}

/*******************************************************************************
* int on_pause_pressed() 
*	
* If the user holds the pause button for 2 seconds, set state to exiting which 
* triggers the rest of the program to exit cleanly.
*******************************************************************************/
int on_pause_pressed(){
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds
	
	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		usleep(us_wait/samples);
		if(get_pause_button() == RELEASED) return 0;
	}
	printf("long press detected, shutting down\n");
	set_state(EXITING);
	return 0;
}