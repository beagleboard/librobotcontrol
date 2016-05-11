/*******************************************************************************
* blink.c
*
* See README.TXT for more details
*******************************************************************************/


#include <robotics_cape.h>
#include <useful_includes.h>

#define QUIT_TIMEOUT_US 1500000 // quit after 1.5 seconds holding pause button

// mode=0,1,2 corresponds to us_delay index for slow,medium,fast
const int us_delay[] = {400000, 200000, 100000};	
int mode;
int toggle; // toggles between 0&1 for led blink


// Function to be called when pause button is pressed
// this is called on press instead of release so it can time
// how long the user has held the button and exit after 1.5 seconds.
int on_pause_press(){
	printf("user pressed Pause\n");
	// toggle between PAUSED and RUNNING 
	switch(get_state()){
	case PAUSED:
		set_state(RUNNING); // toggle running/paused
		break;
	case RUNNING:
		set_state(PAUSED);	// toggle running/paused
		break;
	case EXITING: // be careful to exit if program is already shutting down!!!
		return 0; 
	default: // ignore other states
		break;
	}
	
	// now wait to see if the user want to shut down the program
	int i=0;
	while(i<QUIT_TIMEOUT_US){
		usleep(1000);
		if(get_pause_button() == RELEASED){
			printf("user released pause button\n");
			return 0; //user let go before time-out
		}
		i+=1000;
	}
	printf("long press detected, shutting down\n");
	blink_led(RED, 15, 0.5); // blink 5hz for 1 second
	//user held the button down long enough, blink and exit cleanly
	set_state(EXITING);
	return 0;
}

// increment mode and therefore speed
int on_mode_release(){
	if(mode<2)mode++;
	else mode=0;
	
	printf("setting mode: %d\n", mode);
	return 0;
}

// main function usually sits in one while loop blinking LEDs depending
int main(){
	if(initialize_cape()){
		printf("failed to initialize cape\n");
		return -1;
	}
	printf("calling blink_led\n");
	blink_led(GREEN, 15, 0.5); // blink 15hz for 0.5 second
	
	printf("\nPress mode to change blink rate\n");
	printf("hold pause button to exit\n");
	
	//Assign your own functions to be called when events occur
	set_pause_pressed_func(&on_pause_press);
	set_mode_released_func(&on_mode_release);
	
	// start in slow mode
	mode = 0;
	set_state(RUNNING);
	
	// Run the main loop untill state is EXITING which is set by hitting ctrl-c
	// or holding down the pause button for more than the quit timeout period
	while(get_state()!=EXITING){
		// if the state is RUNNING (instead of PAUSED) then blink!
		if(get_state()==RUNNING){
			if(toggle){
				set_led(GREEN,OFF);
				set_led(RED,ON);
				toggle = 0;
			}
			else{
				set_led(GREEN,ON);
				set_led(RED,OFF);
				toggle=1;
			}
		}
		// sleep the right delay based on current mode.
		usleep(us_delay[mode]);
	}
	
	// now that the while loop has exited, clean up neatly and exit compeltely.
	cleanup_cape();
	return 0;
}