/*
Copyright (c) 2014, James Strawson
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*/

// Button and LED tester for the Robotics Cape
// Pressing either button makes an LED blink
// Hold the pause button or ctrl-c to exit cleanly
// James Strawson - 2014

#include <robotics_cape.h>

int mode; 	// 0, 1, 2  slow medium fast blink rate
// sleep between toggling LEDs in 3 modes
const int us_delay[] = {400000, 200000, 100000};	
int toggle; // toggles between 0&1 for led blink

// Print to the console when buttons are pressed
int on_pause_press(){
	printf("user pressed Pause\n");
	// toggle between PAUSED and RUNNING
	switch(get_state()){
	case PAUSED:
		set_state(RUNNING);
		break;
	case RUNNING:
		set_state(PAUSED);
		break;
	case EXITING: 
		// careful to exit if program is already shutting down
		return 0; 
	default: // ignore other states
		break;
	}
	
	// now wait to see if the user want to shut down the program
	int i=0;
	const int us_wait = 1500000; //1.5 seconds
	while(i<100){
		usleep(us_wait/100);
		if(get_pause_button_state() == UNPRESSED){
			printf("user let go\n");
			return 0; //user let go before time-out
		}
		i++;
	}
	printf("long press detected, shutting down\n");
	//user held the button down long enough, blink and exit cleanly
	set_state(EXITING);
	return 0;
}

// increment mode
int on_mode_unpress(){
	if(mode<2)mode++;
	else mode=0;
	
	printf("setting mode: %d\n", mode);
	return 0;
}

int main(){
	initialize_cape();
	
	printf("\nPress mode to change blink rate\n");
	printf("hold pause button to exit\n");
	
	//Assign your own functions to be called when events occur
	set_pause_pressed_func(&on_pause_press);
	set_mode_unpressed_func(&on_mode_unpress);
	
	// start in slow mode
	mode = 0;
	set_state(RUNNING);
	
	//toggle leds till the program state changes
	while(get_state()!=EXITING){
		if(get_state()==RUNNING){
			if(toggle){
				setGRN(LOW);
				setRED(HIGH);
				toggle = 0;
			}
			else{
				setGRN(HIGH);
				setRED(LOW);
				toggle=1;
			}
		}
		usleep(us_delay[mode]);
	}
	
	cleanup_cape();
	return 0;
}