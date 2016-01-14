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

// pause button pressed interrupt function
int on_pause_pressed(){
	printf("Pause Pressed\n");
	return 0;
}

// pause button released interrupt function
int on_pause_released(){
	printf("Pause Released\n");
	return 0;
}

// mode button pressed interrupt function
int on_mode_pressed(){
	printf("Mode Pressed\n");
	return 0;
}

// mode button released interrupt function
int on_mode_released(){
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
	set_pause_unpressed_func(&on_pause_released);
	set_mode_pressed_func(&on_mode_pressed);
	set_mode_unpressed_func(&on_mode_released);
	
	//toggle leds till the program state changes
	while(get_state()!=EXITING){
		usleep(10000);
	}
	
	cleanup_cape();
	return 0;
}