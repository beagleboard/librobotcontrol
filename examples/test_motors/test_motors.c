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

/*
	test_motors.c
	
	demonstrates use of H-bridges to drive motors. This program takes in
	1 or two arguments.
	
	The first argument is the duty cycle normalized from -1 to 1;
	
	The seconds argument is optional and is the channel between 1 and 4.
	If the second argument isn't given then all motors are driven.
*/

#include <robotics_cape.h>

int main(int argc, char *argv[]){
	float duty;
	int ch, all;
	
	initialize_cape();
		
	// first parse the command line arguments to determine what position to go to
	if (argc==1){
		printf("\nPlease give a normalized duty cycle from -1 to 1\n");
		return -1;
    }
	if(argc>3){
		printf("too many input arguments\n");
		return -1;
	}

	duty = atof(argv[1]);
	printf("\n");
	
	if(duty<=1 && duty >=-1){
		printf("using normalized duty_cycle: %0.4f\n", duty);
	}
	else{
		printf("invalid input\n");
		return -1;
	}
	
	// now check if a channel was also given as an argument
	if(argc==3){
		ch = atoi(argv[2]);
		if(ch>MOTOR_CHANNELS || ch<1){
			printf("choose a channel between 1 and %d\n", MOTOR_CHANNELS);
			return -1;
		}
		all = 0;
		printf("Sending signal only to motor %d\n", ch);
	}
	else{
		all=1;
		printf("Sending signal to all channels\n");
	}
	

	enable_motors(); // bring H-bridges of of standby
	set_led(GREEN,HIGH);
	set_led(RED,HIGH);
	
	// set one or all motors
	if(all) set_motor_all(duty);
	else set_motor(ch,duty);
	
	// chill till the user exits
	while(get_state() != EXITING){
		usleep(500000);
	}
	
	// User must have existed, put H-bridges into standby
	// not entirely necessary since clenup_cape does this too
	kill_pwm();
	disable_motors();	
	printf("All Motors Off\n\n");
	
	cleanup_cape();
	return 0;
}

