/*
Copyright (c) 2015, James Strawson
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
	test_servos.c
	
	demonstrates use of pru to control servos or ESCs with pulse widths.
	This program takes in 1 or 2 arguments. 
	
	The first argument is the pulse width in either microseconds or a normalized
	value from 0 to 1. the normalized value will scale to microseconds between 
	900 and 2100 microseconds which is common for servos.
	
	The seconds argument is optional and is the channel between 1 and 8.
	If the second argument isn't given then all servos are driven.
*/

#include <robotics_cape.h>

typedef enum servo_mode_t{
	NORMALIZED,
	MICROS
}servo_mode_t;

int main(int argc, char *argv[]){
	initialize_cape();
	
	servo_mode_t mode;
	float input;
	int micros, ch, all;
	
	// first parse the command line arguments to determine what position to go to
	if (argc==1){
		printf("\nPlease give a normalized value or pulse width in microseconds\n");
		return -1;
    }
	if(argc>3){
		printf("too many input arguments\n");
		return -1;
	}

	input = atof(argv[1]);
	micros = lround(input);
	printf("\n");
	
	if(input<=1 && input >=0){
		printf("using normalized pulse width: %0.4f\n", input);
		mode = NORMALIZED;
	}
	else if(micros>= SERVO_MIN_US && micros<=SERVO_MAX_US){
		printf("using pulse width in microseconds: %d\n", micros);
		mode = MICROS;
	}
	else{
		printf("invalid input\n");
		return -1;
	}

	// now check if a channel was also given as an argument
	if(argc==3){
		ch = atoi(argv[2]);
		if(ch>SERVO_CHANNELS || ch<1){
			printf("choose a channel between 1 and %d\n", SERVO_CHANNELS);
			return -1;
		}
		all = 0;
		printf("Sending signal only to servo %d\n", ch);
	}
	else{
		all=1;
		printf("Sending signal to all channels\n");
	}

	
	// if we got here, ready to send signal
	while(get_state()!=EXITING){
		if(mode==NORMALIZED && all==0)
			send_servo_pulse_normalized(ch,input);
		else if(mode==NORMALIZED && all==1)
			send_servo_pulse_normalized_all(input);
		else if(mode==MICROS && all==0)
			send_servo_pulse_us(ch,micros);
		else if(mode==MICROS && all==1)
			send_servo_pulse_us_all(micros);
		else{
			printf("logic error\n");
			return -1;
		}
	usleep(20000);
	}
	
	cleanup_cape();
    return 0;
	
}
	
