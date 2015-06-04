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
	test_servos.c
	
	demonstrates use of pru to control servos or ESCs with pulse widths
	note that this library purposefully sends only a single pulse for
	each call to send_servo_pulse_us(). This allows the user to send
	pulses immediately after calculating a control input and at any
	arbitrary frequency to suit specific hardware. For example a user can
	send a 50hz signal to control a servo on one channel and a 200hz signal
	on another channel for controlling high-performance brushless ESCs
*/

#include <robotics_cape.h>

#define FORWARD 1
#define REVERSE -1
// only test over half the range to prevent stalling
#define MIN_US 1500-300
#define MAX_US 1500+300

int main(int argc, char *argv[]){
    initialize_cape();
    
	int i;
	int micros = MIN_US;
	int direction = FORWARD;
	int ch = 0;
	int all = 0;

	// first parse the command line arguments to determine if one servo was selected
	if (argc==1){
		all = 1; // no argument given, enable commands to all servos
		printf("\nSending signal to all servos\n");
    }
	else{
		ch = atoi(argv[1]);
		if(ch>SERVO_CHANNELS || ch<1){
			printf("choose a channel between 1 and %d\n", SERVO_CHANNELS);
			return -1;
		}
		all = 0;
		printf("\nSending signal only to servo %d\n", ch);
	}
	
	printf("Pulse width in microseconds:\n");
	
	// now loop back and forth continuously
	while(get_state()!=EXITING){
	
		// increase or decrease # of microseconds each loop 
		micros += direction * 10;
		
		// reset pulse width at end of sweep
		if(micros>MAX_US){
			micros = MAX_US;
			direction = REVERSE;
		}
		else if(micros<MIN_US){
			micros = MIN_US;
			direction = FORWARD;
		}
		

		
		// send single pulse to each servo
		if(all){
			for(i=0; i<SERVO_CHANNELS; i++){
				send_servo_pulse_us(i+1,micros);
			}
		}
		// or just send one pulse to one servo
		else send_servo_pulse_us(ch,micros);
		
		// tell the user the current microsecond pulse width
		printf("\r%d", micros);
		fflush(stdout); // flush the output
		
		// Send pulses at roughly 50hz
		usleep(20000); 
	}
    
	cleanup_cape();
    return 0;
}

