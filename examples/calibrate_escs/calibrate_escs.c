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

#include <robotics_cape.h>

int sending;
void *send_pulses(void *params){
	int i;
	while(sending && (get_state()!=EXITING)){
		for(i=0; i<SERVO_CHANNELS; i++){
			send_servo_pulse_us(i+1, SERVO_MAX_US);
		}
		usleep(20000);
	}
	return 0;
}

int main(){
	int i, j;
	
	initialize_cape();
	
	printf("\nDISCONNECT PROPELLERS FROM MOTORS\n");
	printf("DISCONNECT POWER FROM ESCS\n");
	printf("press enter to continue\n");
	
	// wait for the user to power off escs
	while(getchar() != '\n');
	
	
	printf("\n");
	printf("Now reapply power to the ESCs.\n");
	printf("Press enter again after the ESCs chirp\n");
	setGRN(HIGH);
	fflush(stdin);
	
	//Send full throttle until the user hits enter
	sending = 1;
	pthread_t  send_pulse_thread;
	pthread_create(&send_pulse_thread, NULL, send_pulses, (void*) NULL);
	while( getchar() != '\n' );
	sending = 0; //stop sending thread
	setGRN(LOW);
	usleep(20000);
	
	//set throttle to 0 for a second to define lower bound
	for(j=1; j<=50; j++){
		for(i=0; i<SERVO_CHANNELS; i++){
			send_servo_pulse_us(i+1, SERVO_MIN_US);
		}
		usleep(20000);
	}
	
	printf("\nCalibration complete, check with test_servos\n");

	cleanup_cape();
	return 0;
}