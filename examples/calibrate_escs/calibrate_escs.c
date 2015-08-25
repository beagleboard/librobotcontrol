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
float width;

void *send_pulses(void *params){
	while(sending && (get_state()!=EXITING)){
		send_esc_pulse_normalized_all(width);
		usleep(20000);
	}
	return 0;
}

int main(){
	initialize_cape();
	
	printf("\nDISCONNECT PROPELLERS FROM MOTORS\n");
	printf("DISCONNECT POWER FROM ESCS\n");
	printf("press enter to start sending max pulse width\n");
	while(getchar() != '\n');
	
	
	//Send full throttle until the user hits enter
	sending = 1;
	width = 1;
	pthread_t  send_pulse_thread;
	pthread_create(&send_pulse_thread, NULL, send_pulses, (void*) NULL);
	
	printf("\n");
	printf("Now reapply power to the ESCs.\n");
	printf("Press enter again after the ESCs finish chirping\n");
	setGRN(HIGH);
	fflush(stdin);
	while( getchar() != '\n' );
	
	printf("\n");
	printf("Sending minimum width pulses\n");
	printf("Press enter again after the ESCs chirping to finish calibration\n");
	width = 0;
	while( getchar() != '\n' );

	// cleanup and close
	printf("\nCalibration complete, check with test_servos\n");
	sending = 0;
	pthread_join(send_pulse_thread, NULL);
	
	cleanup_cape();
	return 0;
}