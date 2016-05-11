/*******************************************************************************
* calibrate_escs.c
*
* see README.TXT for details.
*******************************************************************************/

#include <useful_includes.h>
#include <robotics_cape.h>

float width; // global variable for normalized pulse width to send

// background thread to send pulses at 50hz to ESCs
void *send_pulses(void *params){
	while(get_state()!=EXITING){
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
	width = 1;
	pthread_t  send_pulse_thread;
	pthread_create(&send_pulse_thread, NULL, send_pulses, (void*) NULL);
	
	printf("\n");
	printf("Now reapply power to the ESCs.\n");
	printf("Press enter again after the ESCs finish chirping\n");
	set_led(GREEN,1);
	fflush(stdin);
	while( getchar() != '\n' );
	
	// now set lower bound
	printf("\n");
	printf("Sending minimum width pulses\n");
	printf("Press enter again after the ESCs chirping to finish calibration\n");
	width = 0;
	while( getchar() != '\n' );

	// cleanup and close
	printf("\nCalibration complete, check with test_servos\n");
	set_state(EXITING); // this tells the send_pulses thread to stop
	pthread_join(send_pulse_thread, NULL); // wait for it to stop
	
	cleanup_cape();
	return 0;
}