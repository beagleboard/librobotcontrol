/*******************************************************************************
* dsm2_passthrough.c
*
* James Strawson 2016
* This sends all dsm2 data straight out the servo channels as they come in.
* When running this program the BBB acts exactly like a normal DSM2 receiver.
*
* You must specify SERVO or ESC mode with -s or -e to turn om or off the 6V
* power rail. Sending 6V into an ESC may damage it!!!
*
* Raw data is also printed to the terminal for monitoring.
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

typedef enum p_mode_t{
	NONE,
	POWERON,
	POWEROFF
} p_mode_t;

// function to be called every time new a new DSM2 packet is received.
void send_pulses(){
	int i, ch;
	// send single pulse to each servo
	for(i=0; i<8; i++){
		rc_send_servo_pulse_us(i+1,rc_get_dsm_ch_raw(i+1));
	}

	//print all channels
	printf("\r");
	ch =  rc_num_dsm_channels();
	for(i=0;i<ch;i++){
		printf("% 4d   ", rc_get_dsm_ch_raw(i+1));
	}
	fflush(stdout);	
	return;
}


// printed if some invalid argument was given
void print_usage(){
	printf("\n");
	printf(" Options\n");
	printf(" -s   Enable 6V power rail for servos.\n");
	printf(" -e   Disable 6V power rail for ESCs.\n");
	printf(" -h   Print this messege.\n\n");
}

// main routine
int main(int argc, char *argv[]){
	int c;
	uint64_t ns;
	p_mode_t mode = NONE;
	
	// parse arguments
	opterr = 0;
	while ((c = getopt(argc, argv, "seh")) != -1){
		switch (c){
			
		case 's': // enable power for servos
			mode = POWERON;
			break;
			
		case 'e': // disbable power for ESCs
			mode = POWEROFF;
			break;
			
		case 'h': // show help messege
			print_usage();
			return -1;
			break;
			
		default:
			fprintf(stderr,"Invalid Argument \n");
			print_usage();
			return -1;
		}
	}

	if(mode == NONE){
		fprintf(stderr,"You must select a power mode -s or -e\n");
		print_usage();
		return -1;
	}

	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}

	if(rc_initialize_dsm()){
		fprintf(stderr,"ERROR: failed to rc_initialize_dsm2\n");
		return -1;
	}

	if(mode == POWERON){
		rc_enable_servo_power_rail();
	}

	// print header
	printf("1:Thr   ");
	printf("2:Roll  ");
	printf("3:Pitch ");
	printf("4:Yaw   ");
	printf("5:Kill  ");
	printf("6:Mode  ");
	printf("7:Aux1  ");
	printf("8:Aux2  ");
	printf("\n");
	
	printf("Waiting for DSM Connection");
	fflush(stdout);
	
	rc_set_dsm_data_func(&send_pulses);
	while(rc_get_state()!=EXITING){
		ns = rc_nanos_since_last_dsm_packet();
		if(ns>500000000){	
			printf("\rMilliseconds since last packet: %lld         ",ns/1000000);
			fflush(stdout);
		}
		rc_usleep(20000);
	}
	printf("\n");
	rc_cleanup();
	return 0;
}
