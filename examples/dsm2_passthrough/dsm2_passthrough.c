/*******************************************************************************
* dsm2_passthrough.c
*
* This sends all dsm2 data straight out the servo channels as they come in.
* When running this program the BBB acts exactly like a normal DSM2 receiver.
*
* You must specify SERVO or ESC mode with -s or -e to turn off or off the 6V
* power rail. Sending 6V into an ESC may damage it!!!
*
* Raw data is also printed to the terminal for monitoring.
*******************************************************************************/

#include <useful_includes.h>
#include <robotics_cape.h>

typedef enum p_mode_t{
	NONE,
	POWERON,
	POWEROFF
} p_mode_t;

// function to be called every time new a new DSM2 packet is received.
int send_pulses(){
	int i, ch;
	// send single pulse to each servo
	for(i=0; i<8; i++){
		send_servo_pulse_us(i+1,get_dsm2_ch_raw(i+1));
	}

	//print all channels
	printf("\r");
	ch =  get_num_dsm2_channels();
	for(i=0;i<ch;i++){
		printf("% 4d   ", get_dsm2_ch_raw(i+1));
	}
	fflush(stdout);	
	return 0;
}


// printed if some invalid argument was given
void print_usage(){
	printf("\n");
	printf(" Options\n");
	printf(" -s   Enable 6V power rail for servos.\n");
	printf(" -e   Disable 6V power rail for ESCs.\n");
	printf(" -h   Print this messege.\n\n");
}
			
			

int main(int argc, char *argv[]){
	int ms,c;
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
			printf("\nInvalid Argument \n");
			print_usage();
			return -1;
		}
	}
			
	if(mode == NONE){
		printf("You must select a power mode -s or -e\n");
		print_usage();
		return -1;
	}
	
	if(initialize_cape()){
		printf("ERROR: failed to initialize_cape\n");
		return -1;
	}
	
	if(initialize_dsm2()){
		printf("ERROR: failed to initialize_dsm2\n");
		return -1;
	}
	
	if(mode == POWERON){
		enable_servo_power_rail();
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
	
	printf("Waiting for DSM2 Connection");
	fflush(stdout);
	
	set_new_dsm2_data_func(&send_pulses);
	while(get_state()!=EXITING){
		ms = ms_since_last_dsm2_packet();
		if(ms>500){	
			printf("\rMilliseconds since last packet: %d         ", ms);
			fflush(stdout);
		}
		usleep(20000);
	}
	printf("\n");
	cleanup_cape();
	return 0;
}