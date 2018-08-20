/**
 * @example    rc_dsm_passthrough.c
 *
 * This sends all dsm2 data straight out the servo channels
 * as they come in. When running this program the BBB acts exactly like a normal
 * DSM receiver.
 *
 * You must specify SERVO or ESC mode with -s or -e to turn om or off the 6V
 * power rail. Sending 6V into an ESC may damage it!!!
 *
 * Raw data is also printed to the terminal for monitoring.
 */

#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <rc/dsm.h>
#include <rc/servo.h>
#include <rc/time.h>
#include <rc/adc.h>

static int running;

typedef enum p_mode_t{
	NONE,
	POWERON,
	POWEROFF
} p_mode_t;

// function to be called every time new a new DSM packet is received.
static void __send_pulses(void)
{
	int i, ch, val;

	// send single to working channels
	for(i=1; i<=8; i++){
		val=rc_dsm_ch_raw(i);
		if(val>0) rc_servo_send_pulse_us(i,val);
	}

	// print all channels
	printf("\r");
	ch = rc_dsm_channels();
	for(i=1;i<=ch;i++){
		printf("% 4d   ", rc_dsm_ch_raw(i));
	}
	fflush(stdout);
	return;
}

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}


// printed if some invalid argument was given
static void __print_usage(void)
{
	printf("\n");
	printf(" Options\n");
	printf(" -s   Enable 6V power rail for servos.\n");
	printf(" -e   Disable 6V power rail for ESCs.\n");
	printf(" -h   Print this messege.\n\n");
	return;
}

// main routine
int main(int argc, char *argv[])
{
	int c;
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
			__print_usage();
			return -1;
			break;

		default:
			fprintf(stderr,"Invalid Argument \n");
			__print_usage();
			return -1;
		}
	}

	if(mode == NONE){
		fprintf(stderr,"You must select a power mode -s or -e\n");
		__print_usage();
		return -1;
	}

	if(rc_dsm_init()==-1) return -1;

	// if power has been requested, make sure battery is connected!
	if(mode == POWERON){
		// read adc to make sure battery is connected
		if(rc_adc_init()){
			fprintf(stderr,"ERROR: failed to run rc_adc_init()\n");
			return -1;
		}
		if(rc_adc_batt()<6.0){
			fprintf(stderr,"ERROR: battery disconnected or insufficiently charged to drive motors\n");
			return -1;
		}
		rc_adc_cleanup();
		if(rc_servo_power_rail_en(1)){
			fprintf(stderr,"failed to enable power rail\n");
			return -1;
		}
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

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running=1;

	rc_dsm_set_callback(&__send_pulses);
	while(running){
		if(rc_dsm_is_connection_active()==0){
			printf("\rSeconds since last DSM packet: ");
			printf("%0.1f ", rc_dsm_nanos_since_last_packet()/1000000000.0);
			printf("                             ");
			fflush(stdout);
		}
		rc_usleep(25000);
	}
	printf("\n");
	rc_dsm_cleanup();
	return 0;
}