/**
 * @file rc_test_servos.c
 * @example    rc_test_servos
 *
 *
 * Demonstrates use of pru to control servos. This program
 * operates in 4 different modes. See the option list below for how to select an
 * operational mode from the command line. In all modes the power rail is
 * enabled to power the servos and the program will refuse to start unless a
 * charged 2-cell LiPo is plugged into the while balance jack and detected by
 * the ADC.
 *
 * NORM: uses rc_servo_send_pulse_normalized() to set one or all servo
 * positions to a value from -1.5 to 1.5 corresponding to their extended range.
 * -1 to 1 is considered the "safe" normal range as some servos will not go
 * beyond this. Test your servos incrementally to find their safe range.
 *
 * MICROSECONDS: You can also specify your own pulse width in microseconds (us).
 * This uses the rc_servo_send_pulse_us() function.
 *
 * SWEEP: This is intended to gently sweep a servo back and forth about the
 * center position. Specify a range limit as a command line argument as
 * described below. This also uses the rc_servo_send_pulse_normalized()
 * function.
 *
 * RADIO: Uses a single user-selected channel of a DSM radio to control one
 * or all of the servo channels. This is really just for testing, use the
 * rc_dsm_passthrough example for controlling multiple servos, or better yet
 * write an application-specific program for your robot. After all, this program
 * is just for testing!!!!
 *
 * @author     James Strawson
 * @date       3/20/2018
 */

#include <stdio.h>
#include <getopt.h>
#include <stdlib.h> // for atoi
#include <signal.h>
#include <rc/time.h>
#include <rc/adc.h>
#include <rc/dsm.h>
#include <rc/servo.h>

static int running;

typedef enum test_mode_t{
	DISABLED,
	NORM,
	MICROSECONDS,
	SWEEP,
	RADIO
}test_mode_t;


static void __print_usage(void)
{
	printf("\n");
	printf(" Options\n");
	printf(" -c {channel}   Specify one channel from 1-8.\n");
	printf("                Otherwise all channels will be driven equally\n");
	printf(" -f {hz}        Specify pulse frequency, otherwise 50hz is used\n");
	printf(" -p {position}  Drive servo to a position between -1.5 & 1.5\n");
	printf(" -w {width_us}  Send pulse width in microseconds (us)\n");
	printf(" -s {limit}     Sweep servo back/forth between +- limit\n");
	printf("                Limit can be between 0 & 1.5\n");
	printf(" -r {ch}        Use DSM radio channel {ch} to control servo\n");
	printf(" -h             Print this help messege \n\n");
	printf("sample use to center servo channel 1:\n");
	printf("   rc_test_servo -c 1 -p 0.0\n\n");
}

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}

int main(int argc, char *argv[])
{
	double servo_pos=0;
	double sweep_limit=0;
	uint64_t dsm_nanos=0;
	int width_us=0;
	int ch=0; // channel to test, 0 means all channels
	double direction = 1; // switches between 1 & -1 in sweep mode
	test_mode_t mode = DISABLED; //start mode disabled
	int frequency_hz = 50; // default 50hz frequency to send pulses
	int i, c, radio_ch;

	// parse arguments
	opterr = 0;
	while ((c = getopt(argc, argv, "c:f:p:w:s:r:h")) != -1){
		switch (c){
		// channel option
		case 'c':
			ch = atoi(optarg);
			if(ch<RC_SERVO_CH_MIN || ch>RC_SERVO_CH_MAX){
				fprintf(stderr,"ERROR channel option must be between %d and %d\n", RC_SERVO_CH_MIN, RC_SERVO_CH_MAX);
				return -1;
			}
			break;

		// pulse frequency option
		case 'f':
			frequency_hz = atoi(optarg);
			if(frequency_hz<1){
				fprintf(stderr,"Frequency option must be >=1\n");
				return -1;
			}
			break;

		// position option
		case 'p':
			// make sure only one mode in requested
			if(mode!=DISABLED){
				__print_usage();
				return -1;
			}
			servo_pos = atof(optarg);
			if(servo_pos>1.5 || servo_pos<-1.5){
				fprintf(stderr,"Servo position must be from -1.5 to 1.5\n");
				return -1;
			}
			mode = NORM;
			break;

		// width in microseconds option
		case 'w':
			// make sure only one mode in requested
			if(mode!=DISABLED){
				__print_usage();
				return -1;
			}
			width_us = atof(optarg);
			if(width_us<10){
				fprintf(stderr,"ERROR: Width in microseconds must be >10\n");
				return -1;
			}
			mode = MICROSECONDS;
			break;

		// sweep mode option
		case 's':
			// make sure only one mode in requested
			if(mode!=DISABLED){
				__print_usage();
				return -1;
			}
			sweep_limit = atof(optarg);
			if(sweep_limit>1.5 || sweep_limit<-1.5){
				fprintf(stderr,"ERROR: Sweep limit must be from -1.5 to 1.5\n");
				return -1;
			}
			mode = SWEEP;
			servo_pos = 0.0;
			break;

		// radio mode option
		case 'r':
			// make sure only one mode in requested
			if(mode!=DISABLED){
				__print_usage();
				return -1;
			}
			radio_ch = atoi(optarg);
			if(radio_ch<1 || radio_ch>RC_MAX_DSM_CHANNELS){
				fprintf(stderr,"ERROR radio channel option must be between 1 and %d\n", RC_MAX_DSM_CHANNELS);
				return -1;
			}
			mode = RADIO;
			break;

		// help mode
		case 'h':
			__print_usage();
			return 0;

		default:
			printf("\nInvalid Argument \n");
			__print_usage();
			return -1;
		}
	}

	// if the user didn't give enough arguments, exit
	if(mode==DISABLED){
		fprintf(stderr,"\nNot enough input arguments\n");
		__print_usage();
		return -1;
	}

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running=1;

	// read adc to make sure battery is connected
	if(rc_adc_init()){
		fprintf(stderr,"ERROR: failed to run rc_adc_init()\n");
		return -1;
	}
	if(rc_adc_batt()<6.0){
		fprintf(stderr,"ERROR: battery disconnected or insufficiently charged to drive servos\n");
		return -1;
	}
	rc_adc_cleanup();

	// initialize PRU
	if(rc_servo_init()) return -1;

	// start radio if necessary
	if(mode==RADIO){
		if(rc_dsm_init()==-1) return -1;
		printf("Waiting for first DSM packet\n");
		fflush(stdout);
		while(rc_dsm_is_new_data()==0){
			if(running==0) return 0;
			rc_usleep(50000);
		}
	}

	// turn on power
	printf("Turning On 6V Servo Power Rail\n");
	rc_servo_power_rail_en(1);

	// print out what the program is doing
	printf("\n");
	if(ch==0) printf("Sending on all channels.\n");
	else	  printf("Sending only to channel %d.\n", ch);
	switch(mode){
	case NORM:
		printf("Using rc_servo_send_pulse_normalized\n");
		printf("Normalized Signal: %f  Pulse Frequency: %d\n", servo_pos, frequency_hz);
		break;
	case MICROSECONDS:
		printf("Using rc_servo_send_pulse_us\n");
		printf("Pulse_width: %d  Pulse Frequency: %d\n", width_us, frequency_hz);
		break;
	case SWEEP:
		printf("Sweeping servos back/forth between +-%f\n", sweep_limit);
		printf("Pulse Frequency: %d\n", frequency_hz);
		break;
	case RADIO:
		printf("Listening for DSM radio signal\n");
		printf("Pulse Frequency: %d\n", frequency_hz);
		break;
	default:
		// should never get here
		fprintf(stderr,"ERROR invalid mode enum\n");
		return -1;
	}



	// Main loop runs at frequency_hz
	while(running){
		switch(mode){

		case NORM:
			if(rc_servo_send_pulse_normalized(ch,servo_pos)==-1) return -1;
			break;

		case MICROSECONDS:
			if(rc_servo_send_pulse_us(ch, width_us)==-1) return -1;
			break;

		case SWEEP:
			// increase or decrease position each loop
			// scale with frequency
			servo_pos += direction * sweep_limit / frequency_hz;

			// reset pulse width at end of sweep
			if(servo_pos>sweep_limit){
				servo_pos = sweep_limit;
				direction = -1;
			}
			else if(servo_pos < (-sweep_limit)){
				servo_pos = -sweep_limit;
				direction = 1;
			}
			// send result
			if(rc_servo_send_pulse_normalized(ch,servo_pos)==-1) return -1;
			break;

		case RADIO:
			dsm_nanos = rc_dsm_nanos_since_last_packet();
			if(dsm_nanos > 200000000){
				printf("\rSeconds since last DSM packet: %.2f              ", dsm_nanos/1000000000.0);
			}
			else{
				servo_pos = rc_dsm_ch_normalized(radio_ch);
				// bound the signal to the escs
				if(servo_pos<-1.5) servo_pos=-1.5;
				if(servo_pos>1.5) servo_pos=1.5;

				// send pulse
				if(rc_servo_send_pulse_normalized(ch,servo_pos)==-1) return -1;

				// print info
				printf("\r");// keep printing on same line
				// print framerate
				printf("%d/", rc_dsm_resolution());
				// print num channels in use
				printf("%d-ch ", rc_dsm_channels());
				//print all channels
				for(i=0; i<rc_dsm_channels(); i++){
					printf("%d:% 0.2f ", i+1, rc_dsm_ch_normalized(i+1));
				}
			}
			break;




		default:
			fprintf(stderr,"ERROR unhandled mode\n");
			return -1;
		}

		// sleep roughly enough to maintain frequency_hz
		rc_usleep(1000000/frequency_hz);
	}

	rc_usleep(50000);
	// turn off power rail and cleanup
	rc_servo_power_rail_en(0);
	rc_servo_cleanup();
	rc_dsm_cleanup();
	return 0;
}

