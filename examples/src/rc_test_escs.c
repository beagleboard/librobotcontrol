/**
 * @file rc_test_esc.c
 * @example    rc_test_esc
 *
 *
 * Demonstrates use of pru to control servos and ESCs with pulses. This program
 * operates in 4 different modes. See the option list below for how to select an
 * operational mode from the command line.
 *
 * SERVO: uses rc_send_servo_pulse_normalized() to set one or all servo
 * positions to a value from -1.5 to 1.5 corresponding to their extended range.
 * -1 to 1 is considered the "safe" normal range as some servos will not go
 * beyond this. Test your servos incrementally to find their safe range.
 *
 * ESC: For unidirectional brushless motor speed controllers specify a range
 * from 0 to 1 as opposed to the bidirectional servo range. Be sure to run the
 * calibrate_esc example first to make sure the ESCs are calibrated to the right
 * pulse range. This mode uses the rc_send_esc_pulse_normalized() function.
 *
 * WIDTH: You can also specify your own pulse width in microseconds (us).
 * This uses the rc_send_servo_pulse_us() function.
 *
 * SWEEP: This is intended to gently sweep a servo back and forth about the
 * center position. Specify a range limit as a command line argument as
 * described below. This also uses the rc_send_servo_pulse_normalized()
 * function.
 *
 *
 *
 * @author     James Strawson
 * @date       3/20/2018
 */

#include <stdio.h>
#include <getopt.h>
#include <stdlib.h> // for atoi
#include <signal.h>
#include <rc/time.h>
#include <rc/dsm.h>
#include <rc/servo.h>

static int running;

typedef enum test_mode_t{
	DISABLED,
	NORM,
	WIDTH,
	SWEEP,
	RADIO
}test_mode_t;


void print_usage()
{
	printf("\n");
	printf(" Options\n");
	printf(" -c {channel}   Specify one channel to be driven from 1-8.\n");
	printf("                Otherwise all channels will be driven equally\n");
	printf(" -f {hz}        Specify pulse frequency, otherwise 50hz is used\n");
	printf(" -t {throttle}  Throttle to send between -0.1 & 1.0\n");
	printf(" -o             Enable One-Shot mode\n");
	printf(" -w {width_us}  Send pulse width in microseconds (us)\n");
	printf(" -s {max}       Gently sweep throttle from 0 to {max} back to 0 again\n");
	printf("                {max} can be between 0 & 1.0\n");
	printf(" -r {ch}        Use DSM radio channel {ch} to control ESC\n");
	printf(" -h             Print this help messege \n\n");
	printf("sample use to control ESC channel 2 with DSM radio channel 1:\n");
	printf("   rc_test_esc -c 2 -r 1\n\n");
	printf("sample use to sweep all ESC channels from 0 to quarter throttle with oneshot mode\n");
	printf("   rc_test_esc -o -s 0.25\n\n");
}

// interrupt handler to catch ctrl-c
void signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}

int main(int argc, char *argv[])
{
	float sweep_limit = 0;	// max throttle allowed when sweeping
	int oneshot_en = 0;	// set to 1 if oneshot is enabled
	float thr = 0;		// normalized throttle
	int width_us = 0;	// pulse width in microseconds mode
	int ch = 0;		// channel to test, 0 means all channels
	int radio_ch;		// DSM radio channel to watch
	float dir = 1;		// switches between 1 & -1 in sweep mode
	int c,i;		// misc variables
	test_mode_t mode;	// current operating mode
	uint64_t dsm_nanos;	// nanoseconds since last dsm packet
	int frequency_hz = 50; // default 50hz frequency to send pulses

	// start with mode as disabled
	mode = DISABLED;

	// parse arguments
	opterr = 0;
	while ((c = getopt(argc, argv, "c:f:t:ow:s:r:h")) != -1){
		switch(c){
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

		// oneshot mode option
		case 'o':
			if(mode==WIDTH){
				fprintf(stderr,"enabling oneshot mode when defining your own pulse width makes no sense\n");
				return -1;
			}
			oneshot_en=1;
			break;

		// throttle
		case 't':
			// make sure only one mode in requested
			if(mode!=DISABLED){
				fprintf(stderr,"ERROR please select only one mode to use\n");
				print_usage();
				return -1;
			}
			thr = atof(optarg);
			if(thr>1.0 || thr<-0.1){
				fprintf(stderr,"ERROR throttle must be from -0.1 to 1\n");
				return -1;
			}
			mode = NORM;
			break;

		// width in microseconds option
		case 'w':
			// make sure only one mode in requested
			if(mode!=DISABLED){
				fprintf(stderr,"ERROR please select only one mode to use\n");
				print_usage();
				return -1;
			}
			if(oneshot_en){
				fprintf(stderr,"enabling oneshot mode when defining your own pulse width makes no sense\n");
				return -1;
			}
			width_us = atof(optarg);
			if(width_us<10){
				printf("ERROR: Width in microseconds must be >10\n");
				return -1;
			}
			mode = WIDTH;
			break;

		// sweep mode option
		case 's':
			// make sure only one mode in requested
			if(mode!=DISABLED){
				fprintf(stderr,"ERROR please select only one mode to use\n");
				print_usage();
				return -1;
			}
			sweep_limit = atof(optarg);
			if(sweep_limit>1.0 || sweep_limit<0){
				fprintf(stderr,"ERROR: Sweep limit must be from 0 to 1.0\n");
				return -1;
			}
			mode = SWEEP;
			thr=0;
			dir=1;
			break;

		// radio mode option
		case 'r':
			// make sure only one mode in requested
			if(mode!=DISABLED){
				fprintf(stderr,"ERROR please select only one mode to use\n");
				print_usage();
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
			print_usage();
			return 0;

		default:
			printf("\nInvalid Argument \n");
			print_usage();
			return -1;
		}
	}

	// if the user didn't give enough arguments, exit
	if(mode==DISABLED){
		fprintf(stderr,"\nNot enough input arguments\n");
		print_usage();
		return -1;
	}

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, signal_handler);
	running=1;

	// initialize PRU and make sure power rail is OFF
	if(rc_servo_init()) return -1;
	rc_servo_power_rail_en(0);

	// wait for radio to start
	if(mode==RADIO){
		if(rc_dsm_init()==-1) return -1;
		printf("Waiting for first DSM packet\n");
		fflush(stdout);
		while(rc_dsm_is_new_data()==0){
			if(running==0) return 0;
			rc_usleep(50000);
		}
	}

	// if driving an ESC, send throttle of 0 first
	// otherwise it will go into calibration mode
	printf("waking ESC up from idle for 3 seconds\n");
	for(i=0;i<frequency_hz*3;i++){
		rc_servo_send_esc_pulse_normalized(ch,-0.1);
		rc_usleep(1000000/frequency_hz);
	}

	// Main loop runs at frequency_hz
	while(running){
		switch(mode){

		case NORM:
			if(oneshot_en) rc_servo_send_oneshot_pulse_normalized(ch,thr);
			else rc_servo_send_esc_pulse_normalized(ch,thr);
			break;

		case WIDTH:
			rc_servo_send_pulse_us(ch,width_us);
			break;

		case SWEEP:
			// increase or decrease position each loop
			// scale with frequency
			thr += dir * sweep_limit / frequency_hz;

			// reset pulse width at end of sweep
			if(thr > sweep_limit){
				thr = sweep_limit;
				dir = -1;
			}
			else if(thr < 0){
				thr = 0;
				dir = 1;
			}
			// send result
			if(oneshot_en) rc_servo_send_oneshot_pulse_normalized(ch,thr);
			else rc_servo_send_esc_pulse_normalized(ch,thr);
			break;

		case RADIO:
			dsm_nanos = rc_dsm_nanos_since_last_packet();
			if(dsm_nanos > 200000000){
				if(oneshot_en) rc_servo_send_oneshot_pulse_normalized(ch,0);
				else rc_servo_send_esc_pulse_normalized(ch,0);
				printf("\rSeconds since last DSM packet: %.2f              ", dsm_nanos/1000000000.0);
			}
			else{
				thr = rc_dsm_ch_normalized(radio_ch);
				// bound the signal to the escs
				if(thr <-0.1) thr=-0.1;
				if(thr > 1.0) thr=1.0;

				// send pulse
				if(oneshot_en) rc_servo_send_oneshot_pulse_normalized(ch,thr);
				else rc_servo_send_esc_pulse_normalized(ch,thr);

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

	// cleanup
	if(oneshot_en) rc_servo_send_oneshot_pulse_normalized(ch,-0.1);
	else rc_servo_send_esc_pulse_normalized(ch,-0.1);
	rc_usleep(50000);
	rc_servo_cleanup();
	rc_dsm_cleanup();
	return 0;
}

