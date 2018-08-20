/**
 * @file rc_test_motors.c
 * @example    rc_test_motors
 *
 * Demonstrates use of H-bridges to drive motors with the Robotics Cape and
 * BeagleBone Blue. Instructions are printed to the screen when called.
 */


#include <stdio.h>
#include <signal.h>
#include <stdlib.h> // for atoi
#include <getopt.h>
#include <rc/motor.h>
#include <rc/time.h>

static int running = 0;

// possible modes, user selected with command line arguments
typedef enum m_mode_t{
	DISABLED,
	NORMAL,
	BRAKE,
	FREE,
	SWEEP
} m_mode_t;

// printed if some invalid argument was given
static void __print_usage(void)
{
	printf("\n");
	printf("-d {duty}   define a duty cycle from -1.0 to 1.0\n");
	printf("-b          enable motor brake function\n");
	printf("-F {freq}   set a custom pwm frequency in HZ, otherwise default 25000 is used\n");
	printf("-f          enable free spin function\n");
	printf("-s {duty}   sweep motors back and forward at duty cycle\n");
	printf("-m {motor}  specify a single motor from 1-4, otherwise all will be driven\n");
	printf("            motors will be driven equally.\n");
	printf("-h          print this help message\n");
	printf("\n");
}

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}

int main(int argc, char *argv[])
{
	double duty = 0.0;
	int ch = 0; // assume all motor unless set otherwise
	int c, in;
	int freq_hz = RC_MOTOR_DEFAULT_PWM_FREQ;
	m_mode_t m_mode = DISABLED;

	// parse arguments
	opterr = 0;
	while ((c = getopt(argc, argv, "m:d:F:fbs:h")) != -1){
		switch (c){
		case 'm': // motor channel option
			in = atoi(optarg);
			if(in<=4 && in>=0){
				ch = in;
			}
			else{
				fprintf(stderr,"-m motor option must be from 0-4\n");
				return -1;
			}
			break;
		case 'd': // duty cycle option
			if(m_mode!=DISABLED) __print_usage();
			duty = atof(optarg);
			if(duty<=1 && duty >=-1){
				m_mode = NORMAL;
			}
			else{
				fprintf(stderr,"duty cycle must be from -1 to 1\n");
				return -1;
			}
			break;
		case 'F': // pwm frequency option
			freq_hz = atoi(optarg);
			if(freq_hz<1){
				fprintf(stderr,"PWM frequency must be >=1\n");
				return -1;
			}
			break;
		case 'f':
			if(m_mode!=DISABLED) __print_usage();
			m_mode = FREE;
			break;
		case 'b':
			if(m_mode!=DISABLED) __print_usage();
			m_mode = BRAKE;
			break;
		case 's':
			if(m_mode!=DISABLED) __print_usage();
			duty = atof(optarg);
			if(duty<=1 && duty >=-1){
				m_mode = SWEEP;
			}
			else{
				fprintf(stderr,"duty cycle must be from -1 to 1\n");
				return -1;
			}
			break;
		case 'h':
			__print_usage();
			return -1;
			break;
		default:
			__print_usage();
			return -1;
			break;
		}
	}

	// if the user didn't give enough arguments, print usage
	if(m_mode==DISABLED){
		__print_usage();
		return -1;
	}

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running =1;

	// initialize hardware first
	if(rc_motor_init_freq(freq_hz)) return -1;

	// decide what to do
	switch(m_mode){
	case NORMAL:
		printf("sending duty cycle %0.4f\n", duty);
		rc_motor_set(ch,duty);
		break;
	case FREE:
		printf("Free Spin Mode\n");
		rc_motor_free_spin(ch);
		break;
	case BRAKE:
		printf("Braking Mode\n");
		rc_motor_brake(ch);
		break;
	default:
		break;
	}

	// wait untill the user exits
	while(running){
		if(m_mode==SWEEP){
			duty = -duty; // toggle back and forth to sweep motors side to side
			printf("sending duty cycle %0.4f\n", duty);
			fflush(stdout);
			rc_motor_set(ch,duty);
		}

		// if not in SWEEP mode, the motors have already been set so do nothing
		rc_usleep(500000);
	}


	// final cleanup
	printf("\ncalling rc_motor_cleanup()\n");
	rc_motor_cleanup();
	return 0;
}

