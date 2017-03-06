/*******************************************************************************
* rc_test_motors.c
*
* demonstrates use of H-bridges to drive motors. Instructions are printed
* to the screen when called.
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

// possible modes, user selected with command line arguments
typedef enum m_mode_t{
	DISABLED,
	NORMAL,
	BRAKE,
	FREE,
	SWEEP
} m_mode_t;

// printed if some invalid argument was given
void print_usage(){
	printf("\n");
	printf("-d {duty}   define a duty cycle from -1.0 to 1.0\n");
	printf("-b          enable motor brake function\n");
	printf("-f          enable free spin function\n");
	printf("-s {duty}   sweep motors back and forward at duty cycle\n");
	printf("-m {motor}  specify a single motor from 1-4, otherwise all\n");
	printf("            motors will be driven equally.\n");
	printf("-h          print this help message\n");
	printf("\n");
}

int main(int argc, char *argv[]){
	double duty = 0.0;
	int ch = 1;
	int c, in;
	int all = 1;	// set to 0 if a motor (-m) argument is given 
	m_mode_t m_mode = DISABLED;
	
	// parse arguments
	opterr = 0;
	while ((c = getopt(argc, argv, "m:d:fbs:h")) != -1){
		switch (c){
		case 'm': // motor channel option
			in = atoi(optarg);
			if(in<=4 && in>=1){
				ch = in;
				all = 0;
			}
			else{
				printf("motor option must be from 1-4\n");
				return -1;
			}
			break;
		case 'd': // duty cycle option
			if(m_mode!=DISABLED) print_usage();
			duty = atof(optarg);
			if(duty<=1 && duty >=-1){
				m_mode = NORMAL;
			}
			else{
				printf("duty cycle must be from -1 to 1\n");
				return -1;
			}
			break;
		case 'f':
			if(m_mode!=DISABLED) print_usage();
			m_mode = FREE;
			break;
		case 'b':
			if(m_mode!=DISABLED) print_usage();
			m_mode = BRAKE;
			break;
		case 's':
			if(m_mode!=DISABLED) print_usage();
			duty = atof(optarg);
			if(duty<=1 && duty >=-1){
				m_mode = SWEEP;
			}
			else{
				printf("duty cycle must be from -1 to 1\n");
				return -1;
			}
			break;
		case 'h':
			print_usage();
			return -1;
			break;
		default:
			print_usage();
			return -1;
			break;
		}
	}
	
	// if the user didn't give enough arguments, print usage
	if(m_mode==DISABLED){
		print_usage();
		return -1;
	}
	
	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}

	// bring H-bridges of of standby
	rc_enable_motors(); 
	rc_set_led(GREEN,ON);
	rc_set_led(RED,ON);
	
	// decide what to do
	switch(m_mode){
	case NORMAL:
		if(all){
			printf("sending duty cycle %0.4f to all motors\n", duty);
			rc_set_motor_all(duty);
		}
		else{
			printf("sending duty cycle %0.4f to motor %d\n", duty, ch);
			rc_set_motor(ch,duty);
		}
		break;
	case FREE:
		if(all){
			printf("Letting all motors free spin\n");
			rc_set_motor_free_spin_all(duty);
		}
		else{
			printf("Letting motor %d free spin\n", ch);
			rc_set_motor_free_spin(ch);
		}
		break;
	case BRAKE:
		if(all){
			printf("Braking all motors\n");
			rc_set_motor_brake_all();
		}
		else{
			printf("Braking motor %d\n", ch);
			rc_set_motor_brake(ch);
		}
		break;
	default:
		break;
	}
	
	// wait untill the user exits
	while(rc_get_state()!=EXITING){
		if(m_mode==SWEEP){
			duty = -duty; // toggle back and forth to sweep motors side to side
			if(all){
				printf("sending duty cycle %0.4f to all motors\n", duty);
				rc_set_motor_all(duty);
			}
			else{
				printf("sending duty cycle %0.4f to motor %d\n", duty, ch);
				rc_set_motor(ch,duty);
			}
		}
		
		// if not in SWEEP mode, the motors have already been set so do nothing
		rc_usleep(500000);
	}
	
	// User must have existed, put H-bridges into standby
	// not entirely necessary since cleanup_cape does this too
	rc_disable_motors();	
	printf("All Motors Off\n\n");
	
	// final cleanup
	rc_cleanup();
	return 0;
}

