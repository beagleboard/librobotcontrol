/*******************************************************************************
* test_motors.c
*
* James Strawson 2016
* demonstrates use of H-bridges to drive motors. Instructions are printed
* to the screen when called.
*******************************************************************************/

#include "../../libraries/roboticscape-usefulincludes.h"
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
	printf("-d {duty}		define a duty cycle from -1.0 to 1.0\n");
	printf("-b  			enbale motor brake function\n");
	printf("-f 			enable free spin function\n");
	printf("-s {duty}		sweep motors back and foward at duty cycle\n");
	printf("-m {motor}		specify a single motor from 1-4, otherwise all\n");
	printf("			motors will be driven equally.\n");
	printf("-h			print this help message\n");
	printf("\n");
}

int main(int argc, char *argv[]){
	float duty;
	int ch, c, in;
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
	
	// sanity check cape library initialized
	if(initialize_roboticscape()){
		printf("failed to initialize cape\n");
		return -1;
	}

	// bring H-bridges of of standby
	enable_motors(); 
	rc_set_led(GREEN,ON);
	rc_set_led(RED,ON);
	
	// decide what to do
	switch(m_mode){
	case NORMAL:
		if(all){
			printf("sending duty cycle %0.4f to all motors\n", duty);
			set_motor_all(duty);
		}
		else{
			printf("sending duty cycle %0.4f to motor %d\n", duty, ch);
			set_motor(ch,duty);
		}
		break;
	case FREE:
		if(all){
			printf("Letting all motors free spin\n");
			set_motor_free_spin_all(duty);
		}
		else{
			printf("Letting motor %d free spin\n", ch);
			set_motor_free_spin(ch);
		}
		break;
	case BRAKE:
		if(all){
			printf("Braking all motors\n");
			set_motor_brake_all();
		}
		else{
			printf("Braking motor %d\n", ch);
			set_motor_brake(ch);
		}
		break;
	default:
		break;
	}
	
	// wait untill the user exits
	while(rc_get_state() != EXITING){
		if(m_mode==SWEEP){
			duty = -duty; // toggle back and forth to sweep motors side to side
			if(all){
				printf("sending duty cycle %0.4f to all motors\n", duty);
				set_motor_all(duty);
			}
			else{
				printf("sending duty cycle %0.4f to motor %d\n", duty, ch);
				set_motor(ch,duty);
			}
		}
		
		// if not in SWEEP mode, the motors have already been set so do nothing
		usleep(500000);
	}
	
	// User must have existed, put H-bridges into standby
	// not entirely necessary since cleanup_cape does this too
	disable_motors();	
	printf("All Motors Off\n\n");
	
	// final cleanup
	cleanup_roboticscape();
	return 0;
}

