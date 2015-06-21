// Sample Code for testing MPU-9150 operation
// James Strawson - 2013

#include <robotics_cape.h>
#define DEFAULT_SAMPLE_RATE	200  // This is also the fastest speed the DMP will do

/************************************************************************
* 	orientation_t
*	possible orientations
************************************************************************/
typedef enum orientation_t {
	FLAT,
	LEFT_DOWN,
	RIGHT_DOWN,
	NOSE_DOWN,
	NOSE_UP
}orientation_t;


/************************************************************************
* 	config_t
*	configuration struct for orientation detection
************************************************************************/
typedef struct config_t{
	int sample_rate; 		// rate to sample IMU in Hz
	float det_time;			// time in seconds for certain orientation
	float det_poss_time;	// time in seconds for possible orientation
	float det_tolerance;	// plus or minus degrees allowed about perfect
}config_t;

/************************************************************************
* 	core_state_t
*	global state for the orientation detection program
************************************************************************/
typedef struct core_state_t{
	int counter;
	orientation_t poss_orientation;
	orientation_t orientation;
}core_state_t;


/************************************************************************
* 	Global Variables
************************************************************************/
config_t conf;
core_state_t cstate;

/************************************************************************
* 	Function Definitions
************************************************************************/
int read_imu_data();
int print_orientation(orientation_t direction);
void* orientation_detector(void* ptr);

/************************************************************************
* 	int main()
*	initializes hardware and configuration struct.
* 	starts the IMU interrupt routine
*	starts the orientation detection thread
*	prints out orientation to the screen
************************************************************************/
int main(){
	initialize_cape();
	
	// initialize the global conf struct with some values
	conf.sample_rate = 50;	// Hz
	conf.det_time = 0.3;		// seconds
	conf.det_poss_time = 0.1;	// seconds
	conf.det_tolerance = 30; 	// degrees
	
	// initialize the IMU in flat orientation so that
	// flat is with the cape facing up
	signed char imu_orientation[9] = ORIENTATION_FLAT; 
	initialize_imu(conf.sample_rate, imu_orientation);
	
	//start the interrupt handler
	set_imu_interrupt_func(&read_imu_data);
	
	// start the orientation detection thread in the background
	pthread_t orientation_thread;
	pthread_create(&orientation_thread, NULL, orientation_detector, (void*) NULL);
	
	// print a header
	printf("Move your BBB around and observe the changing orientation\n");
	printf("\n\ncounter  possible  certain   \n");
	
	//now just wait, print_imu_data will run
	while (get_state() != EXITING) {
		printf("\r");
		printf(" %3d    ", cstate.counter);
		print_orientation(cstate.poss_orientation);
		printf(" ");
		print_orientation(cstate.orientation);
		printf("               ");
		// its important we flush the output to make sure
		// the next printf's print onto a clean line
		fflush(stdout); 
		
		// sleep for 10ms
		usleep(10000);
	}
	cleanup_cape();
	return 0;
}


/************************************************************************
* 	int read_imu_data()
*	only purpose is to update the global mpu struct with new 
*	data when it is read in. This is called off of an interrupt.
************************************************************************/
int read_imu_data(){
	return mpu9150_read(&mpu);
}

/************************************************************************
* 	int print_orientation(orientation_t orient)
*	prints a human readble version of the orientation enum
************************************************************************/
int print_orientation(orientation_t orient){
	switch(orient){
	case FLAT:
		printf("   FLAT  ");
		break;
	case LEFT_DOWN:
		printf("LEFT_DOWN");
		break;
	case RIGHT_DOWN:
		printf("RIGHT_DOWN");
		break;
	case NOSE_DOWN:
		printf(" NOSE_DOWN");
		break;
	case NOSE_UP:
		printf(" NOSE_UP  ");
		break;
	default:
		printf("unknown orientation");
		return -1;
		break;
	}
	return 0;
}


/************************************************************************
* 	void* orientation_detector(void* ptr)
*	independent thread that montitors the imu data and determines
*	a possible orientation quickly and a definite orientation more 
* 	slowly with more certainty. 
************************************************************************/
void* orientation_detector(void* ptr){
	// local copies of roll and pitch read from IMU
	float roll, pitch;
	
	// local orientation based on simple imu sample
	orientation_t immediate_orientation;
	
	// counter limits for vertain orientation and possible orientation
	int counter_limit=lrint(conf.det_time*conf.sample_rate);
	int counter_poss_limit=lrint(conf.det_poss_time*conf.sample_rate);
	
	// start the counter at 0
	cstate.counter = 0;
	
	// run untill the rest of the program closes
	while(get_state()!=EXITING){
		// make local copies of roll and pitch from IMU
		roll = mpu.fusedEuler[VEC3_X] * RAD_TO_DEGREE;
		pitch = mpu.fusedEuler[VEC3_Y] * RAD_TO_DEGREE;
		
		// check for flat
		if(fabs(roll)<(90-conf.det_tolerance)&& \
				fabs(pitch)<(90-conf.det_tolerance)){
			if (immediate_orientation != FLAT){
				cstate.counter = 0;
				immediate_orientation = FLAT;
			}
			cstate.counter ++;
		}
	
		// check for NOSE_DOWN
		else if(roll>-90 && roll<(-90+conf.det_tolerance)){
			if (immediate_orientation != NOSE_DOWN){
				cstate.counter = 0;
				immediate_orientation = NOSE_DOWN;
			}
			cstate.counter ++;
		}

		// check for NOSE_UP
		else if(roll<90 && roll>(90-conf.det_tolerance)){
			if (immediate_orientation != NOSE_UP){
				cstate.counter = 0;
				immediate_orientation = NOSE_UP;
			}
			cstate.counter ++;
		}
		
		// check for nose LEFT_DOWN
		else if(pitch<90 && pitch>(90-conf.det_tolerance)){
			if (immediate_orientation != LEFT_DOWN){
				cstate.counter = 0;
				immediate_orientation = LEFT_DOWN;
			}
			cstate.counter ++;
		}
		
		// check for RIGHT_DOWN
		else if(pitch>-90 && pitch<(-90+conf.det_tolerance)){
			if (immediate_orientation != RIGHT_DOWN){
				cstate.counter = 0;
				immediate_orientation = RIGHT_DOWN;
			}
			cstate.counter ++;
		}
		
		// check for possible counter timeout
		if(cstate.counter>=counter_poss_limit){
			cstate.poss_orientation = immediate_orientation;
		}
		
		// check for certain counter timeout
		if(cstate.counter>=counter_limit){
			cstate.orientation = immediate_orientation;
			// to prevent the counter from overflowing, stop it here
			cstate.counter = counter_limit;
		}
		
		// sleep for roughly enough time to keep the sample rate
		usleep(1000000/conf.sample_rate);
	}
	
	return NULL;
}

