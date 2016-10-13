/*******************************************************************************
* balance.c
*
* James Strawson 2016
* Reference solution for balancing EduMiP
*******************************************************************************/

#include "../../libraries/usefulincludes.h"
#include "../../libraries/roboticscape.h"

#include "balance_config.h"

/*******************************************************************************
* drive_mode_t
*
* NOVICE: Drive rate and turn rate are limited to make driving easier.
* ADVANCED: Faster drive and turn rate for more fun.
*******************************************************************************/
typedef enum drive_mode_t{
	NOVICE,
	ADVANCED
}drive_mode_t;

/*******************************************************************************
* arm_state_t
*
* ARMED or DISARMED to indicate if the controller is running
*******************************************************************************/
typedef enum arm_state_t{
	ARMED,
	DISARMED
}arm_state_t;

/*******************************************************************************
* setpoint_t
*	
* Controller setpoint written to by setpoint_manager and read by the controller.
*******************************************************************************/
typedef struct setpoint_t{
	arm_state_t arm_state;	// see arm_state_t declaration
	drive_mode_t drive_mode;// NOVICE or ADVANCED
	float theta;			// body lean angle (rad)
	float phi;				// wheel position (rad)
	float phi_dot;			// rate at which phi reference updates (rad/s)
	float gamma;			// body turn angle (rad)
	float gamma_dot;		// rate at which gamma setpoint updates (rad/s)
}setpoint_t;

/*******************************************************************************
* core_state_t
*
* This is the system state written to by the balance controller.	
*******************************************************************************/
typedef struct core_state_t{
	float wheelAngleR;	// wheel rotation relative to body
	float wheelAngleL;
	float theta; 		// body angle radians
	float phi;			// average wheel angle in global frame
	float gamma;		// body turn (yaw) angle radians
	float vBatt; 		// battery voltage 
	float d1_u;			// output of balance controller D1 to motors
	float d2_u;			// output of position controller D2 (theta_ref)
	float d3_u;			// output of steering controller D3 to motors
	float mot_drive;	// u compensated for battery voltage
} core_state_t;

/*******************************************************************************
* Local Function declarations	
*******************************************************************************/
// IMU interrupt routine
int balance_controller(); 
// threads
void* setpoint_manager(void* ptr);
void* battery_checker(void* ptr);
void* printf_loop(void* ptr);
// regular functions
int zero_out_controller();
int disarm_controller();
int arm_controller();
int wait_for_starting_condition();
int on_pause_press();
int on_mode_release();
int blink_green();
int blink_red();

/*******************************************************************************
* Global Variables				
*******************************************************************************/
core_state_t cstate;
setpoint_t setpoint;
d_filter_t D1, D2, D3;	
imu_data_t imu_data;

/*******************************************************************************
* main()
*
* Initialize the filters, IMU, threads, & wait untill shut down
*******************************************************************************/
int main(){
	set_cpu_frequency(FREQ_1000MHZ);

	if(initialize_cape()<0){
		printf("ERROR: failed to initialize cape\n");
		return -1;
	}
	set_led(RED,1);
	set_led(GREEN,0);
	set_state(UNINITIALIZED);


	// make sure setpoint starts at normal values
	setpoint.arm_state = DISARMED;
	setpoint.drive_mode = NOVICE;
	
	// set up D1 Theta controller
	float D1_num[] = D1_NUM;
	float D1_den[] = D1_DEN;
	D1 = create_filter(D1_ORDER, DT, D1_num, D1_den);
	D1.gain = D1_GAIN;
	enable_saturation(&D1, -1.0, 1.0);
	enable_soft_start(&D1, SOFT_START_SEC);
	
	// set up D2 Phi controller
	float D2_num[] = D2_NUM;
	float D2_den[] = D2_DEN;
	D2 = create_filter(D2_ORDER, DT, D2_num, D2_den);
	D2.gain = D2_GAIN;
	enable_saturation(&D2, -THETA_REF_MAX, THETA_REF_MAX);

	// set up D3 gamma (steering) controller
	D3 = create_pid(D3_KP, D3_KI, D3_KD, 4*DT, DT);
	enable_saturation(&D3, -STEERING_INPUT_MAX, STEERING_INPUT_MAX);

	// set up button handlers
	set_pause_pressed_func(&on_pause_press);
	set_mode_released_func(&on_mode_release);
	
	// start a thread to slowly sample battery 
	pthread_t  battery_thread;
	pthread_create(&battery_thread, NULL, battery_checker, (void*) NULL);
	// wait for the battery thread to make the first read
	while(cstate.vBatt==0 && get_state()!=EXITING) usleep(1000);
	
	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	if(isatty(fileno(stdout))){
		pthread_t  printf_thread;
		pthread_create(&printf_thread, NULL, printf_loop, (void*) NULL);
	}
	
	// set up IMU configuration
	imu_config_t imu_config = get_default_imu_config();
	imu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	imu_config.orientation = ORIENTATION_Y_UP;

	// start imu
	if(initialize_imu_dmp(&imu_data, imu_config)){
		printf("ERROR: can't talk to IMU, all hope is lost\n");
		blink_led(RED, 5, 5);
		return -1;
	}
	
	// start balance stack to control setpoints
	pthread_t  setpoint_thread;
	pthread_create(&setpoint_thread, NULL, setpoint_manager, (void*) NULL);

	// this should be the last step in initialization 
	// to make sure other setup functions don't interfere
	set_imu_interrupt_func(&balance_controller);
	
	// start in the RUNNING state, pressing the puase button will swap to 
	// the PUASED state then back again.
	printf("\nHold your MIP upright to begin balancing\n");
	set_state(RUNNING);
	
	// chill until something exits the program
	while(get_state()!=EXITING){
		usleep(10000);
	}
	
	// cleanup
	power_off_imu();
	cleanup_cape();
	set_cpu_frequency(FREQ_ONDEMAND);
	return 0;
}

/*******************************************************************************
* void* setpoint_manager(void* ptr)
*
* This thread is in charge of adjusting the controller setpoint based on user
* inputs from DSM2 radio control. Also detects pickup to control arming the
* controller.
*******************************************************************************/
void* setpoint_manager(void* ptr){
	float drive_stick, turn_stick; // dsm2 input sticks

	// wait for IMU to settle
	disarm_controller();
	usleep(1000000);
	usleep(1000000);
	usleep(500000);
	set_state(RUNNING);
	set_led(RED,0);
	set_led(GREEN,1);
	
	while(get_state()!=EXITING){
		// sleep at beginning of loop so we can use the 'continue' statement
		usleep(1000000/SETPOINT_MANAGER_HZ); 
		
		// nothing to do if paused, go back to beginning of loop
		if(get_state() != RUNNING) continue;

		// if we got here the state is RUNNING, but controller is not
		// necessarily armed. If DISARMED, wait for the user to pick MIP up
		// which will we detected by wait_for_starting_condition()
		if(setpoint.arm_state == DISARMED){
			if(wait_for_starting_condition()==0){
				zero_out_controller();
				arm_controller();
			} 
			else continue;
		}
	
		// if dsm2 is active, update the setpoint rates
		if(is_new_dsm2_data()){
			// Read normalized (+-1) inputs from RC radio stick and multiply by 
			// polarity setting so positive stick means positive setpoint
			turn_stick  = get_dsm2_ch_normalized(DSM2_TURN_CH) * DSM2_TURN_POL;
			drive_stick = get_dsm2_ch_normalized(DSM2_DRIVE_CH)* DSM2_DRIVE_POL;
			
			// saturate the inputs to avoid possible erratic behavior
			saturate_float(&drive_stick,-1,1);
			saturate_float(&turn_stick,-1,1);
			
			// use a small deadzone to prevent slow drifts in position
			if(fabs(drive_stick)<DSM2_DEAD_ZONE) drive_stick = 0.0;
			if(fabs(turn_stick)<DSM2_DEAD_ZONE)  turn_stick  = 0.0;

			// translate normalized user input to real setpoint values
			switch(setpoint.drive_mode){
			case NOVICE:
				setpoint.phi_dot   = DRIVE_RATE_NOVICE * drive_stick;
				setpoint.gamma_dot =  TURN_RATE_NOVICE * turn_stick;
				break;
			case ADVANCED:
				setpoint.phi_dot   = DRIVE_RATE_ADVANCED * drive_stick;
				setpoint.gamma_dot = TURN_RATE_ADVANCED  * turn_stick;
				break;
			default: break;
			}
		}
		// if dsm2 had timed out, put setpoint rates back to 0
		else if(is_dsm2_active()==0){
			setpoint.theta = 0;
			setpoint.phi_dot = 0;
			setpoint.gamma_dot = 0;
			continue;
		}
	}

	// if state becomes EXITING the above loop exists and we disarm here
	disarm_controller();
	return NULL;
}

/*******************************************************************************
* balance_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*******************************************************************************/
int balance_controller(){
	static int inner_saturation_counter = 0; 
	float dutyL, dutyR;
	/******************************************************************
	* STATE_ESTIMATION
	* read sensors and compute the state when either ARMED or DISARMED
	******************************************************************/
	// angle theta is positive in the direction of forward tip around X axis
	cstate.theta = imu_data.dmp_TaitBryan[TB_PITCH_X] + CAPE_MOUNT_ANGLE; 
	
	// collect encoder positions, right wheel is reversed 
	cstate.wheelAngleR = (get_encoder_pos(ENCODER_CHANNEL_R) * TWO_PI) \
								/(ENCODER_POLARITY_R * GEARBOX * ENCODER_RES);
	cstate.wheelAngleL = (get_encoder_pos(ENCODER_CHANNEL_L) * TWO_PI) \
								/(ENCODER_POLARITY_L * GEARBOX * ENCODER_RES);
	
	// Phi is average wheel rotation also add theta body angle to get absolute 
	// wheel position in global frame since encoders are attachde to the body
	cstate.phi = ((cstate.wheelAngleL+cstate.wheelAngleR)/2) + cstate.theta; 
	
	// steering angle gamma estimate 
	cstate.gamma = (cstate.wheelAngleR-cstate.wheelAngleL) \
											* (WHEEL_RADIUS_M/TRACK_WIDTH_M);

	/*************************************************************
	* check for various exit conditions AFTER state estimate
	***************************************************************/
	if(get_state() == EXITING){
		disable_motors();
		return 0;
	}
	// if controller is still ARMED while state is PAUSED, disarm it
	if(get_state()!=RUNNING && setpoint.arm_state==ARMED){
		disarm_controller();
		return 0;
	}
	// exit if the controller is disarmed
	if(setpoint.arm_state==DISARMED){
		return 0;
	}
	
	// check for a tipover
	if(fabs(cstate.theta) > TIP_ANGLE){
		disarm_controller();
		printf("tip detected \n");
		return 0;
	}
	
	/************************************************************
	* OUTER LOOP PHI controller D2
	* Move the position setpoint based on phi_dot. 
	* Input to the controller is phi error (setpoint-state).
	*************************************************************/
	if(ENABLE_POSITION_HOLD){
		if(setpoint.phi_dot != 0.0) setpoint.phi += setpoint.phi_dot*DT;
		cstate.d2_u = march_filter(&D2,setpoint.phi-cstate.phi);
		setpoint.theta = cstate.d2_u;
	}
	else setpoint.theta = 0.0;
	
	/************************************************************
	* INNER LOOP ANGLE Theta controller D1
	* Input to D1 is theta error (setpoint-state). Then scale the 
	* output u to compensate for changing battery voltage.
	*************************************************************/
	D1.gain = D1_GAIN * V_NOMINAL/cstate.vBatt;
	cstate.d1_u = march_filter(&D1,setpoint.theta - cstate.theta);

	/*************************************************************
	* Check if the inner loop saturated. If it saturates for over
	* a second disarm the controller to prevent stalling motors.
	*************************************************************/
	if(did_filter_saturate(&D1)) inner_saturation_counter++;
	else inner_saturation_counter = 0; 
 	// if saturate for a second, disarm for safety
	if(inner_saturation_counter > (SAMPLE_RATE_HZ*D1_SATURATION_TIMEOUT)){
		printf("inner loop controller saturated\n");
		disarm_controller();
		inner_saturation_counter = 0;
		return 0;
	}
	
	/**********************************************************
	* gama (steering) controller D3
	* move the setpoint gamma based on user input like phi
	***********************************************************/
	if(setpoint.gamma_dot != 0.0) setpoint.gamma += setpoint.gamma_dot * DT;
	cstate.d3_u = march_filter(&D3,setpoint.gamma - cstate.gamma);
	
	/**********************************************************
	* Send signal to motors
	* add D1 balance control u and D3 steering control also 
	* multiply by polarity to make sure direction is correct.
	***********************************************************/
	dutyL = cstate.d1_u - cstate.d3_u;
	dutyR = cstate.d1_u + cstate.d3_u;	
	set_motor(MOTOR_CHANNEL_L, MOTOR_POLARITY_L * dutyL); 
	set_motor(MOTOR_CHANNEL_R, MOTOR_POLARITY_R * dutyR); 

	return 0;
}

/*******************************************************************************
* 	zero_out_controller()
*
*	Clear the controller's memory and zero out setpoints.
*******************************************************************************/
int zero_out_controller(){
	reset_filter(&D1);
	reset_filter(&D2);
	reset_filter(&D3);
	setpoint.theta = 0.0;
	setpoint.phi   = 0.0;
	setpoint.gamma = 0.0;
	set_motor_all(0);
	return 0;
}

/*******************************************************************************
* disarm_controller()
*
* disable motors & set the setpoint.core_mode to DISARMED
*******************************************************************************/
int disarm_controller(){
	disable_motors();
	setpoint.arm_state = DISARMED;
	return 0;
}

/*******************************************************************************
* arm_controller()
*
* zero out the controller & encoders. Enable motors & arm the controller.
*******************************************************************************/
int arm_controller(){
	zero_out_controller();
	set_encoder_pos(ENCODER_CHANNEL_L,0);
	set_encoder_pos(ENCODER_CHANNEL_R,0);
	// prefill_filter_inputs(&D1,cstate.theta); 
	setpoint.arm_state = ARMED;
	enable_motors();
	return 0;
}

/*******************************************************************************
* int wait_for_starting_condition()
*
* Wait for MiP to be held upright long enough to begin.
* Returns 0 if successful. Returns -1 if the wait process was interrupted by 
* pause button or shutdown signal.
*******************************************************************************/
int wait_for_starting_condition(){
	int checks = 0;
	const int check_hz = 20;	// check 20 times per second
	int checks_needed = round(START_DELAY*check_hz);
	int wait_us = 1000000/check_hz; 

	// exit if state becomes paused or exiting
	while(get_state()==RUNNING){
		// if within range, start counting
		if(fabs(cstate.theta) < START_ANGLE){
			checks++;
			// waited long enough, return
			if(checks >= checks_needed) return 0;
		}
		// fell out of range, restart counter
		else checks = 0;
		usleep(wait_us);
	}
	return -1;
}

/*******************************************************************************
* battery_checker()
*
* Slow loop checking battery voltage. Also changes the D1 saturation limit
* since that is dependent on the battery voltage.
*******************************************************************************/
void* battery_checker(void* ptr){
	float new_v;
	while(get_state()!=EXITING){
		new_v = get_battery_voltage();
		// if the value doesn't make sense, use nominal voltage
		if (new_v>9.0 || new_v<5.0) new_v = V_NOMINAL;
		cstate.vBatt = new_v;
		usleep(1000000 / BATTERY_CHECK_HZ);
	}
	return NULL;
}

/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*******************************************************************************/
void* printf_loop(void* ptr){
	state_t last_state, new_state; // keep track of last state 
	while(get_state()!=EXITING){
		new_state = get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("    θ    |");
			printf("  θ_ref  |");
			printf("    φ    |");
			printf("  φ_ref  |");
			printf("    γ    |");
			printf("  D1_u   |");
			printf("  D3_u   |");
			printf("  vBatt  |");
			printf("arm_state|");
			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED: press pause again to start.\n");
		}
		last_state = new_state;
		
		// decide what to print or exit
		if(new_state == RUNNING){	
			printf("\r");
			printf("%7.2f  |", cstate.theta);
			printf("%7.2f  |", setpoint.theta);
			printf("%7.2f  |", cstate.phi);
			printf("%7.2f  |", setpoint.phi);
			printf("%7.2f  |", cstate.gamma);
			printf("%7.2f  |", cstate.d1_u);
			printf("%7.2f  |", cstate.d3_u);
			printf("%7.2f  |", cstate.vBatt);
			
			if(setpoint.arm_state == ARMED) printf("  ARMED  |");
			else printf("DISARMED |");
			fflush(stdout);
		}
		usleep(1000000 / PRINTF_HZ);
	}
	return NULL;
} 

/*******************************************************************************
*	on_pause_press() 
*	Disarm the controller and set system state to paused.
*	If the user holds the pause button for 2 seconds, exit cleanly
*******************************************************************************/
int on_pause_press(){
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds
	
	switch(get_state()){
	// pause if running
	case EXITING:
		return 0;
	case RUNNING:
		set_state(PAUSED);
		disarm_controller();
		set_led(RED,1);
		set_led(GREEN,0);
		break;
	case PAUSED:
		set_state(RUNNING);
		disarm_controller();
		set_led(GREEN,1);
		set_led(RED,0);
		break;
	default:
		break;
	}
	
	// now wait to see if the user want to shut down the program
	while(i<samples){
		usleep(us_wait/samples);
		if(get_pause_button() == RELEASED){
			return 0; //user let go before time-out
		}
		i++;
	}
	printf("long press detected, shutting down\n");
	//user held the button down long enough, blink and exit cleanly
	blink_led(RED,5,2);
	set_state(EXITING);
	return 0;
}

/*******************************************************************************
*	on_mode_release()
*	toggle between position and angle modes if MiP is paused
*******************************************************************************/
int on_mode_release(){
	// toggle between position and angle modes
	if(setpoint.drive_mode == NOVICE){
		setpoint.drive_mode = ADVANCED;
		printf("using drive_mode = ADVANCED\n");
	}
	else {
		setpoint.drive_mode = NOVICE;
		printf("using drive_mode = NOVICE\n");
	}
	
	blink_led(GREEN,5,1);
	return 0;
}
