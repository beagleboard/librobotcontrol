/*******************************************************************************
* rc_balance.c
*
* Reference solution for balancing EduMiP
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
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
void balance_controller(); 
// threads
void* setpoint_manager(void* ptr);
void* battery_checker(void* ptr);
void* printf_loop(void* ptr);
// regular functions
int zero_out_controller();
int disarm_controller();
int arm_controller();
int wait_for_starting_condition();
void on_pause_press();
void on_mode_release();
int blink_green();
int blink_red();

/*******************************************************************************
* Global Variables				
*******************************************************************************/
core_state_t cstate;
setpoint_t setpoint;
rc_filter_t D1, D2, D3;
rc_imu_data_t imu_data;

/*******************************************************************************
* main()
*
* Initialize the filters, IMU, threads, & wait until shut down
*******************************************************************************/
int main(){
	// make sure everything initializes first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}

	rc_set_led(RED,1);
	rc_set_led(GREEN,0);
	rc_set_state(UNINITIALIZED);

	// if gyro isn't calibrated, run the calibration routine
	if(!rc_is_gyro_calibrated()){
		printf("Gyro not calibrated, automatically starting calibration routine\n");
		printf("Let your MiP sit still on a firm surface\n");
		rc_calibrate_gyro_routine();
	}

	// make sure setpoint starts at normal values
	setpoint.arm_state = DISARMED;
	setpoint.drive_mode = NOVICE;
	
	D1=rc_empty_filter();
	D2=rc_empty_filter();
	D3=rc_empty_filter();
	
	// set up D1 Theta controller
	float D1_num[] = D1_NUM;
	float D1_den[] = D1_DEN;
	if(rc_alloc_filter_from_arrays(&D1,D1_ORDER, DT, D1_num, D1_den)){
		fprintf(stderr,"ERROR in rc_balance, failed to make filter D1\n");
		return -1;
	}
	D1.gain = D1_GAIN;
	rc_enable_saturation(&D1, -1.0, 1.0);
	rc_enable_soft_start(&D1, SOFT_START_SEC);
	
	// set up D2 Phi controller
	float D2_num[] = D2_NUM;
	float D2_den[] = D2_DEN;
	if(rc_alloc_filter_from_arrays(&D2, D2_ORDER, DT, D2_num, D2_den)){
		fprintf(stderr,"ERROR in rc_balance, failed to make filter D2\n");
		return -1;
	}
	D2.gain = D2_GAIN;
	rc_enable_saturation(&D2, -THETA_REF_MAX, THETA_REF_MAX);
	rc_enable_soft_start(&D2, SOFT_START_SEC);
	
	printf("Inner Loop controller D1:\n");
	rc_print_filter(D1);
	printf("\nOuter Loop controller D2:\n");
	rc_print_filter(D2);
	
	// set up D3 gamma (steering) controller
	if(rc_pid_filter(&D3, D3_KP, D3_KI, D3_KD, 4*DT, DT)){
		fprintf(stderr,"ERROR in rc_balance, failed to make steering controller\n");
		return -1;
	}
	rc_enable_saturation(&D3, -STEERING_INPUT_MAX, STEERING_INPUT_MAX);

	// set up button handlers
	rc_set_pause_pressed_func(&on_pause_press);
	rc_set_mode_released_func(&on_mode_release);
	
	// start a thread to slowly sample battery 
	pthread_t  battery_thread;
	pthread_create(&battery_thread, NULL, battery_checker, (void*) NULL);
	// wait for the battery thread to make the first read
	while(cstate.vBatt==0 && rc_get_state()!=EXITING) rc_usleep(1000);
	
	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	if(isatty(fileno(stdout))){
		pthread_t  printf_thread;
		pthread_create(&printf_thread, NULL, printf_loop, (void*) NULL);
	}
	
	// set up IMU configuration
	rc_imu_config_t imu_config = rc_default_imu_config();
	imu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	imu_config.orientation = ORIENTATION_Y_UP;

	// start imu
	if(rc_initialize_imu_dmp(&imu_data, imu_config)){
		fprintf(stderr,"ERROR: can't talk to IMU, all hope is lost\n");
		rc_blink_led(RED, 5, 5);
		return -1;
	}
	
	// start balance stack to control setpoints
	pthread_t  setpoint_thread;
	pthread_create(&setpoint_thread, NULL, setpoint_manager, (void*) NULL);

	// this should be the last step in initialization 
	// to make sure other setup functions don't interfere
	rc_set_imu_interrupt_func(&balance_controller);
	
	// start in the RUNNING state, pressing the pause button will swap to
	// the PAUSED state then back again.
	printf("\nHold your MIP upright to begin balancing\n");
	rc_set_state(RUNNING);
	
	
	// start dsm listener
	rc_initialize_dsm();
	
	// chill until something exits the program
	while(rc_get_state()!=EXITING){
		rc_usleep(10000);
	}
	
	// cleanup
	rc_free_filter(&D1);
	rc_free_filter(&D2);
	rc_free_filter(&D3);
	rc_power_off_imu();
	rc_cleanup();
	return 0;
}

/*******************************************************************************
* void* setpoint_manager(void* ptr)
*
* This thread is in charge of adjusting the controller setpoint based on user
* inputs from dsm radio control. Also detects pickup to control arming the
* controller.
*******************************************************************************/
void* setpoint_manager(void* ptr){
	float drive_stick, turn_stick; // dsm input sticks

	// wait for IMU to settle
	disarm_controller();
	rc_usleep(2500000);
	rc_set_state(RUNNING);
	rc_set_led(RED,0);
	rc_set_led(GREEN,1);
	
	while(rc_get_state()!=EXITING){
		// sleep at beginning of loop so we can use the 'continue' statement
		rc_usleep(1000000/SETPOINT_MANAGER_HZ); 
		
		// nothing to do if paused, go back to beginning of loop
		if(rc_get_state() != RUNNING) continue;

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
	
		// if dsm is active, update the setpoint rates
		if(rc_is_new_dsm_data()){
			// Read normalized (+-1) inputs from RC radio stick and multiply by 
			// polarity setting so positive stick means positive setpoint
			turn_stick  = rc_get_dsm_ch_normalized(DSM_TURN_CH) * DSM_TURN_POL;
			drive_stick = rc_get_dsm_ch_normalized(DSM_DRIVE_CH)* DSM_DRIVE_POL;
			
			// saturate the inputs to avoid possible erratic behavior
			rc_saturate_float(&drive_stick,-1,1);
			rc_saturate_float(&turn_stick,-1,1);
			
			// use a small deadzone to prevent slow drifts in position
			if(fabs(drive_stick)<DSM_DEAD_ZONE) drive_stick = 0.0;
			if(fabs(turn_stick)<DSM_DEAD_ZONE)  turn_stick  = 0.0;

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
		// if dsm had timed out, put setpoint rates back to 0
		else if(rc_is_dsm_active()==0){
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
* void balance_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*******************************************************************************/
void balance_controller(){
	static int inner_saturation_counter = 0; 
	float dutyL, dutyR;
	/******************************************************************
	* STATE_ESTIMATION
	* read sensors and compute the state when either ARMED or DISARMED
	******************************************************************/
	// angle theta is positive in the direction of forward tip around X axis
	cstate.theta = imu_data.dmp_TaitBryan[TB_PITCH_X] + CAPE_MOUNT_ANGLE; 
	
	// collect encoder positions, right wheel is reversed 
	cstate.wheelAngleR = (rc_get_encoder_pos(ENCODER_CHANNEL_R) * TWO_PI) \
								/(ENCODER_POLARITY_R * GEARBOX * ENCODER_RES);
	cstate.wheelAngleL = (rc_get_encoder_pos(ENCODER_CHANNEL_L) * TWO_PI) \
								/(ENCODER_POLARITY_L * GEARBOX * ENCODER_RES);
	
	// Phi is average wheel rotation also add theta body angle to get absolute 
	// wheel position in global frame since encoders are attached to the body
	cstate.phi = ((cstate.wheelAngleL+cstate.wheelAngleR)/2) + cstate.theta; 
	
	// steering angle gamma estimate 
	cstate.gamma = (cstate.wheelAngleR-cstate.wheelAngleL) \
											* (WHEEL_RADIUS_M/TRACK_WIDTH_M);

	/*************************************************************
	* check for various exit conditions AFTER state estimate
	***************************************************************/
	if(rc_get_state()==EXITING){
		rc_disable_motors();
		return;
	}
	// if controller is still ARMED while state is PAUSED, disarm it
	if(rc_get_state()!=RUNNING && setpoint.arm_state==ARMED){
		disarm_controller();
		return;
	}
	// exit if the controller is disarmed
	if(setpoint.arm_state==DISARMED){
		return;
	}
	
	// check for a tipover
	if(fabs(cstate.theta) > TIP_ANGLE){
		disarm_controller();
		printf("tip detected \n");
		return;
	}
	
	/************************************************************
	* OUTER LOOP PHI controller D2
	* Move the position setpoint based on phi_dot. 
	* Input to the controller is phi error (setpoint-state).
	*************************************************************/
	if(ENABLE_POSITION_HOLD){
		if(setpoint.phi_dot != 0.0) setpoint.phi += setpoint.phi_dot*DT;
		cstate.d2_u = rc_march_filter(&D2,setpoint.phi-cstate.phi);
		setpoint.theta = cstate.d2_u;
	}
	else setpoint.theta = 0.0;
	
	/************************************************************
	* INNER LOOP ANGLE Theta controller D1
	* Input to D1 is theta error (setpoint-state). Then scale the 
	* output u to compensate for changing battery voltage.
	*************************************************************/
	D1.gain = D1_GAIN * V_NOMINAL/cstate.vBatt;
	cstate.d1_u = rc_march_filter(&D1,(setpoint.theta-cstate.theta));
	
	/*************************************************************
	* Check if the inner loop saturated. If it saturates for over
	* a second disarm the controller to prevent stalling motors.
	*************************************************************/
	if(fabs(cstate.d1_u)>0.95) inner_saturation_counter++;
	else inner_saturation_counter = 0; 
 	// if saturate for a second, disarm for safety
	if(inner_saturation_counter > (SAMPLE_RATE_HZ*D1_SATURATION_TIMEOUT)){
		printf("inner loop controller saturated\n");
		disarm_controller();
		inner_saturation_counter = 0;
		return;
	}
	
	/**********************************************************
	* gama (steering) controller D3
	* move the setpoint gamma based on user input like phi
	***********************************************************/
	if(fabs(setpoint.gamma_dot)>0.0001) setpoint.gamma += setpoint.gamma_dot * DT;
	cstate.d3_u = rc_march_filter(&D3,setpoint.gamma - cstate.gamma);
	
	/**********************************************************
	* Send signal to motors
	* add D1 balance control u and D3 steering control also 
	* multiply by polarity to make sure direction is correct.
	***********************************************************/
	dutyL = cstate.d1_u - cstate.d3_u;
	dutyR = cstate.d1_u + cstate.d3_u;	
	rc_set_motor(MOTOR_CHANNEL_L, MOTOR_POLARITY_L * dutyL); 
	rc_set_motor(MOTOR_CHANNEL_R, MOTOR_POLARITY_R * dutyR); 

	return;
}

/*******************************************************************************
* 	zero_out_controller()
*
*	Clear the controller's memory and zero out setpoints.
*******************************************************************************/
int zero_out_controller(){
	rc_reset_filter(&D1);
	rc_reset_filter(&D2);
	rc_reset_filter(&D3);
	setpoint.theta = 0.0f;
	setpoint.phi   = 0.0f;
	setpoint.gamma = 0.0f;
	rc_set_motor_all(0.0f);
	return 0;
}

/*******************************************************************************
* disarm_controller()
*
* disable motors & set the setpoint.core_mode to DISARMED
*******************************************************************************/
int disarm_controller(){
	rc_disable_motors();
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
	rc_set_encoder_pos(ENCODER_CHANNEL_L,0);
	rc_set_encoder_pos(ENCODER_CHANNEL_R,0);
	// prefill_filter_inputs(&D1,cstate.theta); 
	setpoint.arm_state = ARMED;
	rc_enable_motors();
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

	// wait for MiP to be tipped back or forward first
	// exit if state becomes paused or exiting
	while(rc_get_state()==RUNNING){
		// if within range, start counting
		if(fabs(cstate.theta) > START_ANGLE) checks++;
		// fell out of range, restart counter
		else checks = 0;
		// waited long enough, return
		if(checks >= checks_needed) break;
		rc_usleep(wait_us);
	}
	// now wait for MiP to be upright
	checks = 0;
	// exit if state becomes paused or exiting
	while(rc_get_state()==RUNNING){
		// if within range, start counting
		if(fabs(cstate.theta) < START_ANGLE) checks++;
		// fell out of range, restart counter
		else checks = 0;
		// waited long enough, return
		if(checks >= checks_needed) return 0;
		rc_usleep(wait_us);
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
	while(rc_get_state()!=EXITING){
		new_v = rc_battery_voltage();
		// if the value doesn't make sense, use nominal voltage
		if (new_v>9.0 || new_v<5.0) new_v = V_NOMINAL;
		cstate.vBatt = new_v;
		rc_usleep(1000000 / BATTERY_CHECK_HZ);
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
	rc_state_t last_rc_state, new_rc_state; // keep track of last state 
	last_rc_state = rc_get_state();
	while(rc_get_state()!=EXITING){
		new_rc_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_rc_state==RUNNING && last_rc_state!=RUNNING){
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
		else if(new_rc_state==PAUSED && last_rc_state!=PAUSED){
			printf("\nPAUSED: press pause again to start.\n");
		}
		last_rc_state = new_rc_state;
		
		// decide what to print or exit
		if(new_rc_state == RUNNING){	
			printf("\r");
			printf("%7.3f  |", cstate.theta);
			printf("%7.3f  |", setpoint.theta);
			printf("%7.3f  |", cstate.phi);
			printf("%7.3f  |", setpoint.phi);
			printf("%7.3f  |", cstate.gamma);
			printf("%7.3f  |", cstate.d1_u);
			printf("%7.3f  |", cstate.d3_u);
			printf("%7.3f  |", cstate.vBatt);
			
			if(setpoint.arm_state == ARMED) printf("  ARMED  |");
			else printf("DISARMED |");
			fflush(stdout);
		}
		rc_usleep(1000000 / PRINTF_HZ);
	}
	return NULL;
} 

/*******************************************************************************
* void on_pause_press() 
*
* Disarm the controller and set system state to paused.
* If the user holds the pause button for 2 seconds, exit cleanly
*******************************************************************************/
void on_pause_press(){
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds
	
	switch(rc_get_state()){
	// pause if running
	case EXITING:
		return;
	case RUNNING:
		rc_set_state(PAUSED);
		disarm_controller();
		rc_set_led(RED,1);
		rc_set_led(GREEN,0);
		break;
	case PAUSED:
		rc_set_state(RUNNING);
		disarm_controller();
		rc_set_led(GREEN,1);
		rc_set_led(RED,0);
		break;
	default:
		break;
	}
	
	// now wait to see if the user want to shut down the program
	while(i<samples){
		rc_usleep(us_wait/samples);
		if(rc_get_pause_button() == RELEASED){
			return; //user let go before time-out
		}
		i++;
	}
	printf("long press detected, shutting down\n");
	//user held the button down long enough, blink and exit cleanly
	rc_blink_led(RED,5,2);
	rc_set_state(EXITING);
	return;
}

/*******************************************************************************
* void on_mode_release()
*
* toggle between position and angle modes if MiP is paused
*******************************************************************************/
void on_mode_release(){
	// toggle between position and angle modes
	if(setpoint.drive_mode == NOVICE){
		setpoint.drive_mode = ADVANCED;
		printf("using drive_mode = ADVANCED\n");
	}
	else {
		setpoint.drive_mode = NOVICE;
		printf("using drive_mode = NOVICE\n");
	}
	
	rc_blink_led(GREEN,5,1);
	return;
}
