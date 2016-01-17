
/*
Copyright (c) 2014, James Strawson
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*/

//	fly.c
//	see README.txt for description and use


/********************************************
* 			Includes & Constants			*
*********************************************/
#include <robotics_cape.h>

#define SAMPLE_RATE_HZ 		200		// Run the main control loop at this rate
#define DT 				   .005		// timestep seconds MUST MATCH SAMPLE_RATE_HZ

// local function definitions
#include "filter_lib.h"				// for discrete filters and controllers
#include "fly_config.h"				// for loading and saving settings
#include "mixing_matrices.h"
//#include "fly_logger.h"				// for logging control loop data

// Flight Core Constants
#define MAX_YAW_COMPONENT	0.21 	// Max control delta the controller can apply
#define MAX_THRUST_COMPONENT 0.8	// upper limit of net thrust input
#define MAX_ROLL_COMPONENT	0.2 	// Max control delta the controller can apply
#define MAX_PITCH_COMPONENT	0.2 	// Max control delta the controller can apply
#define INT_CUTOFF_TH 		0.3		// prevent integrators from running unless flying
#define YAW_CUTOFF_TH 		0.1		// prevent yaw from changing when grounded
#define ARM_TIP_THRESHOLD	0.2		// radians from level to allow arming sequence 
#define LAND_SATURATION 	0.05	// saturation of roll, yaw, pitch controllers
									// while landed


/************************************************************************
*	Type Definitions 
*************************************************************************/

/************************************************************************
* 	flight_mode_t
*	
*	user_interface.flight_mode determines how the flight stack behaves
*
*	EMERGENCY_LAND: slowly decrease altitude in place until touchdown
*	
*	USER_ATTITUDE: gives the user direct joystick control of the inner-loop
*	throttle, yaw rate, and roll/pitch attitude
*
*
*	TODO: future modes
*
*	LOITER: sets the flight_core to position mode and updates the position
*	setpoint based on user inputs such that the user joystick controls 
*	velocity from the perspective of the UAV. This would be the most_useful
*	mode when flying First-Person view.
*	
*	USER_POSITION_CARTESIAN: Similar to jog mode on a CNC mill. The user 
*	controls the global position setpoint using the ARMING location as
*	the origin with positive Y facing forward and X to the right
*
*	USER_POSITION_RADIAL: Left/Right forward/back are from the perspective 
*	of the pilot at takeoff location.
*
************************************************************************/
typedef enum flight_mode_t{
	EMERGENCY_LAND,
	USER_ATTITUDE,
	// USER_LOITER,
	// USER_POSITION_CARTESIAN,
	// USER_POSITION_RADIAL,
	// TARGET_HOLD,
}flight_mode_t;

/************************************************************************
* 	core_mode_t
*
*	ATTITUDE: The controller will read throttle, roll, pitch, and yaw_rate
*	setpoints so the user has direct control of inner attitude control loop.
*	The yaw controller will still hold an absolute position but the 
*	yaw_setpoint will be updated by the flight_core based on the yaw_rate
*	setpoint.
*
*	POSITION: The controller will instead read the absolute global position
*	inside setpoint and modulate attitude itself to maintain position 
*	via successive loop closure. This means the continuously changing 
*	attitude setpoint can be read back by other threads.
************************************************************************/
typedef enum core_mode_t{
	ATTITUDE,
	POSITION,
}core_mode_t;

/************************************************************************
* 	arm_state_t
*
*	ARMED or DISARMED to indicate if the controller is running
************************************************************************/
typedef enum arm_state_t{
	DISARMED,
	ARMED
}arm_state_t;

/************************************************************************
* 	setpoint_t
*	setpoint for the flight_core attitude controller
*	This is controlled by the flight stack and read by the flight core	
************************************************************************/
typedef struct setpoint_t{
	
	core_mode_t core_mode;	// see core_state_tdeclaration
	arm_state_t arm_state;	// see arm_state_t declaration
	
	// attitude setpoint
	float throttle;			// desired upward motor thrust
	float roll;				// roll angle (rad)
	float pitch;			// pitch angle (rad)
	float yaw_rate;			// yaw_rate in rad/s
	
	// Cartesian position setpoint from arming location (m)
	float altitude;			// altitude 	
	float position_X;		// horizontal displacement since arming
	float position_Y;		// forward/back displacement since arming
	float yaw;				// yaw angle displacement since arming
}setpoint_t;

/************************************************************************
* 	core_state_t
*	contains most recent values reported by the flight_core
*	Should only be written to by the flight core after initialization		
************************************************************************/
typedef struct core_state_t{
	uint64_t core_start_time_us;	// time when IMU interrupt routine started
	uint64_t time_us; 				// last time controller has finished a step
	float altitude;					// altitude estimate (m)
	float roll;						// current roll angle (rad)
	float pitch;					// current pitch angle (rad)
	float yaw;						// current yaw angle (rad)
	float last_yaw;					// previous value for crossover detection
	
	float dAltitude;				// first derivative of altitude (m/s)
	float dRoll;					// first derivative of roll (rad/s)
	float dPitch;					// first derivative of pitch (rad/s)
	float dYaw;						// first derivative of yaw (rad/s)
	
	float vBatt;					// main battery pack voltage
	float positionX;				// estimate of X displacement from takeoff (m)
	float positionY;				// estimate of Y displacement from takeoff (m)
	
	float alt_err; 					// current and previous altitudes error
	float dRoll_err; 				// current and previous roll error
	float dPitch_err;				// current pitch error
	float yaw_err;  			 	// current  yaw error
	
	discrete_filter roll_ctrl;		// feedback controller for angular velocity
	discrete_filter pitch_ctrl;		// feedback controller for angular velocity
	discrete_filter yaw_ctrl;		// feedback controller for DMP yaw
	
	float alt_err_integrator; 		// current and previous altitudes error
	float dRoll_err_integrator; 	// current and previous roll error
	float dPitch_err_integrator;	// current and previous pitch error
	float imu_roll_err;
	float imu_pitch_err;
	float yaw_err_integrator;   	// current and previous yaw error
	float control_u[4];				// control outputs  alt,roll,pitch,yaw
	float esc_out[8];				// normalized (0-1) outputs to motors
	int num_yaw_spins; 				// remember number of spins around Z
	float imu_yaw_on_takeoff;		// raw yaw value read on takeoff
}core_state_t;

/************************************************************************
* 	input_mode_t
*	possible modes of user control
*	these are ordered such that DSM2 has highest priority
************************************************************************/
typedef enum input_mode_t {
	NONE,
	MAVLINK,
	BLUETOOTH,
	DSM2
}input_mode_t;

/************************************************************************
* 	user_interface_t
*	represents current command by the user which may be populated from 
*	DSM2, mavlink, or any other communication.
************************************************************************/
typedef struct user_interface_t{
	// written by the last input watching thread to update
	input_mode_t input_mode;
	
	// this is the user commanded flight_mode. 
	// flight stack reads this into flight_mode except in the
	// case of loss of communication or emergency landing
	flight_mode_t flight_mode;  
	
	// All sticks scaled from -1 to 1
	float throttle_stick; 	// positive up
	float yaw_stick;		// positive to the right, CW yaw
	float roll_stick;		// positive to the right
	float pitch_stick;		// positive up
	
	arm_state_t kill_switch;

}user_interface_t;


/************************************************************************
* 	Function declarations				
************************************************************************/
// hardware interrupt routines
int flight_core();

//threads
void* flight_stack(void* ptr);
void* battery_checker(void* ptr);
void* printf_loop(void* ptr);
void* dsm2_watcher(void* ptr);
// void* mavlink_listener(void* ptr);
// void* mavlink_sender(void* ptr);
void* printf_loop(void* ptr);

// regular functions
int zero_out_controller();
int initialize_controllers();
int initialize_core();
int disarm_controller();
int arm_controller();
int saturate_number(float* val, float min, float max);
int wait_for_arming_sequence();
int on_pause_press();
int on_mode_release();
int blink_green();
int blink_red();
int print_flight_mode(flight_mode_t mode);


/************************************************************************
* 	Global Variables				
************************************************************************/
fly_config_t 			config;
setpoint_t 				setpoint;
core_state_t			cstate;
user_interface_t		user_interface;
// mavlink socket and socket address
int* udp_sock;
struct sockaddr_in gcAddr;

/***********************************************************************
*	main()
*	initialize the IMU, start all the threads, and wait still something 
*	triggers a shut down
***********************************************************************/
int main(int argc, char* argv[]){
	// initialize cape hardware
	if(initialize_cape()<0){
		blink_red();
		return -1;
	}
	set_led(RED,HIGH);
	set_led(GREEN,LOW);
	set_state(UNINITIALIZED);
	
	// set up button handlers first
	// so user can exit by holding pause
	set_pause_pressed_func(&on_pause_press);
	set_mode_unpressed_func(&on_mode_release);
	
	// load data from disk.
	if(load_config(&config)==-1){
		printf("aborting, config file error\n");
		return -1;
	}
	
	// start a thread to slowly sample battery 
	pthread_t  battery_thread;
	pthread_create(&battery_thread, NULL, battery_checker, (void*) NULL);
	
	
	// start listening for RC control from dsm2 radio
	if(config.enable_dsm2){
		if(initialize_dsm2()<0){
				printf("failed to start DSM2\n");
		}
		else{
			pthread_t  dsm2_thread;
			pthread_create(&dsm2_thread, NULL, dsm2_watcher, (void*) NULL);
		}
	}
	
	// // start logging thread if enabled
	// if(config.enable_logging){
		// if(start_log(SAMPLE_RATE_HZ, &cstate.time_us)<0){
			// printf("failed to start log\n");
		// }
		// else{
			// // start new thread to write the file occationally
			// pthread_t  logging_thread;
			// pthread_create(&logging_thread, NULL, log_writer, (void*) NULL);
		// }
	// }
	
	// // first check for user options
	// if(parse_arguments(argc, argv)<0){
		// return -1;
	// }
	
	
	// // start logging thread if enabled
	// if(config.enable_logging){
		// if(start_log(SAMPLE_RATE_HZ, &cstate.time_us)<0){
			// printf("failed to start log\n");
		// }
		// else{
			// // start new thread to write the file occationally
			// pthread_t  logging_thread;
			// pthread_create(&logging_thread, NULL, log_writer, (void*) NULL);
		// }
	// }
	
	// // Start Safety checking thread
	// pthread_create(&safety_thread, NULL, safety_thread_func, (void*) NULL);
	
	
	// Finally start the real-time interrupt driven control thread
	// start IMU with equilibrium set with upright orientation 
	// for MiP with Ethernet pointing relatively up
	signed char orientation[9] = ORIENTATION_FLAT; 
	if(initialize_imu(SAMPLE_RATE_HZ, orientation)){
		// can't talk to IMU, all hope is lost
		// blink red until the user exits
		printf("IMU initialization failed, please reboot\n");
		blink_red();
		cleanup_cape();
		return -1;
	}
	// assigning the interrupt function and stack
	// should be the last step in initialization 
	// to make sure other setup functions don't interfere
	printf("starting core IMU interrupt\n");
	cstate.core_start_time_us = microsSinceEpoch();
	set_imu_interrupt_func(&flight_core);
	// start flight stack to control setpoints
	// this thread is in charge of arming and managing the core
	pthread_t  flight_stack_thread;
	pthread_create(&flight_stack_thread, NULL, flight_stack, (void*) NULL);
	
	printf("\nReady for arming sequence\n");
	set_state(RUNNING);
	
	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	if(isatty(fileno(stdout))){
		pthread_t  printf_thread;
		pthread_create(&printf_thread, NULL, printf_loop, (void*) NULL);
	}
	
	//chill until something exits the program
	while(get_state()!=EXITING){
		usleep(100000);
	}
	
	// cleanup before closing
	//close(sock); 	// mavlink UDP socket
	cleanup_cape();	// de-initialize cape hardware
	return 0;
}

/************************************************************************
*	flight_stack()
*	Translates the flight mode and user controls from user_interface
*	into setpoints for the flight_core position and attitude controller
*
*	If the core gets disarmed by another thread, flight_stack manages
*	recognizing the rearming sequence
*
*	The flight_core only takes setpoint values for feedback control, 
************************************************************************/
void* flight_stack(void* ptr){
	int i;
	//flight_mode_t previous_flight_mode;
	
	// wait for IMU to settle
	disarm_controller();
	usleep(1000000);
	usleep(1000000);
	usleep(500000);
	set_state(RUNNING);
	setpoint.core_mode = POSITION; //start in position control
	set_led(RED,LOW);
	set_led(GREEN,HIGH);
	
	// run until state indicates thread should close
	while(1){
		switch (get_state()){
		case EXITING:
			return NULL;
			
		case PAUSED:
			// not much to do if paused!
			break;
		
		case RUNNING:
			// if the core got disarmed, wait for arming sequence 
			if(setpoint.arm_state == DISARMED){
				wait_for_arming_sequence();
				// user may have pressed the pause button or shut down while waiting
				// check before continuing
				if(get_state()!=RUNNING){
					break;
				}
				// read config each time it's picked up to recognize new
				// settings user may have changed
				// only actually reads from disk if the file was modified
				load_config(&config);
				initialize_controllers();
				// // write a blank log entry to mark this time
				// log_blank_entry();
				
				// wake ESCs up at minimum throttle to avoid calibration mode
				// flight_core also sends one minimum pulse at first when armed
				for(i=0; i<config.rotors; i++){
					send_servo_pulse_normalized(i+1,0);
				}
				usleep(10000);
				arm_controller();
			}
			
			// kill switches seem to be fine
			// switch behaviour based on user flight mode
			if(setpoint.arm_state == ARMED){
				// shutdown core on kill switch
				if(user_interface.kill_switch == DISARMED){
					//printf("stack disarmed controller\n");
					disarm_controller();
					break;
				}
				if(user_interface.input_mode==NONE){
					// no input devices connected, keep things zero'd
					disarm_controller();
					break;				
					}
				else{
				// decide how to change setpoint based on flight mode
					switch(user_interface.flight_mode){
					// Raw attitude mode lets user control the inner attitude loop directly
					case USER_ATTITUDE:
						setpoint.core_mode = ATTITUDE;
						
						// translate throttle stick (-1,1) to throttle (0,1)
						setpoint.throttle = (user_interface.throttle_stick + 1.0)/2.0;
						
						// scale roll and pitch angle by max setpoint in rad
						setpoint.roll		= user_interface.roll_stick *
													config.max_roll_setpoint;
						setpoint.pitch		= user_interface.pitch_stick *
													config.max_pitch_setpoint;
						// scale yaw_rate by max yaw rate in rad/s
						setpoint.yaw_rate	= user_interface.yaw_stick *
													config.max_yaw_rate;
						break;
						
					// emergency land just sets the throttle low for now
					// TODO: gently lower altitude till landing detected 
					case EMERGENCY_LAND:
						setpoint.core_mode = ATTITUDE;
						setpoint.throttle  = config.land_throttle;
						setpoint.roll		= 0;
						setpoint.pitch		= 0;
						setpoint.yaw_rate	= 0;
						break;
					
					// TODO: other modes
					// case USER_LOITER:
						// break;
					// case USER_POSITION_CARTESIAN:
						// break;
					// case USER_POSITION_RADIAL:
						// break; 
					// case TARGET_HOLD:
						// break;
					default:
						break;
					}
				}
			}
		
		default:
			break;
		}// end of switch(get_state())
		
		// // record previous flight mode to detect changes
		// previous_flight_mode = user_interface.flight_mode; 
		// run about as fast as the core itself 
		usleep(1000000/SAMPLE_RATE_HZ); 
	}
	return NULL;
}

/************************************************************************
*	flight_core()
*	Hardware Interrupt-Driven Flight Control Loop
*	- read sensor values
*	- estimate system state
*	- read setpoint from flight_stack
*	- if is position mode, calculate a new attitude setpoint
* 	- otherwise use user attitude setpoint
*	- calculate and send ESC commands
************************************************************************/
int flight_core(){
	// remember previous arm mode to detect transition from DISARMED
	static arm_state_t previous_arm_state;
	int i;	// general purpose
	static float new_esc[8];
	static float u[4];	// normalized roll, pitch, yaw, throttle, components 
	
	/************************************************************************
	*	Begin control loop if there was a valid interrupt with new IMU data
	************************************************************************/
	if (mpu9150_read(&mpu) != 0){
		return -1;
	}
	
	/***********************************************************************
	*	STATE_ESTIMATION
	*	read sensors and compute the state regardless of if the controller
	*	is ARMED or DISARMED
	************************************************************************/
	// collect new IMU roll/pitch data
	// positive roll right according to right hand rule
	// MPU9150 driver has incorrect minus sign on Y axis, correct for it here
	// positive pitch backwards according to right hand rule
	cstate.roll  = -(mpu.fusedEuler[VEC3_Y] - cstate.imu_roll_err);
	cstate.pitch =   mpu.fusedEuler[VEC3_X] - cstate.imu_pitch_err;

	
	// current roll/pitch/yaw rates straight from gyro 
	// converted to rad/s with default FUll scale range
	// raw gyro matches sign on MPU9150 coordinate system, unlike Euler angle
	cstate.dRoll  = mpu.rawGyro[VEC3_Y] * GYRO_FSR * DEGREE_TO_RAD / 32767.0;
	cstate.dPitch = mpu.rawGyro[VEC3_X] * GYRO_FSR * DEGREE_TO_RAD / 32767.0;
	cstate.dYaw	  = mpu.rawGyro[VEC3_Z] * GYRO_FSR * DEGREE_TO_RAD / 32767.0;
	

		
	// if this is the first loop since being armed, reset yaw trim
	if(previous_arm_state==DISARMED && setpoint.arm_state!=DISARMED){	
		cstate.imu_yaw_on_takeoff = mpu.fusedEuler[VEC3_Z];
		cstate.num_yaw_spins = 0;
	}
	
	
	float new_yaw = -(mpu.fusedEuler[VEC3_Z] - cstate.imu_yaw_on_takeoff) \
									+ (cstate.num_yaw_spins*2*PI);
	
	// detect the crossover point at Z = +-PI
	if(new_yaw - cstate.last_yaw > 6){
		cstate.num_yaw_spins -= 1;
	}
	else if(new_yaw - cstate.last_yaw < -6){
		cstate.num_yaw_spins += 1;
	}
	
	// also reset yaw if the throttle stick is down to prevent drift while on the ground	
	if (user_interface.throttle_stick < -0.95){
		cstate.imu_yaw_on_takeoff = mpu.fusedEuler[VEC3_Z];
		cstate.num_yaw_spins=0;
	}
	
	// record new yaw compensating for full rotation
	cstate.last_yaw = cstate.yaw;
	cstate.yaw = -(mpu.fusedEuler[VEC3_Z] - cstate.imu_yaw_on_takeoff) +
											(cstate.num_yaw_spins*2*PI);
		
		
	/***********************************************************************
	*	Control based on the robotics_library defined state variable
	*	PAUSED: make sure the controller stays DISARMED
	*	RUNNING: Normal operation of controller.
	*		- check for tipover
	*		- choose mode from setpoint.core_mode
	*		- evaluate difference equation and check saturation
	*		- actuate motors
	************************************************************************/
	switch (get_state()){
	
	// make sure things are off if program is closing
	case EXITING:
		return 0;
	
	// if the controller is somehow still ARMED, disarm it
	case PAUSED:
		if(setpoint.arm_state==ARMED){
			disarm_controller();
		}
		break;
	
	// normal operating mode
	case RUNNING:
		// exit if the controller was not armed properly
		if(setpoint.arm_state==DISARMED){
			/************************************************************************
			*	if disarmed, reset controllers and return
			************************************************************************/
			zero_out_controller();
			goto END;
		}

		// check for a tipover
		if (fabs(cstate.roll) >config.tip_angle ||
			fabs(cstate.pitch)>config.tip_angle)
		{
			disarm_controller();
			printf("\n TIPOVER DETECTED \n");
			goto END;
		}
		
		/************************************************************************
		* 	manage the setpoints based on attitude or position mode
		************************************************************************/
		switch(setpoint.core_mode){
	
		/************************************************************************
		*	in Position control mode, evaluate an outer loop controller to
		*	change the attitude setpoint. Discard user attitude setpoints
		************************************************************************/
		case POSITION:
			// TODO: outer loop position controller
			break;
			
		/************************************************************************
		*	in attitude control mode, user has direct control over throttle
		*	roll, and pitch angles. Absolute yaw setpoint gets updated at
		*	user-commanded yaw_rate
		************************************************************************/
		case ATTITUDE:
			// only when flying, update the yaw setpoint
			if(setpoint.throttle > YAW_CUTOFF_TH){
				setpoint.yaw += DT*setpoint.yaw_rate;
			}
			else{
				setpoint.yaw = cstate.yaw;
			}
			
			break;
			
		default:
			break;		//should never get here
		}
		
		
		/************************************************************************
		* 	Finally run the attitude feedback controllers
		************************************************************************/
		
		/************************************************************************
		*	Roll & Pitch Controllers
		************************************************************************/
		float dRoll_setpoint = (setpoint.roll - cstate.roll) *
														config.roll_rate_per_rad;
		float dPitch_setpoint = (setpoint.pitch - cstate.pitch) *
														config.pitch_rate_per_rad;
		cstate.dRoll_err  = dRoll_setpoint  - cstate.dRoll;
		cstate.dPitch_err = dPitch_setpoint - cstate.dPitch;
		
		// // if last state was DISARMED, then errors will all be 0.
		// // make the previous error the same
		// if(previous_core_mode == DISARMED){
			// preFillFilter(&cstate.roll_ctrl, cstate.dRoll_err);
			// preFillFilter(&cstate.pitch_ctrl, cstate.dPitch_err);
		// }
		
		// only run integrator if airborne 
		// TODO: proper landing/takeoff detection
		if(u[3] > INT_CUTOFF_TH){
			cstate.dRoll_err_integrator  += cstate.dRoll_err  * DT;
			cstate.dPitch_err_integrator += cstate.dPitch_err * DT;
		}
		
				
		marchFilter(&cstate.roll_ctrl, cstate.dRoll_err);
		marchFilter(&cstate.pitch_ctrl, cstate.dPitch_err);
		
		if(setpoint.throttle<0.1){
			saturateFilter(&cstate.roll_ctrl, -LAND_SATURATION,LAND_SATURATION);
			saturateFilter(&cstate.pitch_ctrl, -LAND_SATURATION, LAND_SATURATION);
		}
		else{
			saturateFilter(&cstate.roll_ctrl, -MAX_ROLL_COMPONENT, MAX_ROLL_COMPONENT);
			saturateFilter(&cstate.pitch_ctrl, -MAX_PITCH_COMPONENT, MAX_PITCH_COMPONENT);
		}
		
		u[0] = cstate.roll_ctrl.current_output;
		u[1] = cstate.pitch_ctrl.current_output;
		
		
		/************************************************************************
		*	Yaw Controller
		************************************************************************/
		cstate.yaw_err = setpoint.yaw - cstate.yaw;
		
		// only run integrator if airborne 
		if(u[3] > INT_CUTOFF_TH){
			cstate.yaw_err_integrator += cstate.yaw_err * DT;
		}

		marchFilter(&cstate.yaw_ctrl, cstate.yaw_err);
		
		if (user_interface.throttle_stick < -0.95){
			zeroFilter(&cstate.yaw_ctrl);
		}
		else{
			saturateFilter(&cstate.yaw_ctrl, -MAX_YAW_COMPONENT, MAX_YAW_COMPONENT);
		}
		u[2] = cstate.yaw_ctrl.current_output;
		
		/************************************************************************
		*	Throttle Controller
		************************************************************************/
		// compensate for roll/pitch angle to maintain Z thrust
		float throttle_compensation;
		throttle_compensation = 1 / cos(cstate.roll);
		throttle_compensation *= 1 / cos(cstate.pitch);
		float thr = setpoint.throttle*(MAX_THRUST_COMPONENT-config.idle_speed)
						+ config.idle_speed;
		
		u[3] = throttle_compensation * thr;
		// saturate thrust component now, this will help prevent saturation
		// later once r,p,y are added
		saturate_number(&u[3], 0, MAX_THRUST_COMPONENT);
		
		/************************************************************************
		*  see mixing_matrixes.h for how the mixer works
		************************************************************************/
		if(mix_controls(u[0],u[1],u[2],u[3],&new_esc[0],config.rotors)<0){
			printf("ERROR, mixing failed\n");
			return -1;
		}
		
		/************************************************************************
		*	Prevent saturation under heavy vertical acceleration by reducing all
		*	outputs evenly such that the largest doesn't exceed 1
		************************************************************************/
		// find control output limits 
		float largest_value = 0;
		float smallest_value = 1;
		for(i=0;i<config.rotors;i++){
			if(new_esc[i]>largest_value){
				largest_value = new_esc[i];

			}
			if(new_esc[i]<smallest_value){
				smallest_value=new_esc[i];
			}
		}
		// if upper saturation would have occurred, reduce all outputs evenly
		if(largest_value>1){
			for(i=0;i<config.rotors;i++){
			float offset = largest_value - 1;
				new_esc[i]-=offset;
			}
		}
			
		/************************************************************************
		*	Send a servo pulse immediately at the end of the control loop.
		*	Intended to update ESCs exactly once per control timestep
		*	also record this action to cstate.new_esc_out[] for telemetry
		************************************************************************/
		
		// if this is the first time armed, make sure to send minimum 
		// pulse width to prevent ESCs from going into calibration
		
		if(previous_arm_state == DISARMED){
			for(i=0;i<config.rotors;i++){
				send_servo_pulse_normalized(i+1,0);
			}
		}
		else{
			for(i=0;i<config.rotors;i++){
				if(new_esc[i]>1.0){
					new_esc[i]=1.0;
				}
				else if(new_esc[i]<0){
					new_esc[i]=0;
				}
				send_servo_pulse_normalized(i+1,new_esc[i]);
				cstate.esc_out[i] = new_esc[i];
				cstate.control_u[i] = u[i];		
			}
		}	
		
		// // pass new information to the log with add_to_buffer
		// // this only puts information in memory, doesn't
		// // write to disk immediately
		// if(config.enable_logging){
			// new_log_entry.roll		= cstate.roll;
			// new_log_entry.pitch		= cstate.pitch;
			// new_log_entry.yaw		= cstate.yaw;
			// new_log_entry.u_0		= cstate.u[0];
			// new_log_entry.u_1		= cstate.u[1];
			// new_log_entry.u_2		= cstate.u[2];
			// new_log_entry.u_3		= cstate.u[3];
			// new_log_entry.esc_1		= cstate.esc_out[0];
			// new_log_entry.esc_2		= cstate.esc_out[1];
			// new_log_entry.esc_3		= cstate.esc_out[2];
			// new_log_entry.esc_4		= cstate.esc_out[3];
			// new_log_entry.v_batt	= cstate.v_batt;
			// add_to_buffer(new_log_entry);
		// }
		break;
		
	default:
		break;
	} // end of switch(get_state())
		
	END: // allow quick jump to here to clean up the several switch statements

	//remember the last state to detect transition from DISARMED to ARMED
	previous_arm_state = setpoint.arm_state;
	return 0;
}

/************************************************************************
* 	zero_out_controller()
*	clear the controller state and setpoint
*	especially should be called before swapping state to RUNNING
************************************************************************/
int zero_out_controller(){
	zeroFilter(&cstate.roll_ctrl);
	zeroFilter(&cstate.pitch_ctrl);
	zeroFilter(&cstate.yaw_ctrl);
	setpoint.roll = 0;
	setpoint.pitch = 0;
	setpoint.yaw_rate = 0;
	setpoint.throttle = 0;
	cstate.dRoll_err_integrator = 0;
	cstate.dPitch_err_integrator = 0;
	cstate.alt_err_integrator = 0;
	cstate.yaw_err_integrator = 0;
	
	return 0;
}

/************************************************************************
*	initialize_controllers()
*	setup of feedback controllers used in flight core
************************************************************************/
int initialize_controllers(){
	cstate.roll_ctrl = generatePID(
							config.Droll_KP,
							config.Droll_KI,
							config.Droll_KD,
							.015,
							DT);
							
	cstate.pitch_ctrl = generatePID(
							config.Dpitch_KP,
							config.Dpitch_KI,
							config.Dpitch_KD,
							.015,
							DT);
	cstate.yaw_ctrl = generatePID(
							config.yaw_KP,
							config.yaw_KI,
							config.yaw_KD,
							.015,
							DT);
	zero_out_controller();
	if(parse_mix_layout(config.rotors, config.rotor_layout)){
		return -1;
	}
	return 0;
}



/************************************************************************
*	wait_for_arming_sequence()
*	
*	blocking_function that returns after the user has released the
*	kill_switch and toggled the throttle stick up and down
************************************************************************/
int wait_for_arming_sequence(){
START:
	// wait for level MAV before starting
	while(fabs(cstate.roll)>ARM_TIP_THRESHOLD ||
		fabs(cstate.pitch)>ARM_TIP_THRESHOLD){
		usleep(100000);
		if(get_state()==EXITING)return 0;
	} 	  
	
	while(user_interface.kill_switch==DISARMED){ 
		usleep(100000);
		if(get_state()==EXITING)return 0;
	}
 	
	//wait for throttle down
	while(user_interface.throttle_stick > -0.9){ 
		usleep(100000);
		if(get_state()==EXITING)return 0;}
	
	//wait for throttle up
	while(user_interface.throttle_stick<.9){ 
		usleep(100000);
		if(get_state()==EXITING)return 0;}
	
	//wait for throttle down
	while(user_interface.throttle_stick > -0.9){ 
		usleep(100000);
		if(get_state()==EXITING)return 0;
	}
	
	if(user_interface.kill_switch==DISARMED){
		goto START;
	}
	
	if(fabs(cstate.roll)>ARM_TIP_THRESHOLD ||
		fabs(cstate.pitch)>ARM_TIP_THRESHOLD){
		printf("\nRestart arming sequence with level MAV\n");
		goto START;
	} 
	
	return 0;
}

/************************************************************************
*	disarm_controller()
*	
*	emergency disarm mode
************************************************************************/
int disarm_controller(){
	setpoint.arm_state = DISARMED;
	set_led(RED,LOW);
	set_led(GREEN,HIGH); 
	return 0;
}

/************************************************************************
* 	arm_controller()
*		- zero out the controller
*		- set the setpoint.armed_state to ARMED
*		- enable motors
************************************************************************/
int arm_controller(){
	zero_out_controller();
	setpoint.arm_state = ARMED;
	set_led(GREEN,HIGH);
	set_led(RED,HIGH);
	return 0;
}

/***********************************************************************
*	saturate_num(float val, float limit)
*	bounds val to +- limit
*	return one if saturation occurred, otherwise zero
************************************************************************/
int saturate_number(float* val, float min, float max){
	if(*val>max){
		*val = max;
		return 1;
	}
	else if(*val<min){	
		*val = min;
		return 1;
	}
	return 0;
}


/************************************************************************
*	on_pause_press
*	If the user holds the pause button for a second, exit cleanly
*	disarm on momentary press
************************************************************************/
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
		set_led(RED,HIGH);
		set_led(GREEN,LOW);
		break;
	case PAUSED:
		set_state(RUNNING);
		disarm_controller();
		set_led(GREEN,HIGH);
		set_led(RED,LOW);
		break;
	default:
		break;
	}
	
	// now wait to see if the user wants to shut down the program
	while(i<samples){
		usleep(us_wait/samples);
		if(get_pause_button_state() == UNPRESSED){
			return 0; //user let go before time-out
		}
		i++;
	}
	printf("long press detected, shutting down\n");
	//user held the button down long enough, blink and exit cleanly
	blink_red();
	set_state(EXITING);
	return 0;
}

/***********************************************************************
*	on_mode_release()
*	placeholder for now, just blink led
***********************************************************************/
int on_mode_release(){
	blink_green();
	return 0;
}

/***********************************************************************
*	battery_checker()
*	super slow loop checking battery voltage
************************************************************************/
void* battery_checker(void* ptr){
	float new_v;
	while(get_state()!=EXITING){
		new_v = get_battery_voltage();
		// check if there is a bad reading
		if (new_v>9.0 || new_v<5.0){
			// printf("problem reading battery\n");
			// use nominal for now
			new_v = config.v_nominal;
		}
		cstate.vBatt = new_v;
		usleep(1000000);
		usleep(1000000);
		usleep(1000000);
	}
	return NULL;
}

/***********************************************************************
*	blink_green()
*	nothing exciting, just blink the GRN LED for half a second
*	then return the LED to its original state
***********************************************************************/
int blink_green(){
	// record if the led was on or off so we can return later
	int old_state = get_led_state(GREEN);
	
	const int us_to_blink = 700000; // 0.7 seconds
	const int blink_hz = 10;
	const int delay = 1000000/(2*blink_hz); 
	const int blinks = blink_hz*us_to_blink/1000000;
	int i;
	for(i=0;i<blinks;i++){
		usleep(delay);
		set_led(GREEN,!old_state);
		usleep(delay);
		set_led(GREEN,old_state);
	}
	return 0;
}

/***********************************************************************
*	blink_red()
*	used to warn user that the program is exiting
***********************************************************************/
int blink_red(){
	const int us_to_blink = 2000000; // 2 seconds
	const int blink_hz = 10;
	const int delay = 1000000/(2*blink_hz); 
	const int blinks = blink_hz*us_to_blink/1000000;
	int i;
	for(i=0;i<blinks;i++){
		usleep(delay);
		set_led(RED,HIGH);
		usleep(delay);
		set_led(RED,LOW);
	}
	return 0;
}


// /************************************************************************
// *	mavlink_sender
// *	send mavlink heartbeat and IMU attitude packets
// ************************************************************************/
// void* mavlink_sender(void* ptr){
	// uint8_t buf[MAV_BUF_LEN];
	// mavlink_message_t msg;
	// uint16_t len;
	// while(get_state() != EXITING){
		
		// // send heartbeat
		// memset(buf, 0, MAV_BUF_LEN);
		// mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
		// len = mavlink_msg_to_send_buffer(buf, &msg);
		// sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
		// //send attitude
		// memset(buf, 0, MAV_BUF_LEN);
		// mavlink_msg_attitude_pack(1, 200, &msg, microsSinceEpoch(), 
											// cstate.roll, 
											// cstate.pitch,
											// cstate.yaw, 
											// cstate.dRoll,
											// cstate.dPitch,
											// cstate.dYaw);
		// len = mavlink_msg_to_send_buffer(buf, &msg);
		// sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
		// usleep(100000); // 10 hz
	// }
	// return NULL;
// }

// /************************************************************************
// *	Safety thread 
// *	check for rollover 
// *	TODO: check for low battery too
// ************************************************************************/
// void* safety_thread_func(void* ptr){
	// while(get_state()!=EXITING){
		// // check for tipover
		// if(setpoint.core_mode != DISARMED){
			
		// }
		// usleep(50000); // check at ~20hz
	// }
	// return NULL;
// }

/************************************************************************
*	Watch for new DSM2 data and interpret into local user mode
*	Watch for loss of DSM2 radio communication
*	after config.land_timeout, go into emergency land mode
* 	after config.disarm_timeout disarm the motors completely 
************************************************************************/
void* dsm2_watcher(void* ptr){
	timespec last_dsm2_time, current_time;
	
	// toggle using_dsm2 to 1 when first packet arrives
	// only check timeouts if this is true
	int using_dsm2; 
	
	while(get_state()!=EXITING){
		// record time and process new data
		clock_gettime(CLOCK_MONOTONIC, &current_time);
		
		switch (is_new_dsm2_data()){
		case 1:	
			using_dsm2 = 1;
			user_interface.input_mode = DSM2; // DSM2 takes highest priority right now
			// record time and process new data
			clock_gettime(CLOCK_MONOTONIC, &last_dsm2_time);
			// user hit the kill switch, emergency disarm
			if(get_dsm2_ch_normalized(5)<0){
				user_interface.kill_switch = DISARMED;
			}
			else{
				user_interface.kill_switch = ARMED;
			}
			// configure your radio switch layout here
			user_interface.throttle_stick = get_dsm2_ch_normalized(1);
			// positive roll means tipping right
			user_interface.roll_stick 	= -get_dsm2_ch_normalized(2);
			// positive pitch means tipping backwards
			user_interface.pitch_stick 	= -get_dsm2_ch_normalized(3);
			// positive yaw means turning left
			user_interface.yaw_stick 	= get_dsm2_ch_normalized(4);
			
			// only use ATTITUDE for now
			if(get_dsm2_ch_normalized(6)>0){
				user_interface.flight_mode = USER_ATTITUDE;
			}
			else{
				user_interface.flight_mode = USER_ATTITUDE;
			}
			break;
			
		// No new data, check for time-outs
		case 0:
			if(using_dsm2){
				timespec timeout = diff(last_dsm2_time, current_time);
				float timeout_secs = timeout.tv_sec + (timeout.tv_nsec/1000000000.0);
				
				// if timeout met, set the input mode from DSM2 to none
				if(timeout_secs > config.dsm2_timeout && user_interface.input_mode==DSM2){
					printf("\n\nlost DSM2 communication \n");
					user_interface.input_mode = NONE;
				}
				
				// start landing the the cutout is still short
				else if(user_interface.flight_mode != EMERGENCY_LAND &&
							timeout_secs > config.land_timeout){
					printf("\n\nlost DSM2 communication for %0.1f seconds\n", timeout_secs);
					printf("EMERGENCY LANDING\n");
					user_interface.flight_mode = EMERGENCY_LAND;
					user_interface.throttle_stick 	= -1;
					user_interface.roll_stick 		= 0;
					user_interface.pitch_stick 		= 0;
					user_interface.yaw_stick 		= 0;
				}
				break;
			}
		default:
			break;  // should never get here
		}
		
		usleep(10000); // ~ 100hz
	}
	return NULL;
}


/************************************************************************
*	print a flight mode to console
************************************************************************/
int print_flight_mode(flight_mode_t mode){
	switch(mode){
	case EMERGENCY_LAND:
		printf("EMERGENCY_LAND");
		break;
	case USER_ATTITUDE:
		printf("USER_ATTITUDE");
		break;
	// case USER_LOITER:
		// printf("USER_LOITER\n");
		// break;
	// case USER_POSITION_CARTESIAN:
		// printf("USER_POSITION_CARTESIAN\n");
		// break;
	// case USER_POSITION_RADIAL:
		// printf("USER_POSITION_RADIAL\n");
		// break;
	// case TARGET_HOLD:
		// printf("TARGET_HOLD\n");
		// break;
	default:
		printf("unknown\n");
		break;
	}
	fflush(stdout);
	return 0;
}

/***********************************************************************
*	printf_loop() 
*	prints diagnostics to console
*   this only gets started if executing from terminal
************************************************************************/
void* printf_loop(void* ptr){
	// keep track of last global state variable
	state_t last_state, new_state;
	arm_state_t last_arm_state, new_arm_state;
	last_arm_state = DISARMED;
	int i = 0;
	
	printf("\nTurn your transmitter kill switch UP\n");
	printf("Then move throttle UP then DOWN to arm\n");
	
	while(1){
		if(get_state()==EXITING){
			return NULL;
		}
		
		new_state = get_state();
		new_arm_state = setpoint.arm_state;
		// check if this is the first time since being paused
		if((new_state==RUNNING && last_state!=RUNNING) ||\
			(new_arm_state != last_arm_state)){
			printf("\nyaw  u_r  u_p  u_y  u_thr ");
			for(i=0; i<config.rotors; i++){
				printf("    m%d", i+1);
			}
			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED: press pause button again to start.\n");
		}
		last_state = new_state;
		last_arm_state = new_arm_state;
		
		printf("\r");
		printf("%0.1f ", cstate.yaw);
		printf("%0.2f ", cstate.control_u[0]);
		printf(" %0.2f ", cstate.control_u[1]);
		printf(" %0.1f ", cstate.control_u[2]);
		printf(" %0.1f    ", cstate.control_u[3]);
		for(i=0; i<config.rotors; i++){
			printf(" %0.2f ", cstate.esc_out[i]);
		}
		
		if(setpoint.arm_state == ARMED){
			printf("ARMED");}
		else{
			printf("DISARMED");
		}
		
		// if(user_interface.kill_switch == ARMED){
			// printf(" ARMED");}
		// else{
			// printf(" DISARMED");
		// }
		
		
		printf("   "); // clear remaining characters
		fflush(stdout);
			
		usleep(100000);
	}
	return NULL;
}