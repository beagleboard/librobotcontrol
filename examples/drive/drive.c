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

/********************************************
* 			Includes & Constants			*
*********************************************/
#include <robotics_cape.h>
#include "drive_config.h"

#define CONTROL_HZ 50

/************************************************************************
* 	core_state_t
*	contains workings of the drive stack and information about motor
*	and servo control
*	Should only be written to by the drive_stack thread	
************************************************************************/
typedef struct core_state_t{
	// outputs
	float servos[4];
	float motors[4];
	//battery voltage for scaling motor inputs.
	float vBatt; 
} core_state_t;

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
* 	drive_mode_t
*	possible modes of driving around
************************************************************************/
typedef enum drive_mode_t {
	NORMAL,
	NORMAL_4W,
	CRAB,
	SPIN
}drive_mode_t;

/************************************************************************
* 	user_interface_t
*	represents current command by the user which may be populated from 
*	DSM2, mavlink, bluetooth, or any other communication you like
************************************************************************/
typedef struct user_interface_t{
	// written by the last input watching thread to update
	input_mode_t input_mode;
	drive_mode_t drive_mode;
		
	// All sticks scaled from -1 to 1
	float drive_stick; 	// positive forward
	float turn_stick;	// positive to the right, CW yaw
	

}user_interface_t;

/************************************************************************
* 	Function declarations in this c file
*	also check out functions in balance_config.h & balance_logging.h		
************************************************************************/
//threads
void* drive_stack(void* ptr);
void* battery_checker(void* ptr);
void* printf_loop(void* ptr);
void* dsm2_watcher(void* ptr);

// regular functions
int saturate_number(float* val, float min, float max);
int on_pause_press();
int print_drive_mode(drive_mode_t mode);
int on_mode_release();
int blink_green();
int blink_red();

/************************************************************************
* 	Global Variables				
************************************************************************/
drive_config_t config;
core_state_t cstate;
user_interface_t user_interface;


/***********************************************************************
*	main()
*	start all the threads, and wait still something 
*	triggers a shut down
***********************************************************************/
int main(){
	// initialize cape hardware
	if(initialize_cape()<0){
		blink_red();
		return -1;
	}
	setRED(HIGH);
	setGRN(LOW);
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
	
	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	if(isatty(fileno(stdout))){
		pthread_t  printf_thread;
		pthread_create(&printf_thread, NULL, printf_loop, (void*) NULL);
	}
	
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

	// this thread is in charge of arming and managing the core
	pthread_t  drive_stack_thread;
	pthread_create(&drive_stack_thread, NULL, drive_stack, (void*) NULL);
	
	// all threads have started, off we go
	set_state(RUNNING);
	setRED(LOW);
	setGRN(HIGH);
	
	// chill until something exits the program
	while(get_state()!=EXITING){
		usleep(100000);
	}
	
	cleanup_cape(); // always end with cleanup to shut down cleanly
	return 0;
}

/***********************************************************************
*	drive_stack
*	This is the medium between the user_interface struct and the 
*	physical servos and motors
************************************************************************/
void* drive_stack(void* ptr){
	int i; // general purpose counter for for loops
	float net_drive, net_turn, net_torque_split;

	// exiting condition is checked inside the switch case instead
	while(1){
		switch (get_state()){
		case EXITING:
			return NULL;
			
		case PAUSED:
			disable_motors();
			// not much to do if paused!
			break;
			
		// when running, drive_stack checks if an input mode
		// like mavlink, DSM2, or bluetooth is enabled
		// and moves the servos and motors corresponding to 
		// user input and current controller mode
		case RUNNING:
			if(user_interface.input_mode == NONE){
				// no user input, nothing to do yet
				break;
			}
			enable_motors();
			// now send input to servos and motors based on drive mode and UI
			// 1 3		0  2
			// 2 4		1  3
			switch(user_interface.drive_mode){
			case NORMAL:		// Front wheel steering only
				net_drive = user_interface.drive_stick*config.motor_max;
				net_turn = user_interface.turn_stick*config.normal_turn_range;
				net_torque_split=user_interface.turn_stick*config.torque_vec_const*net_drive;
				cstate.motors[0]=(net_drive+net_torque_split)*config.mot1_polarity;
				cstate.motors[1]=(net_drive+net_torque_split)*config.mot2_polarity;
				cstate.motors[2]=(net_drive-net_torque_split)*config.mot3_polarity;
				cstate.motors[3]=(net_drive-net_torque_split)*config.mot4_polarity;
				cstate.servos[0]=config.serv1_straight;
				cstate.servos[1]=config.serv2_straight - net_turn;
				cstate.servos[2]=config.serv3_straight - net_turn;
				cstate.servos[3]=config.serv4_straight;
				break;
				
			case NORMAL_4W:		// Four wheel steering
				net_drive = user_interface.drive_stick*config.motor_max;
				net_turn = user_interface.turn_stick*config.normal_turn_range;
				net_torque_split = user_interface.turn_stick*config.torque_vec_const*net_drive;
				cstate.motors[0]=(net_drive+net_torque_split)*config.mot1_polarity;
				cstate.motors[1]=(net_drive+net_torque_split)*config.mot2_polarity;
				cstate.motors[2]=(net_drive-net_torque_split)*config.mot3_polarity;
				cstate.motors[3]=(net_drive-net_torque_split)*config.mot4_polarity;
				cstate.servos[0]=config.serv1_straight + net_turn;
				cstate.servos[1]=config.serv2_straight - net_turn;
				cstate.servos[2]=config.serv3_straight - net_turn;
				cstate.servos[3]=config.serv4_straight + net_turn;
				break;
			
			// crab, turn all wheels sideways and drive
			case CRAB:
				net_drive = user_interface.drive_stick*config.motor_max;
				net_turn = user_interface.turn_stick*config.crab_turn_const\
														*(net_drive+0.5);
				cstate.motors[0]=(-net_drive+net_turn)*config.mot1_polarity;
				cstate.motors[1]=(net_drive+net_turn)*config.mot2_polarity;
				cstate.motors[2]=(-net_drive-net_turn)*config.mot3_polarity;
				cstate.motors[3]=(net_drive-net_turn)*config.mot4_polarity;
				cstate.servos[0]=config.serv1_straight+config.turn_for_crab;
				cstate.servos[1]=config.serv2_straight-config.turn_for_crab;
				cstate.servos[2]=config.serv3_straight+config.turn_for_crab;
				cstate.servos[3]=config.serv4_straight-config.turn_for_crab;
				break;
				
			case SPIN:
				net_drive = user_interface.turn_stick*config.motor_max;
				cstate.motors[0]=net_drive*config.mot1_polarity;
				cstate.motors[1]=net_drive*config.mot2_polarity;
				cstate.motors[2]=-net_drive*config.mot3_polarity;
				cstate.motors[3]=-net_drive*config.mot4_polarity;
				cstate.servos[0]=config.serv1_straight+config.turn_for_spin;
				cstate.servos[1]=config.serv2_straight-config.turn_for_spin;
				cstate.servos[2]=config.serv3_straight+config.turn_for_spin;
				cstate.servos[3]=config.serv4_straight-config.turn_for_spin;
				break;
				
			default:
				printf("unknown drive_mode\n");
				disable_motors();
				for (i=1; i<=4; i++){
					cstate.motors[i-1]=0;
					cstate.servos[i-1]=0.5;
				}
				break;
			}// end of switch(drive_mode)
			
			// send pulses to servos and drive motors
			for (i=1; i<=4; i++){
				saturate_number(&cstate.servos[i-1],.05,.95);
				saturate_number(&cstate.motors[i-1],-config.motor_max,config.motor_max);
				set_motor(i,cstate.motors[i-1]);
				send_servo_pulse_normalized(i,cstate.servos[i-1]);
			}
	
		default:
			break;
		} // end of switch get_state()
		
		// run about as fast as the core itself 
		usleep(1000000/CONTROL_HZ); 
	}
	return NULL;
}

/***********************************************************************
*	print_drive_mode(drive_mode_t mode)
*	prints a readable text name of one of the 4 drive modes
************************************************************************/
int print_drive_mode(drive_mode_t mode){
	switch(mode){
		case NORMAL:
			printf("drive_mode: NORMAL\n");
			break;
			
		case NORMAL_4W:
			printf("drive_mode: NORMAL_4W\n");
			break;
			
		case CRAB:
			printf("drive_mode: CRAB\n");
			break;
			
		case SPIN:
			printf("drive_mode: SPIN\n");
			break;
			
		default:
			printf("unknown drive_mode\n");
			return -1;
	}
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

/***********************************************************************
*	battery_checker()
*	super slow loop checking battery voltage
************************************************************************/
void* battery_checker(void* ptr){
	float new_v;
	while(get_state()!=EXITING){
		new_v = getBattVoltage();
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
*	printf_loop() 
*	prints diagnostics to console
*   this only gets started if executing from terminal
************************************************************************/
void* printf_loop(void* ptr){
	// keep track of last global state variable
	state_t last_state;
	drive_mode_t last_drive_mode = user_interface.drive_mode;
	int print_header_flag = 1;
	print_drive_mode(last_drive_mode);
	
	while(1){
		// check if this is the first time since being paused
		if(get_state()==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING\n");
			print_header_flag=1;
		}
		else if(get_state()==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED: press pause button again to start.\n");
		}
		if(user_interface.drive_mode != last_drive_mode){
			printf("\n\n");
			print_drive_mode(user_interface.drive_mode);
			print_header_flag=1;
		}
		last_state = get_state();
		last_drive_mode = user_interface.drive_mode;
		// decide what to print or exit
		switch (get_state()){	
		case RUNNING: { // show all the things
			if(print_header_flag){
				printf("    motors              servos  \n");
				printf(" 1   2   3   4       1   2   3   4\n");
				print_header_flag=0;
			}
			printf("\r");
			printf("%0.1f ", cstate.motors[0]);
			printf("%0.1f ", cstate.motors[1]);
			printf("%0.1f ", cstate.motors[2]);
			printf("%0.1f   ", cstate.motors[3]);
			printf("%0.2f ", cstate.servos[0]);
			printf("%0.2f ", cstate.servos[1]);
			printf("%.2f ", cstate.servos[2]);
			printf("%.2f ", cstate.servos[3]);
			printf("   "); // clear remaining characters
			fflush(stdout);
			break;
			}
		case PAUSED: {
			break;
			}
		case EXITING:{
			return NULL;
			}
		default: {
			break; // this is only for UNINITIALIZED state
			}
		}
		usleep(200000);
	}
	return NULL;
} 
/***********************************************************************
*	on_pause_press() 
*	Disarm the controller and set system state to paused.
*	If the user holds the pause button for 2 seconds, exit cleanly
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
		setRED(HIGH);
		setGRN(LOW);
		break;
	case PAUSED:
		set_state(RUNNING);
		setGRN(HIGH);
		setRED(LOW);
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
*	placeholder, blinks green led for now
***********************************************************************/
int on_mode_release(){
	blink_green();
	return 0;
}

/***********************************************************************
*	blink_green()
*	nothing exciting, just blink the GRN LED for half a second
*	then return the LED to its original state
***********************************************************************/
int blink_green(){
	// record if the led was on or off so we can return later
	int old_state = getGRN();
	
	const int us_to_blink = 700000; // 0.7 seconds
	const int blink_hz = 10;
	const int delay = 1000000/(2*blink_hz); 
	const int blinks = blink_hz*us_to_blink/1000000;
	int i;
	for(i=0;i<blinks;i++){
		usleep(delay);
		setGRN(!old_state);
		usleep(delay);
		setGRN(old_state);
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
		setRED(HIGH);
		usleep(delay);
		setRED(LOW);
	}
	return 0;
}


/***********************************************************************
*	dsm2_watcher()
*	listen for RC control for driving around
***********************************************************************/
void* dsm2_watcher(void* ptr){
	const int timeout_frames = 10; // after 10 missed checks, consider broken
	const int check_us = 5000; // dsm2 packets come in at 11ms, check faster
	drive_mode_t temp_drive_mode; // new drive mode selected by user switches
	int missed_frames;
	float turn, drive, switch1, switch2;
	
	while(get_state()!=EXITING){
		if(is_new_dsm2_data()){	
			
			// Read normalized (+-1) inputs from RC radio right stick
			// positive means turn right or go forward
			turn = config.dsm2_turn_polarity * \
					get_dsm2_ch_normalized(config.dsm2_turn_ch);
			drive = config.dsm2_drive_polarity * \
					get_dsm2_ch_normalized(config.dsm2_drive_ch);
			switch1 = config.dsm2_switch1_polarity * \
					get_dsm2_ch_normalized(config.dsm2_switch1_ch);
			switch2 = config.dsm2_switch2_polarity * \
					get_dsm2_ch_normalized(config.dsm2_switch2_ch);
			if(switch1>0 && switch2>0){
				temp_drive_mode = NORMAL;
			}
			else if(switch1>0 && switch2<0){
				temp_drive_mode = NORMAL_4W;
			}
			else if(switch1<0 && switch2>0){
				temp_drive_mode = CRAB;
			}
			else if(switch1<0 && switch2<0){
				temp_drive_mode = SPIN;
			}
			else{
				printf("could not interpret DSM2 switches\n");
			}
			
			if(fabs(turn)>1.1 || fabs(drive)>1.1){
				// bad packet, ignore
			}
			else{
				missed_frames = 0;
				// dsm has highest interface priority so take over
				user_interface.input_mode = DSM2;
				user_interface.drive_stick = drive;
				user_interface.turn_stick  = turn;
				user_interface.drive_mode = temp_drive_mode;
			}
			
		}
		// if no new data and currently operating in DSM2 mode, 
		// count a missed frame
		else if(user_interface.input_mode == DSM2){
			missed_frames ++;
			// if enough frames were missed and in DSM2 mode, 
			// this thread relinquishes control 
			if(missed_frames >= timeout_frames){
				user_interface.input_mode = NONE;
			}
		}
		// wait for the next frame
		usleep(check_us); 
	}
	return 0;
}