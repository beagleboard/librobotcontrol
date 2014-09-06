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

/*
	fly.c
	DESCRIPTION:
	USE:
*/


/********************************************
* 			Inlcudes & Constants			*
*********************************************/
// Includes
#include <robotics_cape.h>

// Flight Core Constants
#define CONTROL_HZ 			200	// Run the main control loop at this rate
#define DT 				   .005	// timestep seconds MUST MATCH CONTROL_HZ
#define	SATE_LEN 			32	// number of timesteps to retain data
#define MAX_YAW_COMPONENT	0.2 // Max control delta the yaw controller can apply
#define INT_CUTOFF_TH 		0.1	// prevent integrators from running unless flying

// Flight Stack Constants
#define KILL_SWITCH_CHANNEL 6 	// reading a normalize value >0.9 should disarm
#define TIP_THRESHOLD 		1.0	// Kill propellers if it rolls or pitches past this


/********************************************
* 				Type Definitions 			*
*********************************************/
// Flight_core configuration struct
typedef struct {
	// PID Control Gains
	float altitude_K_PID[3];		// PID gains for roll rate
	float pitch_rate_per_rad;	// rad/s per rad pitch error
	float pitch_rate_K_PID[3];		// PID gains for pitch rate
	float roll_rate_per_rad;	// rad/s per rad roll error
	float roll_rate_K_PID[3];		// PID gains for roll rate
	float yaw_K_PID[3];		// PID gains for roll rate
	
	// Limits of user input
	float max_throttle;			// .8 is reasonable, set to 1 for max
	float max_yaw_rate;			//
	float max_roll_setpoint;	//
	float max_pitch_setpoint;	//
	float max_vert_velocity;	// fastest the altitude controller will move
	float max_horizontal_velocity;	// fastest the velocity controller will move
	
	// steady state reading of IMU when level. These are read from /root/imu.cal
	// it is recommended to run calibrate_imu before launching this program.
	// alternatively you can set or modify these yourself
	float imu_roll_err;			
	float imu_pitch_err;	
	
	// esc idle input when throttle is at zero
	// set to 0 to disable idle, 0.1 for slow spin
	float esc_idle_speed;
	
	// rough starting estimate of throttle needed to hover
	// this updates automatically by low-passing throttle inputs
	float hover_input = 0.3;
	
	// 3rd order discrete controller gains placeholder for when we replace PID
	// float pitch_num_K[4];		
	// float pitch_den_K[4];
	// float roll_num_K[4];
	// float roll_den_K[4];
}core_config;

// Arming State definition
// note get_state() and set_state() from cape library are for program flow control
// arming state is specifically for controlling the flight core controller 
typedef enum{
	DISARMED;
	ARMED;
}arm_state_t;

// setpoint for the attitude controller
// this can be set by the core itself or another thread
typedef struct {
	arm_state_t arm_state; //
	float altitude;		// altitude estimate (m)
	float roll;			// current roll angle (rad)
	float pitch;		// current pitch angle (rad)
	float yaw;			// current yaw angle (rad)

}core_setpoint;

// core state contains most recent values reported by the flight_core
// Should only be written by the flight core, and read by anything
typedef struct {
	unsigned int control_loops; // number of loops since flight core started
	float current_altitude;		// altitude estimate (m)
	float current_roll;			// current roll angle (rad)
	float current_pitch;		// current pitch angle (rad)
	float current_yaw;			// current yaw angle (rad)
	float altitude[STATE_LEN];  // current and previous altitudes
	float roll[STATE_LEN];  	// current and previous roll angles
	float pitch[STATE_LEN]; 	// current and previous pitch angles
	float yaw[STATE_LEN];  		// current and previous yaw angles
	float alt_err[STATE_LEN]; 	// current and previous altitudes error
	float roll_rate_err[STATE_LEN];  // current and previous roll error
	float pitch_rate_err[STATE_LEN]; // current and previous pitch error
	float yaw_err[STATE_LEN];   // current and previous yaw error
	float dAltitude;			// first derivative of altitude (m/s)
	float dRoll;				// first derivative of roll (rad/s)
	float dPitch;				// first derivative of pitch (rad/s)
	float dYaw;					// first derivative of yaw (rad/s)
	float positionX;			// estimate of X displacement from takeoff (m)
	float positionY;			// estimate of Y displacement from takeoff (m)
	float esc_out[4];			// normalized (0-1) outputs to 4 motors

}core_state;

/********************************************
* 			 Global Variables				*
*********************************************/
core_config_t core_config;
core_setpoint_t core_setpoint;
core_state_t core_state;


/********************************************
* 				 Functions					*
*********************************************/

// TODO: add ability to save and load settings to disk
int load_default_settings(){
	core_config.altitude_K_PID 		= {0,0,0};
	core_config.pitch_rate_per_rad 	= 6;
	core_config.roll_rate_per_rad	= 6;
	core_config.pitch_rate_K_PID 	= {.0,  .1,  .004};
	core_config.roll_rate_K_PID 	= {.0,  .05, .002};	
	core_config.yaw_K_PID 			= {.2,   1,  .4};
	core_config.max_throttle 		= 0.8;	
	core_config.max_yaw_rate 		= 3;
	core_config.max_roll_setpoint	= 0.4;
	core_config.max_pitch_setpoint  = 0.4;
	core_config.max_vert_velocity 	= 2;
	core_config.max_horizontal_velocity = 2;
	core_config.imu_roll_err 		= 0;			
	core_config.imu_pitch_err 		= 0;
	core_config.esc_idle_speed 		= 0.1;
}

// //TODO: load IMU offsets from disk
// int load_imu_calibration(){
	// core_config.imu_roll_err = 0;			
	// core_config.imu_pitch_err = 0;
// }


// Hardware Interrupt-Driven Flight Control Loop
int flight_core(){
	static float integrator[4], derivative[4];
	float u[4];		// normalized throttle, roll, pitch, yaw control components 
	esc[4];			// normalized inputs to escs after mixing
	static arm_state_t previous_arm_state;
	int i;	// general purpose
	
	if (mpu9150_read(&mpu) == 0) {
		
		// march system state history one step
		for(i=(STATE_LEN-1);i>0;i--){
			core_state.altitude[i] = core_state.altitude[i-1];
			core_state.roll[i] = core_state.roll[i-1];
			core_state.pitch[i] = core_state.pitch[i-1];
			core_state.yaw[i] = core_state.yaw[i-1];
			core_state.alt_err[i] = core_state.alt_err[i-1];
			core_state.roll_rate_err[i] = core_state.roll_rate_err[i-1];
			core_state.pitch_rate_err[i] = core_state.pitch_rate_err[i-1];
			core_state.yaw_err[i] = core_state.yaw_err[i-1];
		}
		
		// record new roll and pitch even when disarmed
		// this allows other threads to monitor attiude
		core_state.current_roll = mpu.fusedEuler[VEC3_Y] - core_config.imu_roll_err;
		core_state.roll[0] = core_state.current_roll;
		core_state.current_pitch = mpu.fusedEuler[VEC3_Y]- core_config.imu_pitch_err;
		core_state.pitch[0] = core_state.current_pitch;
		
		
		switch (core_setpoint.armed_state){
		case ARMED:	
		
			// check if this is the first loop since arming, if so zero out yaw
			if(previous_armed_state == DISARMED){
				imu_yaw_on_takeoff = mpu.fusedEuler[VEC3_Z];
			}
			core_state.current_yaw = mpu.fusedEuler[VEC3_Z] - imu_yaw_on_takeoff;
			core_state.yaw[0] = core_state.current_yaw;
			
			// In basic mode, user commands roll pitch directly
			core_setpoint.roll  =  get_dsm2_ch_normalized(2)*MAX_SETPOINT;
			core_setpoint.pitch = -get_dsm2_ch_normalized(3)*MAX_SETPOINT;
			
			// User commands angular velocity of yaw angle setpoint
			core_setpoint.yaw += core_config.max_yaw_rate*DT*get_dsm2_ch_normalized(4);
			
			// calculate throttle input u
			int throttle_compensation;
			throttle_compensation = 1 / cos(core_state.current_roll);
			throttle_compensation *= 1 / cos(core_state.current_pitch);
			u[0] = get_dsm2_ch_normalized(1)*MAX_SETPOINT;
			
			//// PID for Yaw
			state_error[3][0]=set_point[3]-x[3][0];
			// only run integrator if airborne 
			if(u[0] > INT_CUTOFF_TH){
				integrator[3]=DT*state_error[i][0] + integrator[i];
			}
			derivative[i]=(state_error[i][0]-state_error[i][1])/DT;
			u[i]=K[i][0]*(state_error[i][0]+K[i][1]*integrator[i]+K[i][2]*derivative[i]);
			MAX_YAW_COMPONENT
			
			//PID for Roll/pitch
			for(i=1;i<=2;i++){
				state_error[i][0]=set_point[i]-x[i][0];
				if(u[0] > INT_CUTOFF_TH){
					integrator[i]=DT*state_error[i][0] + integrator[i];}
				derivative[i]=(state_error[i][0]-state_error[i][1])/DT;
				u[i]=K[i][0]*(state_error[i][0]+K[i][1]*integrator[i]+K[i][2]*derivative[i]);
			}
			
			//direct throttle for now
			u[0] = ((get_dsm2_ch_normalized(1)+1)/2)*MAX_THROTTLE;
			
			// mixing
			esc[0]=u[0]-u[1]-u[2]-u[3];
			esc[1]=u[0]+u[1]-u[2]+u[3];
			esc[2]=u[0]+u[1]+u[2]-u[3];
			esc[3]=u[0]-u[1]+u[2]+u[3];
			
			for(i=0;i<4;i++){
				send_servo_pulse_normalized(i+1,esc[i]);
			}		
			break;
			
		case DISARMED:
			break;
			
		default: // should never get here
			break;
		}
		//remember the last state to detect transition from disarmed to armed
		previous_arm_state = core_setpoint.arm_state;
	}
	return 0;
}

// Safety thread exists to check for kill switch, rollover, and low battery
void* safety_thread_func(void* ptr){
	while(get_state()!=EXITING){
		// check for kill conditions
		if(get_dsm2_ch_normalized(6)>0.9 ||
			fabs(core_state.current_roll)>TIP_THRESHOLD ||
			fabs(core_state.current_pitch)>TIP_THRESHOLD)
		{
			disarm();
		}
		
		// TODO check for radio cutout
		usleep(20000);
	}
	return NULL;
}

// flash the red LED is armed, or turn on green if disarmed 
void* led_manager(void* ptr){
	int toggle;
	while (get_state()!=EXITING){
		switch (core_setpoint.arming_state){
		case ARMED:	
			toggle = 0;
			setGRN(HIGH);
			setRED(LOW);
			break;
		case DISARMED:
			if(toggle){
				setRED(LOW);
				toggle = 1;
			}
			else{
				setRED(HIGH);
				toggle = 0;
			}
			break;
		default:
			break;
		usleep(500000); //toggle LED every half second
	}
}

void* flight_stack(void* ptr){
	int i;
	while(get_state()!=EXITING){
		switch (core_setpoint.arming_state){
		case ARMED:	
			//print RC Radio Inputs
			printf(" RC: "); 
			for(i=0; i<4; i++){
				printf("%0.2f ", get_dsm2_ch_normalized(i));
			}
			printf(" u: ");//print control outputs u
			for(i=0; i<4; i++){
				printf("%0.2f ", u[i]);
			}
			fflush(stdout);		
			break;
		
		// When disarmed wait for arming sequence
		case DISARMED:
			printf("\nTurn on your transmitter kill switch UP\n");
			printf("Move throttle UP then DOWN to arm\n");
			while(is_new_dsm2_data()==0){ //wait for radio connection
				usleep(100000);
				if(get_state()==EXITING) break;}
			while(get_dsm2_ch_normalized(6)>-0.9){ //wait for kill switch up
				usleep(100000);
				if(get_state()==EXITING) break;}
			while(get_dsm2_ch_normalized(1)>-.9){ //wait for throttle down
				usleep(100000);
				if(get_state()==EXITING) break;}
			while(get_dsm2_ch_normalized(1)<.9){ //wait for throttle up
				usleep(100000);
				if(get_state()==EXITING) break;}
			
			// let the user know they have reached full throttle and are almost armed
			// by turning on the LED and chirping ESCs
			setGRN(HIGH);
			for(i=0;i<4;i++){send_servo_pulse_normalized(i+1,0);}
			
			while(get_dsm2_ch_normalized(1)>-.9){ //wait for throttle down
				usleep(100000);
				if(get_state()==EXITING)break;
			}
			core_setpoint.armed_sate = ARMED;
			printf("ARMED!!\n\n");
			setRED(LOW);
			break;
			
		default: // should never get here
			break;
		}
		usleep(100000); // print data ~10hz
	};
	return NULL;
}


// send mavlink heartbeat and IMU attitude packets
void* mavlink_sender(void* ptr){
	uint8_t buf[MAV_BUF_LEN];
	mavlink_message_t msg;
	uint16_t len;
	while(get_state() != EXITING){
		
		// send heartbeat
		memset(buf, 0, MAV_BUF_LEN);
		mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
		//send attitude
		memset(buf, 0, MAV_BUF_LEN);
		mavlink_msg_attitude_pack(1, 200, &msg, microsSinceEpoch(), 
											core_state.current_roll, 
											core_state.current_pitch,
											core_state.current_yaw, 
											core_state.dRoll,
											core_state.dPitch,
											core_state.dYaw);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
		usleep(100000); // 10 hz
	}
	return NULL;
}

// If the user holds the pause button for a second, exit cleanly
// disarm on momentary press
int on_pause_press(){
	disarm();
	int i=0;
	do{
		usleep(100000);
		if(get_pause_button_state() == LOW){
			return 0; //user let go before time-out
		}
		i++;
	}while(i<10);
	//user held the button down long enough, exit cleanly
	set_state(EXITING);
	return 0;
}


// Main only serves to initialize hardware and spawn threads
int main(int argc, char* argv[]){
	load_default_settings();
	initialize_cape();
	set_pause_pressed_func(&on_pause_press); 
	setRED(1);
	setGRN(0);
	set_state(PAUSED);
	
	// see if the user gave an IP address as argument
	char target_ip[100];
	if (argc == 2){
		strcpy(target_ip, argv[1]);
    }
	else{ //otherwise use default address 
		strcpy(target_ip, DEFAULT_MAV_ADDRESS);
	}
	// open a udp port for mavlink
	// sock and gcAddr are global variables needed to send and receive
	gcAddr = initialize_mavlink_udp(target_ip, &sock);
	
	// Start thread sending heartbeat and IMU attitude packets
	pthread_t  mav_send_thread;
	pthread_create(&mav_send_thread, NULL, mavlink_sender, (void*) NULL);
	printf("Sending Heartbeat Packets\n");

	// Start LED Flasher Thread
	pthread_t led_thread;
	pthread_create(&led_thread, NULL, led_manager, (void*) NULL);
	
	// Start Safety and Disarming thread
	pthread_t safety_thread;
	pthread_create(&safety_thread, NULL, safety_thread_func, (void*) NULL);
	
	/// Begin flight Stack
	pthread_t flight_stack_thread;
	pthread_create(&flight_stack_thread, NULL, flight_stack, (void*) NULL);

	// Finally start the real-time interrupt driven control thread
	signed char orientation[9] = ORIENTATION_FLAT;
	initialize_imu(SAMPLE_RATE_HZ, orientation);
	set_imu_interrupt_func(&flight_core);
	
	//chill until something exits the program
	while(get_state()!=EXITING){
		usleep(100000);
	}
	close(sock); //close network socket
	cleanup_cape();
	return 0;
}

