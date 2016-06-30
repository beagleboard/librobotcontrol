/*******************************************************************************
* balance_config.c
*
* Contains the settings struct for configuration of balance.c
* Settings are contained here for neatness.
* Macros are used to take CONFIG_TABLE and both declare a struct
* and then fill it with the default values.
*******************************************************************************/

#ifndef BALANCE_CONFIG
#define BALANCE_CONFIG

#define SAMPLE_RATE_HZ 200	// main filter and control loop speed
#define DT 0.005       		// 1/sample_rate

// Structural properties of eduMiP
#define 	CAPE_MOUNT_ANGLE		0.40
#define 	GEARBOX 				35.577
#define 	ENCODER_RES				60
#define 	WHEEL_RADIUS_M			0.034
#define 	TRACK_WIDTH_M			0.035
#define 	V_NOMINAL				7.4

// inner loop controller
#define 	D1_K					1.0	
#define 	D1_ORDER				2
#define 	D1_NUM					{-6.289, 11.910, -5.634 }
#define 	D1_DEN					{ 1.000, -1.702,  0.702 }

// outer loop controller
#define 	D2_K					0.7
#define		D2_ORDER				1
#define 	D2_NUM					{ 0.3858, -0.3853 }
#define 	D2_DEN					{ 1.0000, -0.9277 }
#define 	D2_SATURATION			0.37

// steering controller
#define 	D3_KP					1.0
#define 	D3_KI					0.05
#define 	D3_KD					0.1

// electrical hookups
#define 	MOTOR_CHANNEL_L 		4
#define 	MOTOR_CHANNEL_R 		1
#define 	MOTOR_POLARITY_L 		1
#define 	MOTOR_POLARITY_R 		-1
#define 	ENCODER_CHANNEL_L 		3
#define 	ENCODER_CHANNEL_R 		2
#define 	ENCODER_POLARITY_L 		1
#define 	ENCODER_POLARITY_R 		-1

//	drive speeds when using remote control (dsm2)
#define 	DRIVE_RATE_NOVICE 		16
#define 	TURN_RATE_NOVICE		6
#define 	DRIVE_RATE_ADVANCED		26
#define 	TURN_RATE_ADVANCED		10

// DSM2 channel config
#define 	DSM2_DRIVE_POLARITY		1
#define 	DSM2_TURN_POLARITY  	-1
#define 	DSM2_DRIVE_CH 	 	 	3
#define 	DSM2_TURN_CH 	 	 	2
#define 	DSM2_DEAD_ZONE			0.04

// stop/start criteria 
#define 	TIP_ANGLE 				0.75
#define 	START_ANGLE 			0.3	
#define 	START_DELAY 			0.5	
#define 	PICKUP_DETECTION_TIME 	0.65


#endif //BALANCE_CONFIG