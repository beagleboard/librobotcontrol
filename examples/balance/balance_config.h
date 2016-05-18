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

#define CONFIG_TABLE\
	X(float,  	bb_mount_angle,			0.40		) \
	X(float,  	gearbox, 				35.577		) \
	X(float,  	encoder_res,			60			) \
	X(float,  	wheel_radius, 			0.034		) \
	X(float,  	track_width, 			0.035		) \
													  \
	X(float,  	K_D1, 					1.0			) \
    X(float,  	numD1_0, 				-6.289		) \
    X(float,  	numD1_1, 				11.91		) \
    X(float,  	numD1_2, 				-5.634		) \
	X(float,  	denD1_0, 				1			) \
    X(float,  	denD1_1, 				-1.702		) \
    X(float,  	denD1_2, 				0.702		) \
													  \
	X(float,  	K_D2, 					0.7			) \
    X(float,  	numD2_0, 				0.3858		) \
    X(float,  	numD2_1, 				-0.3853		) \
    X(float,  	numD2_2, 				0.0			) \
    X(float,  	denD2_0, 				1.0 		) \
    X(float,  	denD2_1, 				-0.9277		) \
    X(float,  	denD2_2, 				0.0			) \
	X(float,  	theta_ref_max, 			0.37		) \
													  \
	X(float,  	KP_steer, 				1.0			) \
	X(float,  	KD_steer,				0.1			) \
													  \
	X(int,  	motor_channel_L, 		4			) \
	X(int,  	motor_channel_R,		1			) \
	X(int,  	encoder_channel_L, 		3			) \
	X(int,  	encoder_channel_R, 		2			) \
													  \
	X(float,  	drive_rate_novice, 		16			) \
	X(float,  	turn_rate_novice, 		6			) \
	X(float,  	drive_rate_advanced, 	26			) \
	X(float,  	turn_rate_advanced, 	10			) \
													  \
	X(float,  	tip_angle, 				0.75		) \
	X(float,  	start_angle, 			0.3			) \
	X(float,  	start_delay, 			0.5			) \
	X(float,  	pickup_detection_time, 	0.65		) \
	X(float,  	v_nominal, 				7.4			) \
													  \
	X(int,  	enable_dsm2, 	 	 	1			) \
	X(int,  	dsm2_drive_polarity,	1			) \
	X(int,  	dsm2_turn_polarity,  	-1			) \
	X(int,		dsm2_drive_ch, 	 	 	3			) \
	X(int,  	dsm2_turn_ch, 	 	 	2			) \
	X(int,  	dsm2_mode_ch, 	 	 	5			) \
	X(int,  	dsm2_timeout_ms, 	 	500			) \
	

	
// balance_config_t struct type definition
#define X(type, fmt, name, default) type name ;
typedef struct balance_config_t { CONFIG_TABLE } balance_config_t;
#undef X

// now create an instance of this struct filled with defaults
#define X(type, name, default) default ,
balance_config_t config = { CONFIG_TABLE };
#undef X

#endif //BALANCE_CONFIG