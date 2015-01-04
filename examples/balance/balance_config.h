// balance_config.c
// James Strawson - 2014
// functions to load and save a config file

#include <robotics_cape.h>

#define BALANCE_CONFIG_FILE "/root/robot_config/balance_config.txt"

/************************************************************************
* 	BALANCE_CONFIG_TABLE
*	this contains all configuration data for the balance program
*	from this table, a struct balance_config_t is defined
*	with one global instance balace_config which is populated before 
*	launching the balance core. It can be modified on the fly since
*	this struct is populated from the file each time the controller
* 	starts. This table also contains the default values which are loaded
*	and saved to a new file if no existing file was found. 
*	
*	logging, mavlink, and bluetooth functions should be 0 or 1
*	floats are in rad, rad/s, meters, or unitless
*	LEFT and RIGHT correspond to your l&r
*	when looking at the battery side of MiP	
************************************************************************/

#define CONFIG_TABLE\
	X(int, 	  "%d", enable_logging,	  	0		) \
	X(int, 	  "%d", enable_bluetooth, 	0		) \
												  \
	X(int, 	  "%d", enable_dsm2, 	 	 0		) \
	X(int, 	  "%d", dsm2_drive_polarity, 1		) \
	X(int, 	  "%d", dsm2_turn_polarity,  -1		) \
	X(int, 	  "%d", dsm2_drive_ch, 	 	 3		) \
	X(int, 	  "%d", dsm2_turn_ch, 	 	 2		) \
												  \
	X(int, 	  "%d", enable_mavlink_transmitting,1)\
	X(int, 	  "%d", enable_mavlink_listening, 1 ) \
												  \
    X(float,  "%f", numD1_0, 		-6.289		) \
    X(float,  "%f", numD1_1, 		11.91		) \
    X(float,  "%f", numD1_2, 		-5.634		) \
	X(float,  "%f", denD1_0, 		1			) \
    X(float,  "%f", denD1_1, 		-1.702		) \
    X(float,  "%f", denD1_2, 		0.702		) \
	X(float,  "%f", K_D1, 			1.0			) \
											      \
    X(float,  "%f", numD2_0, 		0.3858		) \
    X(float,  "%f", numD2_1, 		-0.3853		) \
    X(float,  "%f", numD2_2, 		0.0			) \
    X(float,  "%f", denD2_0, 		1.0 		) \
    X(float,  "%f", denD2_1, 		-0.9277		) \
    X(float,  "%f", denD2_2, 		0.0			) \
	X(float,  "%f", K_D2, 			1.0			) \
	X(float,  "%f", theta_ref_max, 	0.45		) \
											      \
	X(float,  "%f", KP_steer, 		1.0			) \
	X(float,  "%f", KD_steer,		0.1			) \
												  \
	X(float,  "%f", gearbox, 		35.577		) \
	X(float,  "%f", encoder_res,	60			) \
	X(float,  "%f", bb_mount_angle,	0.51		) \
	X(float,  "%f", wheel_radius, 	0.034		) \
	X(float,  "%f", track_width, 	0.035		) \
												  \
	X(int, 	 "%d", motor_channel_L, 	4		) \
	X(int, 	 "%d", motor_channel_R,		1		) \
	X(int, 	 "%d", encoder_channel_L, 	3		) \
	X(int, 	 "%d", encoder_channel_R, 	2		) \
												  \
	X(float,  "%f", max_drive_rate, 	13		) \
	X(float,  "%f", max_turn_rate, 		6		) \
	X(float,  "%f", max_RC_theta, 		0.4		) \
	X(float,  "%f", tip_angle, 			0.6		) \
	X(float,  "%f", start_angle, 		0.2		) \
	X(float,  "%f", start_delay, 		1.0		) \
	X(float,  "%f", v_nominal, 			7.4		)
	
 // may try to use this later
 // 	X(char,	  "%s", mavlink_gc_ip[16], "192.168.7.1") 
 
 
/************************************************************************
* 	balance_config_t
*	struct definition to contain information in local memory
************************************************************************/
#define X(type, fmt, name, default) type name ;
typedef struct balance_config_t { CONFIG_TABLE } balance_config_t;
#undef X


/************************************************************************
* 	construct_default()
*	struct definition to contain information in local memory
************************************************************************/
balance_config_t construct_default(){		
	#define X(type, fmt, name, default) default ,
	balance_config_t defaults = { CONFIG_TABLE };
	#undef X
	return defaults;
}

/************************************************************************
* 	print_config()
*	print the configuration to console
************************************************************************/
int print_config(balance_config_t* config){	
	
	#define X(type, fmt, name, default) printf("%s " fmt "\n", #name, config->name);
	CONFIG_TABLE
	#undef X
	
	return 0;
}


/************************************************************************
* 	save_config()
*	write the config struct to disk
************************************************************************/
int save_config(FILE *f, balance_config_t* config){
	rewind(f);
	#define X(type, fmt, name, default) fprintf(f, "%s," fmt "\n", #name, config->name);
    CONFIG_TABLE
	#undef X	
	fflush(f);
	return 0;
}

/************************************************************************
* 	load_config()
*	read from the disk
************************************************************************/
int load_config(balance_config_t* config){
	// try opening the file
	printf("opening file\n");
	FILE* config_file = fopen(BALANCE_CONFIG_FILE, "r");
	
	// if no file yet, make a new one
	if (config_file==NULL){
		printf("BALANCE_CONFIG_FILE doesn't exist yet\n");
		printf("generating a new one with default values\n");
		*config = construct_default();
		
		FILE* config_file = fopen(BALANCE_CONFIG_FILE, "w+");
		if (config_file==NULL){
			printf("WARNING can't create balance_config_file\n");
			printf("using default values anyway\n");
			return -1;
		}
		save_config(config_file, config);
		fclose(config_file);
	}
	// file exists, load as normal
	else{
		rewind(config_file);
		#define X(type, fmt, name, default) fscanf(config_file, #name "," fmt"\n", &config->name);
		CONFIG_TABLE
		#undef X
		fclose(config_file);
	}
	return 0;
}
