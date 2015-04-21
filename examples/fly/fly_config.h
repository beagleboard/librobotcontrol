// functions to save and load flight_core configurations
// James Strawson - 2014


#ifndef ROBOTICS_CAPE
#include <robotics_cape.h>
#endif

#define FLY_CONFIG_FILE "/root/robot_config/fly_config.txt"

#define CONFIG_TABLE\
	X(int, 	  "%d", enable_logging,	  	0		) \
	\
	X(int, 	  "%d", enable_dsm2, 	 	 1		) \
	X(int, 	  "%d", dsm2_thr_ch, 	 	 1		) \
	X(int, 	  "%d", dsm2_yaw_ch, 	 	 4		) \
	X(int, 	  "%d", dsm2_roll_ch, 	 	 2		) \
	X(int, 	  "%d", dsm2_pitch_ch, 	 	 3		) \
	X(int, 	  "%d", dsm2_kill_ch, 	 	 5		) \
	X(int, 	  "%d", dsm2_mode_ch, 	 	 6		) \
	X(int, 	  "%d", dsm2_thr_polarity, 	 1		) \
	X(int, 	  "%d", dsm2_yaw_polarity, 	 1		) \
	X(int, 	  "%d", dsm2_roll_polarity,	 -1		) \
	X(int, 	  "%d", dsm2_pitch_polarity, -1		) \
	X(int, 	  "%d", dsm2_kill_polarity,  1		) \
	X(int, 	  "%d", dsm2_mode_polarity,  1		) \
	\
	X(int, 	  "%d", enable_mavlink_tx,	0		)\
	X(int, 	  "%d", enable_mavlink_rx,	0		) \
	\
	X(int, 	  "%d", rotors,				4		) \
	X(char,	  "%c", rotor_layout,		'X'		) \
												\
	X(float,  "%f", alt_KP,				0		)\
	X(float,  "%f", alt_KD, 			0		)\
	X(float,  "%f", alt_KI, 			0		)\
												\
	X(float,  "%f", max_roll_setpoint,	.4		)\
	X(float,  "%f", roll_rate_per_rad,  6		)\
	X(float,  "%f", Droll_KP, 			.05		)\
	X(float,  "%f", Droll_KI, 			.001	)\
	X(float,  "%f", Droll_KD, 			.003	)\
												\
	X(float,  "%f", max_pitch_setpoint,	.4		)\
	X(float,  "%f", pitch_rate_per_rad, 6		)\
	X(float,  "%f", Dpitch_KP, 			.05		)\
	X(float,  "%f", Dpitch_KI, 			.001	)\
	X(float,  "%f", Dpitch_KD, 			.003	)\
												\
	X(float,  "%f", yaw_KP, 			.2		)\
	X(float,  "%f", yaw_KI, 			0		)\
	X(float,  "%f", yaw_KD, 			.3		)\
												\
	X(float,  "%f", max_yaw_rate,		3		)\
	X(float,  "%f", idle_speed,			.11		)\
	X(float,  "%f", max_throttle, 		.8		)\
	X(float,  "%f", tip_angle, 			1.5		)\
	X(float,  "%f", v_nominal, 			7.4		)\
	X(float,  "%f", dsm2_timeout, 		0.3		)\
	X(float,  "%f", land_timeout, 		0.3		)\
	X(float,  "%f", disarm_timeout,		4.0		)\
	X(float,  "%f", land_throttle,		.15		)\
												\
	X(float,  "%f", max_vert_velocity,	3		)\
	X(float,  "%f", max_horz_velocity, 	3		)\
												\
	X(float,  "%f", aux1,	0	)\
	X(float,  "%f", aux2,	0	)\
	X(float,  "%f", aux3,	0	)\
	X(float,  "%f", aux4,	0	)\
	X(float,  "%f", aux5,	0	)\
	X(float,  "%f", aux6,	0	)


/************************************************************************
* 	fly_config_t
*	this contains all configuration data for the flight_core
*	the global instance core_config is populated before launching 
*	the flight core. It can be modified while flying, eg to adjust gains
*	an instance should be declared in your own C file
************************************************************************/
#define X(type, fmt, name, default) type name ;
typedef struct fly_config_t { CONFIG_TABLE } fly_config_t;
#undef X


/************************************************************************
* 	print_core_config()
*	print configuration table to console
************************************************************************/
int print_core_config(fly_config_t* config){	
	#define X(type, fmt, name, default) printf("%s " fmt "\n", #name, config->name);
	CONFIG_TABLE
	#undef X
	
	return 0;
}

/************************************************************************
* 	save_config()
*	write the config struct to disk
************************************************************************/
int save_config(FILE *f, fly_config_t* config){
	rewind(f);
	#define X(type, fmt, name, default) fprintf(f, #name "," fmt "\n", config->name);
    CONFIG_TABLE
	#undef X	
	fflush(f);
	return 0;
}

/************************************************************************
* 	read_file()
*	called by the higher level load_config() function
*	use load_config() in your program
************************************************************************/
int read_file(FILE *f, fly_config_t* config){
	rewind(f);
	#define X(type, fmt, name, default) fscanf(f, #name "," fmt"\n", &config->name);
	CONFIG_TABLE
	#undef X
	return 0;
}

/************************************************************************
* 	construct_default()
*	struct definition to contain information in local memory
************************************************************************/
fly_config_t construct_default(){		
	#define X(type, fmt, name, default) default ,
	fly_config_t defaults = { CONFIG_TABLE };
	#undef X
	return defaults;
}

/************************************************************************
* 	load_config()
*	normally reads config from the disk
*	if the file doesn't exist, make one with defaults
*	if the file hasn't been modified since loading it last
*	then don't bother loading again
*	return -1 if error
*	return 0 if no new data but file is okay
*	return 1 if there is new data
************************************************************************/
int load_config(fly_config_t* config){
	// static struct to remember last modified time of config file
	// these are initialized as 0
	struct stat file_attributes = {0};
	static  time_t last_time = {0};
	const time_t zero_time = {0};
	FILE* config_file;
	
	// last_time is initialized as 0, so if it's 0 then this is the first 
	// call to load_config(). Thus, check if the file exists
	if(last_time == zero_time){
		// try opening the file
		config_file = fopen(FLY_CONFIG_FILE, "r");
		// if no file yet, make a new one
		if (config_file==NULL){
			printf("FLY_CONFIG_FILE doesn't exist yet\n");
			printf("generating a new one with default values\n");
			*config = construct_default();
			config_file = fopen(FLY_CONFIG_FILE, "w+");
			if (config_file==NULL){
				printf("WARNING can't create FLY_CONFIG_FILE\n");
				printf("using default values anyway\n");
				return -1;
			}
			save_config(config_file, config);
		}
		// file exists, load as normal
		else{
			read_file(config_file, config);
		}
		// record the modify time for the future
		stat(FLY_CONFIG_FILE, &file_attributes);
		last_time = file_attributes.st_mtime;
	}
	
	// if this is not the first time opening the file, 
	// check if it's been modified and load if so
	else{
		stat(FLY_CONFIG_FILE, &file_attributes);
		if(last_time == file_attributes.st_mtime){
			return 0; // no changes, just return
		}
		// read the file
		config_file = fopen(FLY_CONFIG_FILE, "r");
		read_file(config_file, config);
		// record when it was saved
		last_time = file_attributes.st_mtime;
		printf("loaded updated config file\n");		
	}
	
	fclose(config_file);
	printf("closed file\n");
	return 1;
}