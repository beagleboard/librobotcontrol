// balance_config.c
// James Strawson - 2014
// functions to load and save a config file

#include <robotics_cape.h>
#include <sys/stat.h>

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
	X(int, 	  "%d", enable_dsm2, 	 	 1		) \
	X(int, 	  "%d", dsm2_drive_polarity, 1		) \
	X(int, 	  "%d", dsm2_turn_polarity,  -1		) \
	X(int, 	  "%d", dsm2_drive_ch, 	 	 3		) \
	X(int, 	  "%d", dsm2_turn_ch, 	 	 2		) \
	X(int, 	  "%d", dsm2_mode_ch, 	 	 5		) \
	\
	X(int, 	  "%d", enable_mavlink_transmitting,0)\
	X(int, 	  "%d", enable_mavlink_listening, 0 ) \
	\
	X(float,  "%f", bb_mount_angle,	0.46		) \
    X(float,  "%f", numD1_0, 		-6.289		) \
    X(float,  "%f", numD1_1, 		11.91		) \
    X(float,  "%f", numD1_2, 		-5.634		) \
	X(float,  "%f", denD1_0, 		1			) \
    X(float,  "%f", denD1_1, 		-1.702		) \
    X(float,  "%f", denD1_2, 		0.702		) \
	X(float,  "%f", K_D1, 			1.0		) \
	\
    X(float,  "%f", numD2_0, 		0.3858		) \
    X(float,  "%f", numD2_1, 		-0.3853		) \
    X(float,  "%f", numD2_2, 		0.0			) \
    X(float,  "%f", denD2_0, 		1.0 		) \
    X(float,  "%f", denD2_1, 		-0.9277		) \
    X(float,  "%f", denD2_2, 		0.0			) \
	X(float,  "%f", K_D2, 			0.7		) \
	X(float,  "%f", theta_ref_max, 	0.37		) \
	\
	X(float,  "%f", KP_steer, 		1.0			) \
	X(float,  "%f", KD_steer,		0.1			) \
	\
	X(float,  "%f", gearbox, 		35.577		) \
	X(float,  "%f", encoder_res,	60			) \
	X(float,  "%f", wheel_radius, 	0.034		) \
	X(float,  "%f", track_width, 	0.035		) \
	\
	X(int, 	 "%d", motor_channel_L, 	4		) \
	X(int, 	 "%d", motor_channel_R,		1		) \
	X(int, 	 "%d", encoder_channel_L, 	3		) \
	X(int, 	 "%d", encoder_channel_R, 	2		) \
	\
	X(float,  "%f", drive_rate_novice, 		16		) \
	X(float,  "%f", turn_rate_novice, 		6		) \
	X(float,  "%f", drive_rate_advanced, 	26		) \
	X(float,  "%f", turn_rate_advanced, 	10		) \
	\
	X(float,  "%f", tip_angle, 			0.75		) \
	X(float,  "%f", start_angle, 		0.3		) \
	X(float,  "%f", start_delay, 		0.5		) \
	X(float,  "%f", pickup_detection_time, 	0.65		) \
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
	//#define X(type, fmt, name, default) fprintf(f, "%s," fmt "\n", #name, config->name);
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
int read_file(FILE *f, balance_config_t* config){
	rewind(f);
	#define X(type, fmt, name, default) fscanf(f, #name "," fmt"\n", &config->name);
	CONFIG_TABLE
	#undef X
	return 0;
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
int load_config(balance_config_t* config){
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
		config_file = fopen(BALANCE_CONFIG_FILE, "r");
		// if no file yet, make a new one
		if (config_file==NULL){
			printf("BALANCE_CONFIG_FILE doesn't exist yet\n");
			printf("generating a new one with default values\n");
			*config = construct_default();
			config_file = fopen(BALANCE_CONFIG_FILE, "w+");
			if (config_file==NULL){
				printf("WARNING can't create balance_config_file\n");
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
		stat(BALANCE_CONFIG_FILE, &file_attributes);
		last_time = file_attributes.st_mtime;
	}
	
	// if this is not the first time opening the file, 
	// check if it's been modified and load if so
	else{
		stat(BALANCE_CONFIG_FILE, &file_attributes);
		if(last_time == file_attributes.st_mtime){
			return 0; // no changes, just return
		}
		// read the file
		config_file = fopen(BALANCE_CONFIG_FILE, "r");
		read_file(config_file, config);
		// record when it was saved
		last_time = file_attributes.st_mtime;
		printf("loaded updated config file\n");		
	}
	
	fclose(config_file);
	printf("closed file\n");
	return 1;
}
