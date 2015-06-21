// drive_config.h
// James Strawson - 2014
// functions to load and save a config file

#include <robotics_cape.h>
#include <sys/stat.h> 

#define DRIVE_CONFIG_FILE "/root/robot_config/drive_basic_config.txt"

/************************************************************************
* 	DRIVE_CONFIG_TABLE
*	this contains all configuration data for the drive program
*	from this table, a struct drive_config_t is defined
*	with one global instance balace_config which is populated before 
*	launching the drive core. It can be modified on the fly since
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
	X(int, 	  "%d", enable_dsm2, 	 	 	1		) \
	X(int, 	  "%d", dsm2_drive_polarity, 	1		) \
	X(int, 	  "%d", dsm2_turn_polarity,  	-1		) \
	X(int, 	  "%d", dsm2_drive_ch, 	 	 	3		) \
	X(int, 	  "%d", dsm2_turn_ch, 	 	 	2		) \
	X(int, 	  "%d", dsm2_switch1_polarity, 	-1		) \
	X(int, 	  "%d", dsm2_switch2_polarity, 	1		) \
	X(int, 	  "%d", dsm2_switch1_ch, 	 	6		) \
	X(int, 	  "%d", dsm2_switch2_ch, 	 	5		) \
	\
	X(float,  "%f", track_width, 			0.12	) \
	X(float,  "%f", wheelbase, 				0.17	) \
	X(float,  "%f", centered_angle, 		37.5	) \
	X(float,  "%f", servo_range, 			120		) \
	\
	X(float,  "%f", mot1_polarity, 			1		) \
	X(float,  "%f", mot2_polarity, 			-1		) \
	X(float,  "%f", mot3_polarity, 			-1		) \
	X(float,  "%f", mot4_polarity, 			1		) \
	\
	X(float,  "%f", serv1_center,			0.5		) \
	X(float,  "%f", serv2_center,			0.5		) \
	X(float,  "%f", serv3_center,			0.5 	) \
	X(float,  "%f", serv4_center,			0.5 	) \
	\
	X(float,  "%f", normal_turn_range,		0.19	) \
	X(float,  "%f", turn_straight,			0.31	) \
	X(float,  "%f", turn_spin,				0.15	) \
	X(float,  "%f", turn_crab,				0.43	) \
	X(float,  "%f", crab_turn_const,		0.7		) \
	X(float,  "%f", turn_max,				1.0		) \
	X(float,  "%f", turn_min,				0.0		) \
	\
	X(float,  "%f", motor_max, 				1.0		) \
	X(float,  "%f", torque_vec_const, 		0.2		) \
	\
	X(float,  "%f", v_nominal, 				7.4		)

  
 
/************************************************************************
* 	drive_config_t
*	struct definition to contain information in local memory
************************************************************************/
#define X(type, fmt, name, default) type name ;
typedef struct drive_config_t { CONFIG_TABLE } drive_config_t;
#undef X


/************************************************************************
* 	construct_default()
*	struct definition to contain information in local memory
************************************************************************/
drive_config_t construct_default(){		
	#define X(type, fmt, name, default) default ,
	drive_config_t defaults = { CONFIG_TABLE };
	#undef X
	return defaults;
}

/************************************************************************
* 	print_config()
*	print the configuration to console
************************************************************************/
int print_config(drive_config_t* config){	
	
	#define X(type, fmt, name, default) printf("%s " fmt "\n", #name, config->name);
	CONFIG_TABLE
	#undef X
	
	return 0;
}


/************************************************************************
* 	save_config()
*	write the config struct to disk
************************************************************************/
int save_config(FILE *f, drive_config_t* config){
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
int read_file(FILE *f, drive_config_t* config){
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
int load_config(drive_config_t* config){
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
		config_file = fopen(DRIVE_CONFIG_FILE, "r");
		// if no file yet, make a new one
		if (config_file==NULL){
			printf("DRIVE_CONFIG_FILE doesn't exist yet\n");
			printf("generating a new one with default values\n");
			*config = construct_default();
			config_file = fopen(DRIVE_CONFIG_FILE, "w+");
			if (config_file==NULL){
				printf("WARNING can't create DRIVE_CONFIG_FILE\n");
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
		stat(DRIVE_CONFIG_FILE, &file_attributes);
		last_time = file_attributes.st_mtime;
	}
	
	// if this is not the first time opening the file, 
	// check if it's been modified and load if so
	else{
		stat(DRIVE_CONFIG_FILE, &file_attributes);
		if(last_time == file_attributes.st_mtime){
			return 0; // no changes, just return
		}
		// read the file
		config_file = fopen(DRIVE_CONFIG_FILE, "r");
		read_file(config_file, config);
		// record when it was saved
		last_time = file_attributes.st_mtime;
		printf("loaded updated config file\n");		
	}
	
	fclose(config_file);
	return 1;
}
