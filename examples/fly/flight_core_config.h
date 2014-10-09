// functions to save and load flight_core configurations
// James Strawson - 2014


#ifndef ROBOTICS_CAPE
#include <robotics_cape.h>
#endif

#define CORE_CONFIG_FILE "flight_core_config.txt"
#define CORE_FILE_HEADER "flight_core_config_name: "
#define CORE_CONFIG_TABLE\
	X(float,  "%f", alt_KP,				0	)\
	X(float,  "%f", alt_KD, 			0	)\
	X(float,  "%f", alt_KI, 			0	)\
											\
	X(float,  "%f", max_roll_setpoint,	.4	)\
	X(float,  "%f", roll_rate_per_rad,  6	)\
	X(float,  "%f", Droll_KP, 			.05	)\
	X(float,  "%f", Droll_KI, 			.001)\
	X(float,  "%f", Droll_KD, 			.003)\
											\
	X(float,  "%f", max_pitch_setpoint,	.4	)\
	X(float,  "%f", pitch_rate_per_rad, 6	)\
	X(float,  "%f", Dpitch_KP, 			.05	)\
	X(float,  "%f", Dpitch_KI, 			.001)\
	X(float,  "%f", Dpitch_KD, 			.003)\
											\
	X(float,  "%f", yaw_KP, 			.2	)\
	X(float,  "%f", yaw_KI, 			0	)\
	X(float,  "%f", yaw_KD, 			.3	)\
											\
	X(float,  "%f", max_yaw_rate,		3	)\
	X(float,  "%f", idle_speed,			.11	)\
	X(float,  "%f", max_throttle, 		.8	)\
											\
	X(float,  "%f", max_vert_velocity,	3	)\
	X(float,  "%f", max_horz_velocity, 	3	)\
											\
	X(float,  "%f", aux1,	0	)\
	X(float,  "%f", aux2,	0	)\
	X(float,  "%f", aux3,	0	)\
	X(float,  "%f", aux4,	0	)\
	X(float,  "%f", aux5,	0	)\
	X(float,  "%f", aux6,	0	)


/************************************************************************
* 	core_config_t
*	this contains all configuration data for the flight_core
*	the global instance core_config is populated before launching 
*	the flight core. It can be modified while flying, eg to adjust gains
*	an instance should be declared in your own C file
************************************************************************/
#define X(type, fmt, name, default) type name ;
typedef struct core_config_t { CORE_CONFIG_TABLE } core_config_t;
#undef X


/************************************************************************
* 	print_core_config()
*	print configuration table to console
************************************************************************/
int print_core_config(core_config_t* config){	
	#define X(type, fmt, name, default) printf("%s " fmt "\n", #name, config->name);
	CORE_CONFIG_TABLE
	#undef X
	
	return 0;
}

/************************************************************************
* 	create_empty_core_config_file()
*	creates a new file from the robotics cape library definition for 
*	config directory and this file's definition for config file name
************************************************************************/
FILE* create_empty_core_config_file(char name[]){
	char core_config_path[100];
	FILE* f;
	
	// construct a new file path string
	strcpy (core_config_path, CONFIG_DIRECTORY);
	strcat (core_config_path, CORE_CONFIG_FILE);
	
	// open
	f = fopen(core_config_path, "w");
	if (f==NULL){
		printf("could not open config directory\n");
		printf(CONFIG_DIRECTORY);
		printf("\n");
		return NULL;
	}
	
	fprintf(f, CORE_FILE_HEADER);
	fprintf(f, "%s\n", name);
	return f;
}

/************************************************************************
* 	save_core_config()
*	write the config struct to disk
* 	should be called after writing the following header line to the file
* 	fprintf(f, "flight_core_config_name: your_name");
************************************************************************/
int save_core_config(FILE* f, core_config_t* config){
	// if the file isn't already open, open it
	if(f == NULL){
		printf("open core_config and write header first\n");
		return -1;
	}
	
	// write 
	#define X(type, fmt, name, default) fprintf(f, "%s," fmt "\n", #name, config->name);
    CORE_CONFIG_TABLE
	#undef X	
	
	fclose(f);
	return 0;
}

/************************************************************************
* 	load_core_config()
*	read from the disk. returns -1 if unsuccessful 
************************************************************************/
int load_core_config(core_config_t* config){

	char core_config_path[100];
	FILE* f;
	
	// construct a new file path string
	strcpy (core_config_path, CONFIG_DIRECTORY);
	strcat (core_config_path, CORE_CONFIG_FILE);
	
	// open
	f = fopen(core_config_path, "r");
	if (f==NULL){
		printf("could not open core_config file\n");
		printf(core_config_path);
		printf("\n");
		return -1;
	}
	// read from beginning
	rewind(f);
	
	char name[100];
	fscanf(f, CORE_FILE_HEADER);
	fscanf(f, "%s\n", name);
	printf("using flight_core_config: ");
	printf(name);
	printf("\n");
	
	#define X(type, fmt, name, default) fscanf(f, #name "," fmt"\n", &config->name);
    CORE_CONFIG_TABLE
	#undef X
	
	fclose(f);
	return 0;
}


/************************************************************************
* 	create_default_core_config_file()
*	creates a new file and returns a struct containing the default values
************************************************************************/
int create_default_core_config_file(core_config_t* config){
	FILE* f;
	// construct new struct from defaults
	#define X(type, fmt, name, default) default ,
	core_config_t defaults = { CORE_CONFIG_TABLE };
	#undef X
	
	// write defaults out to user's struct
	*config=defaults;
	
	//start a new file
	f = create_empty_core_config_file("default");
	
	// if we couldn't open file for writing, return error -1
	if(f == NULL){
		return -1;
	}

	save_core_config(f, &defaults);

	printf("created new default core_config file here:\n");
	printf(CONFIG_DIRECTORY);
	printf("\n");
	
	fclose(f);
	return 0;
}