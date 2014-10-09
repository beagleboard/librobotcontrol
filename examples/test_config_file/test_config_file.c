// test_config_file.c
// James Strawson - 2014
// sample to demonstrate loading and saving a struct to a file
// use as template to add configuration files or save files for you project

#include <robotics_cape.h>



#define TEST_SAVE_DIR "/root/cape_configuration/test_config.txt"

#define CORE_CONFIG_TABLE\
    X(float,  "%f", pitch_kp, 	0.05		) \
    X(float,  "%f", roll_kp, 	0.05		) \
    X(float,  "%f", roll_kd, 	0.008		)


/************************************************************************
* 	core_config_t
*	struct definition to contain information in local memory
************************************************************************/
#define X(type, fmt, name, default) type name ;
typedef struct core_config_t { CORE_CONFIG_TABLE } core_config_t;
#undef X


/************************************************************************
* 	construct_default()
*	struct definition to contain information in local memory
************************************************************************/
core_config_t construct_default(){		
	#define X(type, fmt, name, default) default ,
	core_config_t defaults = { CORE_CONFIG_TABLE };
	#undef X
	return defaults;
}

/************************************************************************
* 	print_config()
*	struct definition to contain information in local memory
************************************************************************/
int print_config(core_config_t* config){	
	
	#define X(type, fmt, name, default) printf("%s " fmt "\n", #name, config->name);
	CORE_CONFIG_TABLE
	#undef X
	
	return 0;
}


/************************************************************************
* 	save_config()
*	write to the config struct to disk
************************************************************************/
int save_config(FILE *f, core_config_t* config){
	rewind(f);
	#define X(type, fmt, name, default) fprintf(f, "%s," fmt "\n", #name, config->name);
    CORE_CONFIG_TABLE
	#undef X	
	fflush(f);
	return 0;
}

/************************************************************************
* 	load_config()
*	read from the isk
************************************************************************/
int load_config(FILE *f, core_config_t* config){
	rewind(f);
	#define X(type, fmt, name, default) fscanf(f, #name "," fmt"\n", &config->name);
    CORE_CONFIG_TABLE
	#undef X
	
	return 0;
}

/************************************************************************
* 	main()
*	test constructing, saving, and loading a config
************************************************************************/
int main(){
	core_config_t core_config;
    FILE* config_file = fopen(TEST_SAVE_DIR, "w+");
	if (config_file==NULL){
		printf("could not open TEST_SAVE_DIR\n");
		return -1;
	}
	fprintf(config_file, "hello\n");
	fflush(config_file);
	
	printf("\n");
	printf("default settings:\n");
	core_config = construct_default();
	print_config(&core_config);
	
	
	printf("hit enter to write defaults to file\n");
	while(getchar() != '\n'){
		usleep(1000);
	}
	save_config(config_file, &core_config);
	
	
	printf("feel free to edit the file TEST_SAVE_DIR\n");
	printf("Then hit enter to read it back\n");
	while(getchar() != '\n'){
		usleep(1000);
	}
	memset(&core_config, 0, sizeof(core_config_t));
	load_config(config_file, &core_config);
	print_config(&core_config);
	
	
	fclose(config_file);

    return 0;
}