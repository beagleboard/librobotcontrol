// balance_logging.h
// struct and function definitions for logging BeagleMiP state data

#include <dirent.h> // for reading files in a directory

/************************************************************************
* 	LOG_TABLE
*	macros are used to turn this into either a struct or giant printf
************************************************************************/
#define LOG_TABLE \
	X(float,  "%f", theta_ref	) \
    X(float,  "%f", theta		) \
	X(float,  "%f", d_theta		) \
	X(float,  "%f", phi_ref		) \
	X(float,  "%f", phi			) \
	X(float,  "%f", d_phi		) \
	X(float,  "%f", gamma		) \
	X(float,  "%f", gamma_ref	) \
	X(float,  "%f", d_gamma	) \
	X(float,  "%f", u			) \
	X(float,  "%f", duty_split	) \
	X(float,  "%f", v_batt		)
	
// save log every 2 seconds
#define LOG_BUFFER_SIZE 200 

/************************************************************************
* 	log_entry_t
*	struct definition to contain single line of the log
************************************************************************/
#define X(type, fmt, name) type name ;
typedef struct log_entry_t { LOG_TABLE } log_entry_t;
#undef X

/************************************************************************
* 	Global Variables
************************************************************************/
long num_entries=0;	// number of entries logged so far
int buffer_position = 0; // position in current buffer
int current_buffer; //0 or 1 to indicate which buffer is being filled
int log_needs_writing = 0;
FILE* log_file;
// array of two buffers so one can fill while writing the other to file
log_entry_t log_buffer[2][LOG_BUFFER_SIZE];


/************************************************************************
* 	print_entry()
*	write the contents of one entry to the console
************************************************************************/
int print_entry(log_entry_t* entry){	
	
	#define X(type, fmt, name) printf("%s " fmt "\n", #name, entry->name);
	LOG_TABLE
	#undef X
	
	return 0;
}


/************************************************************************
* 	add_to_buffer()
*	called by an outside function to quickly add new data to local buffer
************************************************************************/
int add_to_buffer(log_entry_t* new_entry){
	if(log_needs_writing && buffer_position >= LOG_BUFFER_SIZE){
		printf("warning, both logging buffers full\n");
		return -1;
	}
	log_buffer[current_buffer][buffer_position]= *new_entry;
	buffer_position ++;
	num_entries ++;
	// we've filled a buffer, set the write flag and swap to other buffer
	if(buffer_position >= LOG_BUFFER_SIZE){
		buffer_position = 0;
		log_needs_writing = 1;
		if(current_buffer==0)current_buffer = 1;
		else current_buffer = 0;
		
	}
	return 0;
}

/************************************************************************
* 	write_entry()
*	append a single entry to the log file
************************************************************************/
int write_entry(log_entry_t* entry){
	#define X(type, fmt, name) fprintf(log_file, fmt "," , entry->name);
    LOG_TABLE
	#undef X	
	fprintf(log_file, "\n");
	return 0;
}

/************************************************************************
* 	log_writer()
*	independent thread that monitors the log_needs_writing flag
*	and dumps a buffer to file in one go
************************************************************************/
void* log_writer(){
	while(get_state()!= EXITING){
		int i,j;
		if(log_needs_writing){
			if(current_buffer == 0) j=1;
			else j=0;
			for(i=0;i<LOG_BUFFER_SIZE;i++){
				write_entry(&log_buffer[j][i]);
			}
			fflush(log_file);
			log_needs_writing = 0;
		}
		usleep(10000);
	}
	return NULL;
}


/************************************************************************
* 	start_log()
*	create a new csv log file with the date and time as a name
*	also print header as the first line to give variable names
*	For now just number files sequentially till the RTC works
************************************************************************/
int start_log(){
	//char time_str[50];
	char logfile_path[100];
    
	// time_t t;
    // struct tm *tmp;
	// t = time(NULL);
    // tmp = localtime(&t);
    // if (tmp == NULL) {
        // printf("can't access localtime\n");
		// return -1;
    // }
    // if (strftime(time_str, sizeof(time_str),
		// "%a %d %b %Y %R:%S", tmp) == 0) {
        // printf("strftime returned 0");
        // return -1;
    // }
	
	
	// count the number of log files already in the logging directory
	int file_count = 0;
	DIR* dir;
	struct dirent* entry;

	dir = opendir(LOG_DIRECTORY);
	if (dir == NULL){
		printf("Can't find %s\n", LOG_DIRECTORY);
		printf("making it now\n");
		mkdir(LOG_DIRECTORY, 0700);
		dir = opendir(LOG_DIRECTORY);
		if (dir == NULL){
			printf("mkdir failed, can't start log file\n");
			return -1;
		}
	}
	// increment file_count for each file found
	while ((entry = readdir(dir)) != NULL) {
		if (entry->d_type == DT_REG) {
			 file_count++;
		}
	}
	closedir(dir);
	
	// construct new logfile name from timestring or just give it
	// the next number in the file order
	//strcat (logfile_path, time_str);

	sprintf(logfile_path, "%s/balance_log_%d.csv", LOG_DIRECTORY, file_count+1);
	printf("starting new logfile\n");
	printf("%s\n", logfile_path);
	
	log_file = fopen(logfile_path, "w");
	if (log_file==NULL){
		printf("could not open log file\n");
		return -1;
	}
	
	// print variable names as top row of the file to name columns
	#define X(type, fmt, name) fprintf(log_file, "%s," , #name);
    LOG_TABLE
	#undef X	
	fprintf(log_file, "\n");
	fflush(log_file);
	
	// start new thread to write the file occationally
	pthread_t  logging_thread;
	pthread_create(&logging_thread, NULL, log_writer, (void*) NULL);
	
	return 0;
}

/************************************************************************
* 	stop_log()
*	finish writing remaining data to log and close it
************************************************************************/
int stop_log(){
	int i;
	// wait for previous write to finish if it was going
	while(log_needs_writing){
		usleep(10000);
	}
	
	// if there is a partially filled buffer, write to file
	if(buffer_position > 0){
		for(i=0;i<buffer_position;i++) write_entry(&log_buffer[current_buffer][i]);
		fflush(log_file);
		log_needs_writing = 0;
	}
	fclose(log_file);
	return 0;
}

/************************************************************************
* 	log_blank_entry()
*	append a single line of zeros to the log
*	usually to mark when the controller is zero'd before start
************************************************************************/
int log_blank_entry(){
	log_entry_t blank_entry;
	memset(&blank_entry, 0, sizeof(log_entry_t));
	add_to_buffer(&blank_entry);
	return 0;
}


