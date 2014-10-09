// test_log_file.c
// James Strawson - 2014
// sample to demonstrate logging robot data to a file
// specifically this logs IMU sensor readings to a new log file

#ifndef ROBOTICS_CAPE
#include <robotics_cape.h>
#endif

#define CORE_LOG_TABLE \
	X(long,   "%ld", num_loops	) \
    X(float,  "%f",	 roll		) \
    X(float,  "%f",	 pitch		) \
    X(float,  "%f",	 yaw		) \
	X(float,  "%f",	 dRoll		) \
    X(float,  "%f",	 dPitch		) \
    X(float,  "%f",	 dYaw		) \
	X(float,  "%f",	 u_0		) \
    X(float,  "%f",	 u_1		) \
    X(float,  "%f",	 u_2		) \
	X(float,  "%f",	 u_3		) \
    X(float,  "%f",	 esc_1		) \
    X(float,  "%f",	 esc_2		) \
	X(float,  "%f",	 esc_3		) \
    X(float,  "%f",	 esc_4		) \
    X(float,  "%f",	 v_batt		)

#define CORE_LOG_BUF_LEN 200 //once per second is reasonable

/************************************************************************
* 	core_log_entry_t
*	struct definition to contain single line of the log
************************************************************************/
#define X(type, fmt, name) type name ;
typedef struct core_log_entry_t { CORE_LOG_TABLE } core_log_entry_t;
#undef X

/************************************************************************
* 	Global Variables
************************************************************************/
typedef struct core_logger_t{
	long num_entries;	// number of entries logged so far
	int buffer_pos; // position in current buffer
	int current_buf; //0 or 1 to indicate which buffer is being filled
	int needs_writing;
	FILE* log_file;
	// array of two buffers so one can fill while writing the other to file
	core_log_entry_t log_buffer[2][CORE_LOG_BUF_LEN];
}core_logger_t;

core_logger_t core_logger;

/************************************************************************
* 	print_entry()
*	write the contents of one entry to the console
************************************************************************/
int print_entry(core_logger_t* logger, core_log_entry_t* entry){	
	
	#define X(type, fmt, name) printf("%s " fmt "\n", #name, entry->name);
	CORE_LOG_TABLE
	#undef X
	
	return 0;
}


/************************************************************************
* 	log_core_data()
*	called by an outside function to quickly add new data to local buffer
************************************************************************/
int log_core_data(core_logger_t* log, core_log_entry_t* new_entry){
	if(log->needs_writing && log->buffer_pos >= CORE_LOG_BUF_LEN){
		printf("warning, both logging buffers full\n");
		return -1;
	}
	log->log_buffer[log->current_buf][log->buffer_pos] = *new_entry;
	log->buffer_pos ++;
	log->num_entries ++;
	// we've filled a buffer, set the write flag and swap to other buffer
	if(log->buffer_pos >= CORE_LOG_BUF_LEN){
		log->buffer_pos = 0;
		log->needs_writing = 1;
		if(log->current_buf==0) log->current_buf = 1;
		else log->current_buf = 0;
		
	}
	return 0;
}

/************************************************************************
* 	write_core_log_entry()
*	append a single entry to the log file
************************************************************************/
int write_core_log_entry(FILE* f, core_log_entry_t* entry){
	#define X(type, fmt, name) fprintf(f, fmt "," , entry->name);
    CORE_LOG_TABLE
	#undef X	
	fprintf(f, "\n");
	return 0;
}

/************************************************************************
* 	core_log_writer()
*	independent thread that monitors the needs_writing flag
*	and dumps a buffer to file in one go
************************************************************************/
void* core_log_writer(void* new_log){
	core_logger_t *log = new_log;
	while(1){
		int i,j;
		if(log->needs_writing){
			if(log->current_buf == 0) j=1;
			else j=0;
			for(i=0;i<CORE_LOG_BUF_LEN;i++){
				write_core_log_entry(log->log_file, &log->log_buffer[j][i]);
			}
			fflush(log->log_file);
			log->needs_writing = 0;
		}
		usleep(10000);
	}
	return NULL;
}


/************************************************************************
* 	start_core_log()
*	create a new csv log file with the date and time as a name
*	also print header as the first line to give variable names
*	and start a thread to write
************************************************************************/
int start_core_log(core_logger_t* log){
	char time_str[50];
	char logfile_path[100];
    time_t t;
    struct tm *tmp;
	t = time(NULL);
    tmp = localtime(&t);
    if (tmp == NULL) {
        printf("can't access localtime\n");
		return -1;
    }
    if (strftime(time_str, sizeof(time_str),
		"%a %d %b %Y %R:%S", tmp) == 0) {
        printf("strftime returned 0");
        return -1;
    }
	

	// construct new logfile name
	strcpy (logfile_path, LOG_DIRECTORY);
	strcat (logfile_path, time_str);
	strcat (logfile_path, ".csv");
	printf("starting new logfile\n");
	printf("%s\n", logfile_path);
	
	log->log_file = fopen(logfile_path, "w");
	if (log->log_file==NULL){
		printf("could not open logging directory\n");
		return -1;
	}
	
	#define X(type, fmt, name) fprintf(log->log_file, "%s," , #name);
    CORE_LOG_TABLE
	#undef X	
	
	fprintf(log->log_file, "\n");
	fflush(log->log_file);
	return 0;
}

/************************************************************************
* 	stop_core_log()
*	finish writing remaining data to log and close it
************************************************************************/
int stop_core_log(core_logger_t* log){
	int i;
	// wait for previous write to finish if it was going
	while(log->needs_writing){
		usleep(10000);
	}
	
	// if there is a partially filled buffer, write to file
	if(log->buffer_pos > 0){
		for(i=0;i<log->buffer_pos;i++){
			write_core_log_entry(log->log_file, &log->log_buffer[log->current_buf][i]);
		}
		fflush(log->log_file);
		log->needs_writing = 0;
	}
	fclose(log->log_file);
	return 0;
}