/*
Copyright (c) 2014, James Strawson
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*/

// balance_logging.c
// internal and external function definitions for writing 
// logs to disk
// see balance_logging.h for outside function declarations

#include <robotics_cape.h>
#include "balance_logging.h"

/************************************************************************
* 	Global Variables
************************************************************************/
int log_buf_length; // number of log_entry_t in each buffer
long num_entries = 0;	// number of entries logged so far
int buffer_position = 0; // position in current buffer
int current_buffer; //0 or 1 to indicate which buffer is being filled
int log_needs_writing = 0;
FILE* log_file;
uint64_t* ctrl_time_ptr_local; // stores pointer to parent cstate time
int ctrl_rate_local = 200; // default to 200, this is reset by user later
int is_logging_functional = 0;

// array of two buffer pointers
// memory will be allocated to these pointers in start_log
log_entry_t* log_buffer[2]; 


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
int add_to_buffer(log_entry_t new_entry){
	if(is_logging_functional==0){
		return -1;
	}
	if(log_needs_writing && buffer_position >= log_buf_length){
		printf("warning, both logging buffers full\n");
		return -1;
	}
	log_buffer[current_buffer][buffer_position] = new_entry;
	buffer_position ++;
	num_entries ++;
	// if we've filled a buffer, set the write flag and swap to other buffer
	if(buffer_position >= log_buf_length){
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
int write_entry(log_entry_t entry){
	#define X(type, fmt, name) fprintf(log_file, fmt "," , entry.name);
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
	int i,j;
	uint64_t time;
	
	// allocate memory for the log buffer
	log_buffer[0] = (log_entry_t*)calloc(log_buf_length, sizeof(log_entry_t));
	log_buffer[1] = (log_entry_t*)calloc(log_buf_length, sizeof(log_entry_t));
	
	while(get_state()!= EXITING){
		if(log_needs_writing){
			printf("writing\n");
			// wait for the core to have just completed a control step
			// before starting the long write process
			time = microsSinceEpoch();
			if(time > *ctrl_time_ptr_local+1000){
				usleep((*ctrl_time_ptr_local)+(1000000/ctrl_rate_local)+1000-time);
			}
			// pick array index j such that is
			// the one not currently being written into
			if(current_buffer == 0) j=1;
			else j=0;
			for(i=0;i<log_buf_length;i++){
				write_entry(log_buffer[j][i]);
			}
			fflush(log_file);
			log_needs_writing = 0;
		}
		else usleep(10000);
	}
	stop_log(); 	// write rest of log file and close it
	return NULL;
}


/************************************************************************
* 	start_log()
*	create a new csv log file with the date and time as a name
*	also print header as the first line to give variable names
*	For now just number files sequentially till the RTC works
************************************************************************/
int start_log(int ctrl_rate, uint64_t * ctrl_time_us){
	ctrl_time_ptr_local = ctrl_time_us;
	ctrl_rate_local = ctrl_rate;
	log_buf_length = ctrl_rate/LOG_RATE_HZ;
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

	sprintf(logfile_path, "%sbalance_log_%04d.csv", LOG_DIRECTORY, file_count+1);
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
	is_logging_functional = 1;
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
		for(i=0;i<buffer_position;i++){
			write_entry(log_buffer[current_buffer][i]);
		}
		fflush(log_file);
		log_needs_writing = 0;
	}
	printf("closing log file\n");
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
	add_to_buffer(blank_entry);
	return 0;
}


