// balance_logging.h
// struct and function definitions for logging BeagleMiP state data

#include <dirent.h> // for reading files in a directory

/************************************************************************
* 	LOG_TABLE
*	macros are used to turn this into either a struct or giant printf
************************************************************************/
#define LOG_TABLE \
	X(uint64_t,"%lld",time_us	) \
	X(float,  "%0.3f", theta_ref	) \
    X(float,  "%0.3f", theta		) \
	X(float,  "%0.3f", phi			) \
	X(float,  "%0.2f", u			)

	

// save log every 2 seconds
#define LOG_RATE_HZ 1

/************************************************************************
* 	log_entry_t
*	struct definition to contain single line of the log
************************************************************************/
#define X(type, fmt, name) type name ;
typedef struct log_entry_t{LOG_TABLE}log_entry_t;
#undef X


/************************************************************************
* 	log_writer()
*	independent thread that monitors the log_needs_writing flag
*	and dumps a buffer to file in one go
*	this should be started by you after start_log with pthread_create
************************************************************************/
void* log_writer();

/************************************************************************
* 	start_log()
*	create a new csv log file with the date and time as a name
*	also print header as the first line to give variable names
*	For now just number files sequentially till the RTC works
************************************************************************/
int start_log(int ctrl_rate, uint64_t * ctrl_time_us);

/************************************************************************
* 	stop_log()
*	finish writing remaining data to log and close it
************************************************************************/
int stop_log();

/************************************************************************
* 	add_to_buffer()
*	called by parent function to quickly add new data to local buffer
************************************************************************/
int add_to_buffer(log_entry_t new_entry);

/************************************************************************
* 	log_blank_entry()
*	append a single line of zeros to the log
*	usually to mark when the controller is zero'd before start
************************************************************************/
int log_blank_entry();