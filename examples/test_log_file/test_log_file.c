// test_log_file.c
// James Strawson - 2014
// sample to demonstrate logging robot data to a file
// specifically this logs IMU sensor readings to a new log file

#include <robotics_cape.h>

#define CONTROL_HZ 50

#define LOG_DIR "/root/robot_logs/"

#define LOG_TABLE \
    X(float,  "%f", roll	) \
    X(float,  "%f", pitch	) \
    X(float,  "%f", yaw		)
	
#define LOG_BUFFER_SIZE 50 //once per second is reasonable

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
	while(1){
		int i,j;
		if(log_needs_writing){
			if(current_buffer == 0) j=1;
			else j=0;
			for(i=0;i<LOG_BUFFER_SIZE;i++) write_entry(&log_buffer[j][i]);
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
************************************************************************/
int start_log(){
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
	strcpy (logfile_path, LOG_DIR);
	strcat (logfile_path, time_str);
	strcat (logfile_path, ".csv");
	printf("starting new logfile\n");
	printf("%s\n", logfile_path);
	
	log_file = fopen(logfile_path, "w");
	if (log_file==NULL){
		printf("could not open logging directory\n");
		return -1;
	}
	
	#define X(type, fmt, name) fprintf(log_file, "%s," , #name);
    LOG_TABLE
	#undef X	
	fprintf(log_file, "\n");
	fflush(log_file);
	
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
* 	print_imu_data()
*	hardware interrupt routine called from IMU interrupt
*	prints new IMU data and adds data to log buffer
************************************************************************/
int print_imu_data(){
	mpudata_t mpu; //struct to read IMU data into
	memset(&mpu, 0, sizeof(mpudata_t)); //make sure it's clean before starting
	log_entry_t new_log_entry; // new struct to add to the log
	
	if (mpu9150_read(&mpu) == 0) {
		printf("\r");
		
		printf("X: %0.1f Y: %0.1f Z: %0.1f ",
		mpu.fusedEuler[VEC3_X] * RAD_TO_DEGREE, 
		mpu.fusedEuler[VEC3_Y] * RAD_TO_DEGREE, 
		mpu.fusedEuler[VEC3_Z] * RAD_TO_DEGREE);

		fflush(stdout);
		
		new_log_entry.roll	= -mpu.fusedEuler[VEC3_X];
		new_log_entry.pitch =  mpu.fusedEuler[VEC3_Y];
		new_log_entry.yaw 	=  mpu.fusedEuler[VEC3_Z];
		write_entry(&new_log_entry);
	}
	return 0; 
}


/************************************************************************
* 	main()
*	test constructing, saving, and loading a config
************************************************************************/
int main(){
	
	// initialize cape hardware
	if(initialize_cape()<0){
		return -1;
	}
	
	if(start_log()<0){
		return -1;
	}
	
	pthread_t logging_thread;
	pthread_create(&logging_thread, NULL, log_writer, log_file);
	
	
	// Start reading from IMU to get data to log
	signed char orientation[9] = ORIENTATION_FLAT;
	if(initialize_imu(CONTROL_HZ, orientation)){
		printf("IMU initialization failed, please reboot\n");
		cleanup_cape();
		return -1;
	}
	set_imu_interrupt_func(&print_imu_data);
	

	while(get_state() != EXITING){
		usleep(100000);
	}
	
	
	
	
	stop_log();
	cleanup_cape();

    return 0;
}