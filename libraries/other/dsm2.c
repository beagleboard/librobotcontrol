/*******************************************************************************
* dsm2.c
*
* This file contains all dsm2 related functions and is compiled into 
* robotics_cape.so but kept here separately for tidyness.
*
*******************************************************************************/

#include "../useful_includes.h"
#include "../robotics_cape.h"
#include "../robotics_cape_defs.h"

// used for setting interrupt input pin
#include "../simple_gpio/simple_gpio.h" 

// uncomment debug defines to print raw data for debugging
//#define DEBUG
//#define DEBUG_RAW

#define MAX_DSM2_CHANNELS 9
#define GPIO_PIN_BIND 30 //P9.11 gpio_0[30]
#define PINMUX_PATH "/sys/devices/ocp.3/P9_11_pinmux.13/state"
#define PAUSE 115	//microseconds
#define DEFAULT_MIN 1142
#define DEFAULT_MAX 1858

#define DSM2_UART_BUS 		4
#define DSM2_BAUD_RATE 		115200
#define DSM2_PACKET_SIZE 	16

/*******************************************************************************
* Local Global Variables
*******************************************************************************/
int running;
int rc_channels[MAX_DSM2_CHANNELS];
int rc_maxes[MAX_DSM2_CHANNELS];
int rc_mins[MAX_DSM2_CHANNELS];
int num_channels; // actual number of channels being sent
int resolution; // 10 or 11
int new_dsm2_flag;
int dsm2_frame_rate;
uint64_t last_time;
pthread_t serial_parser_thread;
int listening; // for calibration routine only
int (*dsm2_ready_func)();
int is_dsm2_active_flag; 

/*******************************************************************************
* Local Function Declarations
*******************************************************************************/
int load_default_calibration();
void* serial_parser(void *ptr); //background thread
void* calibration_listen_func(void *params);

/*******************************************************************************
* int initialize_dsm2()
* 
* returns -1 for failure or 0 for success
* This starts the background thread serial_parser_thread which listens
* for serials packets on that interface.
*******************************************************************************/ 
int initialize_dsm2(){
	int i;
	//if calibration file exists, load it and start spektrum thread
	FILE *cal;
	char file_path[100];

	// construct a new file path string
	strcpy (file_path, CONFIG_DIRECTORY);
	strcat (file_path, DSM2_CAL_FILE);
	
	// open for reading
	cal = fopen(file_path, "r");

	if (cal == NULL) {
		printf("\nDSM2 Calibration File Doesn't Exist Yet\n");
		printf("Run calibrate_dsm2 example to create one\n");
		printf("Using default values for now\n");
		load_default_calibration();
	}
	else{
		for(i=0;i<MAX_DSM2_CHANNELS;i++){
			fscanf(cal,"%d %d", &rc_mins[i],&rc_maxes[i]);
		}
		#ifdef DEBUG
		printf("DSM2 Calibration Loaded\n");
		#endif
		fclose(cal);
	}
	
	
	dsm2_frame_rate = 0; // zero until mode is detected on first packet
	running = 1; // lets uarts 4 thread know it can run
	num_channels = 0;
	last_time = 0;
	is_dsm2_active_flag = 0;
	set_new_dsm2_data_func(&null_func);
	
	if(initialize_uart(DSM2_UART_BUS, DSM2_BAUD_RATE, 0.1)){
		printf("Error, failed to initialize UART%d for DSM2\n", DSM2_UART_BUS);
	}
	
	pthread_create(&serial_parser_thread, NULL, serial_parser, (void*) NULL);
	#ifdef DEBUG
	printf("DSM2 Thread Started\n");
	#endif
	return 0;
}


/*******************************************************************************
* @ int get_dsm2_ch_raw(int ch)
* 
* Returns the pulse width in microseconds commanded by the transmitter for a
* particular channel. The user can specify channels 1 through 9 but non-zero 
* values will only be returned for channels the transmitter is actually using. 
* The raw values in microseconds typically range from 900-2100us for a standard
* radio with default settings.
*******************************************************************************/
int get_dsm2_ch_raw(int ch){
	if(ch<1 || ch > MAX_DSM2_CHANNELS){
		printf("please enter a channel between 1 & %d",MAX_DSM2_CHANNELS);
		return -1;
	}
	else{
		new_dsm2_flag = 0;
		return rc_channels[ch-1];
	}
}

/*******************************************************************************
* float get_dsm2_ch_normalized(int ch)
* 
* Returns a scaled value from -1 to 1 corresponding to the min and max values
* recorded during calibration. The user
* MUST run the clalibrate_dsm2 example to ensure the normalized values returned
* by this function are correct.
*******************************************************************************/
float get_dsm2_ch_normalized(int ch){
	if(ch<1 || ch > MAX_DSM2_CHANNELS){
		printf("please enter a channel between 1 & %d",MAX_DSM2_CHANNELS);
		return -1;
	}
	float range = rc_maxes[ch-1]-rc_mins[ch-1];
	if(range!=0 && rc_channels[ch-1]!=0) {
		new_dsm2_flag = 0;
		float center = (rc_maxes[ch-1]+rc_mins[ch-1])/2;
		return 2*(rc_channels[ch-1]-center)/range;
	}
	else{
		return 0;
	}
}

/*******************************************************************************
* @ int is_new_dsm2_data()
* 
* returns 1 if new data is ready to be read by the user. 
* otherwise returns 0
*******************************************************************************/
int is_new_dsm2_data(){
	return new_dsm2_flag;
}

/*******************************************************************************
* @ int set_new_dsm2_data_func(int (*func)(void))
* 
* sets the 
*******************************************************************************/
int set_new_dsm2_data_func(int (*func)(void)){
	dsm2_ready_func = func;
	return 0;
}

/*******************************************************************************
* @ int get_dsm2_frame_resolution()
* 
* returns 10 or 11 indicating 10-bit or 11-bit resolution
* returns a 0 if no packet has been received yet
*******************************************************************************/
int get_dsm2_frame_resolution(){
	return resolution;
}

/*******************************************************************************
* @ int get_num_dsm2_channels()
* 
* returns the number of ds2 channels currently being sent
* returns 0 if no packets have been received yet.
*******************************************************************************/
int get_num_dsm2_channels(){
	return num_channels;
}

/*******************************************************************************
* @ int is_dsm2_active()
* 
* returns 1 if packets are arriving in good health without timeouts.
* returns 0 otherwise.
*******************************************************************************/
int is_dsm2_active(){
	return is_dsm2_active_flag;
}

/*******************************************************************************
* @ int ms_since_last_dsm2_packet()
* 
* returns the number of milliseconds since the last dsm2 packet was received.
* if no packet has ever been received, return -1;
*******************************************************************************/
int ms_since_last_dsm2_packet(){
	// if global variable last_time ==0 then no packet must have arrived yet
	if (last_time==0){
		return -1;
	}
	// otherwise in normal operation just subtract last time from new time
	uint64_t current_time = micros_since_epoch();
	
	return (int)((current_time-last_time)/1000);
}

/*******************************************************************************
* @ void* serial_parser(void *ptr)
* 
* This is a local function that is started as a background thread by 
* initialize_dsm2(). This monitors the serial port and interprets data
* for each packet, it determines 10 or 11 bit resolution
* Radios with more than 7 channels split data across multiple packets. Thus, 
* new data is not committed until a full set of channel data is received.
*******************************************************************************/
void* serial_parser(void *ptr){
	char buf[DSM2_PACKET_SIZE]; // large serial buffer to catch doubled up packets
	int i, ret;
	int new_values[MAX_DSM2_CHANNELS]; // hold new values before committing
	
	flush_uart(DSM2_UART_BUS); // flush the buffer
	
	// running will become 0 when stop_dsm2_service() is called
	// mostly likely that will be called by cleanup_cape()
	while(running && get_state()!=EXITING){
		memset(&buf, 0, sizeof(buf)); // clear buffer
		// read the buffer and decide what to do
		ret = uart_read_bytes(DSM2_UART_BUS, DSM2_PACKET_SIZE, buf);
		if(ret<0){ //error
			printf("ERROR reading uart %d\n", DSM2_UART_BUS);
			is_dsm2_active_flag=0;
			goto end;
		}
		else if(ret==0){//timeout
			is_dsm2_active_flag=0; // indicate connection is no longer active
			goto end;
		}
		else if (ret>0 && ret<DSM2_PACKET_SIZE){ // partial packet
			// try to get back in sync
			flush_uart(DSM2_UART_BUS); // flush the buffer
			goto end;
		}
		
		// okay, must have a full packet now
		
		#ifdef DEBUG
			printf("read %d bytes, ", j+i);
		#endif
		
		#ifdef DEBUG_RAW
		printf("read %d bytes, ", j+i);
		for(i=0; i<8; i++){
			printf(byte_to_binary(buf[i*2]));
			printf(" ");
			printf(byte_to_binary(buf[(i*2)+1]));
			printf("   ");
		}
		printf("\n");
		#endif
		
		
		
		// Next we must check if the packets are 10-bit 1024/22ms mode
		// or 11bit 2048/11ms mode. read through and decide which one, then read
		int mode = 22; // start assuming 10-bit 1024 mode, 22ms
		unsigned char ch_id;
		int16_t value;
		
		// first check each channel id assuming 1024/22ms mode
		// where the channel id lives in 0b01111000 mask
		// if one doesn't make sense, must be using 2048/11ms mode
		for(i=1;i<=7;i++){
			// last few words in buffer are often all 1's, ignore those
			if(buf[2*i]!=0xFF && buf[(2*i)+1]!=0xFF){
				// grab channel id from first byte
				ch_id = (buf[i*2]&0b01111100)>>2;
				// maximum 9 channels, if the channel id exceeds that,
				// we must be reading it wrong, swap to 11ms mode
				if((ch_id+1)>MAX_DSM2_CHANNELS){
					#ifdef DEBUG
					printf("2048/11ms ");
					#endif
					
					mode = 11;
					goto read_packet;
				}
			}
		}
		#ifdef DEBUG
			printf("1024/22ms ");
		#endif

read_packet:			
		// packet is 16 bytes, 8 words long
		// first word doesn't have channel data, so iterate through last 7 words
		for(i=1;i<=7;i++){
			// in  8 and 9 ch radios, unused words are 0xFF
			// skip if one of them
			if(buf[2*i]!=0xFF || buf[(2*i)+1]!=0xFF){
				// grab channel id from first byte
				// and value from both bytes
				if(mode == 22){
					dsm2_frame_rate = 22;
					ch_id = (buf[i*2]&0b01111100)>>2; 
					// grab value from least 11 bytes
					value = ((buf[i*2]&0b00000011)<<8) + buf[(2*i)+1];
					value += 989; // shift range so 1500 is neutral
				}
				else if(mode == 11){
					dsm2_frame_rate = 11;
					ch_id = (buf[i*2]&0b01111000)>>3; 
					// grab value from least 11 bytes
					value = ((buf[i*2]&0b00000111)<<8) + buf[(2*i)+1];
					
					// extra bit of precision means scale is off by factor of 
					// two, also add 989 to center channels around 1500
					value = (value/2) + 989; 
				}
				else{
					printf("ERROR: dsm2 mode incorrect\n");
					goto end;
				}
				
				#ifdef DEBUG
				printf("%d %d  ",ch_id,value);
				#endif
				
				if((ch_id+1)>9){
					#ifdef DEBUG
					printf("error: bad channel id\n");
					#endif
					
					#ifndef DEBUG
					goto end;
					#endif
				}
				// throttle is first channel always
				// ch_id is 0 indexed
				// and rc_channels is 0 indexed
				new_values[ch_id] = value;
				if((ch_id+1)>num_channels){
					num_channels = ch_id+1;
				}
			}
		}

		// check if a complete set of channel data has been received
		// otherwise wait for another packet with more data
		int is_complete=1;
		for(i=0;i<num_channels;i++){
			if (new_values[i]==0){
				is_complete=0;
				#ifdef DEBUG
				printf("waiting for rest of data in next packet\n");
				#endif
				break;
			}
		}
		if(is_complete){
			#ifdef DEBUG
			printf("all data complete now\n");
			#endif
			new_dsm2_flag=1;
			is_dsm2_active_flag=1;
			resolution = mode;
			last_time = micros_since_epoch();
			for(i=0;i<num_channels;i++){
				rc_channels[i]=new_values[i];
				new_values[i]=0;
			}
			// run the dsm2 ready function.
			// this is null unless user changed it
			dsm2_ready_func();
		}
		
		#ifdef DEBUG
		printf("\n");
		#endif
		
end: ;

	}
	return NULL;
}

/*******************************************************************************
* @ int stop_dsm2_service()
* 
* signals the serial_parser_thread to stop and allows up to 1 second for the 
* thread to  shut down before returning.
*******************************************************************************/
int stop_dsm2_service(){
	int ret = 0;
	
	if (running){
		running = 0; // this tells serial_parser_thread loop to stop
		// allow up to 0.3 seconds for thread cleanup
		timespec thread_timeout;
		clock_gettime(CLOCK_REALTIME, &thread_timeout);
		timespec_add(&thread_timeout, 0.3);
		int thread_err = 0;
		thread_err = pthread_timedjoin_np(serial_parser_thread, NULL, 
															   &thread_timeout);
		if(thread_err == ETIMEDOUT){
			printf("WARNING: dsm2 serial_parser_thread exit timeout\n");
			ret = -1;
		}
	}
	
	running = 0;
	return ret;
}

/*******************************************************************************
* int bind_dsm2()
*
* the user doesn't need to call this function. Just use the bind_dsm2 example
* program instead.
*
* DSM satellite receivers are put into bind mode by sending them a sequence of
* pulses right after it receives power and starts up. This program puts the 
* normally UART signal pin into GPIO pulldown mode temporarily, detects when the 
* user unplugs and plugs back in the receiver, then sends the binding pulses. 
*
* the number of pulses dictates the mode the satellite receiver will request
* the transmitter to use. The transmitter may bind but use a different mode.
* I suggest configuring your radio to use DSMX 11ms fast mode if it allows that.
*
* 2048 & 1024 indicates 10 or 11 bit resolution.
* 11ms & 22ms indicates the time period between the transmitter sending frames.
* 11ms is required for transmitters with 8 or more channels.
* 
* Testing done with DX7s, DX6i, DX8, and Orange T-SIX
* 
* Table of Bind Modes
*  pulses      mode        
*   3      DSM2 1024/22ms 
*   5  	DSM2 2048/11ms
*   7  	DSMX 1024/22ms: 
*   9  	DSMx 2048/11ms: 
*******************************************************************************/

int bind_dsm2(){
	int value;
	int i;
	char c = 0; // for reading user input
	// default to dsmx 11ms mode for most applications
	int pulses = 9; 
	int delay = 200000;
	
	// swap pinmux from UART4_RX to GPIO
	FILE *pinmux_fd;
	pinmux_fd = fopen(PINMUX_PATH, "w+");
	if((int)pinmux_fd == -1){
		printf("error opening pinmux\n");
		return -1;
	}
	fprintf(pinmux_fd, "%s", "gpio_pd");
	fflush(pinmux_fd);
	//export GPIO pin to userspace
	if(gpio_export(GPIO_PIN_BIND)){
		printf("error exporting gpio pin\n");
		return -1;
	}
	// first set the pin as input (pulldown) to detect when receiver is attached
	gpio_set_dir(GPIO_PIN_BIND, INPUT_PIN);
	
	// give user instructions
	printf("\n\nYou must choose which DSM mode to request from your\
															transmitter\n");
	printf("Note that your transmitter may actually bind in a different \
																	mode\n");
	printf("depending on how it is configured.\n");
	printf("We suggest option 1 for 6-channel DSM2 radios,\n");
	printf("and option 4 for 7-9 channel DSMX radios\n");
	printf("\n");
	printf("1: DSM2 10-bit 22ms framerate\n");
	printf("2: DSM2 11-bit 11ms framerate\n"); 
	printf("3: DSMX 10-bit 22ms framerate\n"); 
	printf("4: DSMX 11-bit 11ms framerate\n"); 
	printf("5: Orange 10-bit 22ms framerate\n");
	printf("\n"); 
	printf("Enter mode 1-5: ");
	
	// wait for user input
enter:
	c = getchar();
 
	switch(c){
		case '1':
			pulses = 3;
			break;
		case '2':
			pulses = 5;
			break;
		case '3':
			pulses = 7;
			break;
		case '4':
			pulses = 9;
			break;
		case '5':
			pulses = 9;
			delay = 50000;
			break;
		case '\n':
			goto enter;
			break;
		default:
			printf("incorrect mode number\n");
			getchar();
			goto enter;
			break;
	}
		
    printf("Using mode %c\n", c);

	// wait for user to hit enter before continuing
	printf("\nDisconnect your dsm2 satellite receiver if it is still connected\n");
	printf("Plug it into the cape quickly and firmly to begin binding.\n");
	
	// wait for the receiver to be disconnected
	value = 1;
	while(value==1){ //pin will go low when disconnected
		gpio_get_value(GPIO_PIN_BIND, &value);
	}
	usleep(100000);
	
	//wait for the receiver to be plugged in
	//receiver will pull pin up when connected
	while(value==0){ 
		gpio_get_value(GPIO_PIN_BIND, &value);
	}
	
	// start pairing packet
	gpio_set_dir(GPIO_PIN_BIND, OUTPUT_PIN);
	gpio_set_value(GPIO_PIN_BIND, HIGH);
	
	// wait as long as possible before sending pulses
	// in case the user plugs in the receiver slowly at an angle
	// which would delay the power pin from connecting 
	usleep(delay); 
	
	for(i=0; i<pulses; i++){
		gpio_set_value(GPIO_PIN_BIND, LOW);
		usleep(PAUSE);
		gpio_set_value(GPIO_PIN_BIND, HIGH);
		usleep(PAUSE);
	}
	
	usleep(1000000);
	
	// swap pinmux back to uart
	fprintf(pinmux_fd, "%s", "uart");
	fflush(pinmux_fd);
	fclose(pinmux_fd);
	
	// all done
	printf("Your receiver should now be blinking. If not try again.\n");
	printf("Now turn on your transmitter in bind mode.\n");
	printf("Use test_dsm2 to confirm functionality.\n\n");
	
	return 0;
}

/*******************************************************************************
* int load_default_calibration()
*
* internal function, writes default values to the config file. Generally only
* used once the first time initialize_dsm2 is called after a clean install
*******************************************************************************/
int load_default_calibration(){
	int i;
	for(i=0;i<MAX_DSM2_CHANNELS;i++){
		rc_mins[i]=DEFAULT_MIN;
		rc_maxes[i]=DEFAULT_MAX;
	}
	return 0;
}

/*******************************************************************************
* void *calibration_listen_func(void *params)
*
* this is started as a background thread by calibrate_dsm2_routine(). 
* Only used during calibration to monitor data as it comes in.
*******************************************************************************/
void *calibration_listen_func(void *params){
	//wait for data to start
	printf("waiting for dsm2 connection");
	get_dsm2_ch_raw(1); // flush the data ready flag with a read
	while(!is_new_dsm2_data()){
		if(get_state()==EXITING || listening==0){
			return 0;
		}
		usleep(5000); 
	}
	
	//start limits at first value
	int j;
	for(j=0;j<MAX_DSM2_CHANNELS;j++){
		rc_mins[j]=get_dsm2_ch_raw(j+1);
		rc_maxes[j]=get_dsm2_ch_raw(j+1);
	}
	
	// record limits until user presses enter
	while(listening && get_state()!=EXITING){
		printf("\r");
		if (is_new_dsm2_data()){
			for(j=0;j<MAX_DSM2_CHANNELS;j++){
				if(get_dsm2_ch_raw(j+1) > 0){ //record only non-zero channels
					if(get_dsm2_ch_raw(j+1)>rc_maxes[j]){
						rc_maxes[j] = get_dsm2_ch_raw(j+1);
					}
					else if(get_dsm2_ch_raw(j+1)<rc_mins[j]){
						rc_mins[j] = get_dsm2_ch_raw(j+1);
					}
					printf("%d:%d ",j+1,get_dsm2_ch_raw(j+1));
				}
			}
			fflush(stdout);
		}
		usleep(10000); 
	}
	return 0;
}

/*******************************************************************************
* int calibrate_dsm2_routine()
*
* routine for measuring the min and max values from a transmitter on each
* channel and save to disk for future use.
* If a channel isn't used by the transmitter then default values are saved.
* if the user forgot to move one of the channels during the calibration process
* then defualt values are also saved.
*******************************************************************************/
int calibrate_dsm2_routine(){
	int i,ret;
	
	dsm2_frame_rate = 0; // zero until mode is detected on first packet
	running = 1; // lets uarts 4 thread know it can run
	num_channels = 0;
	last_time = 0;
	is_dsm2_active_flag = 0;
	set_new_dsm2_data_func(&null_func);
	
	if(initialize_uart(DSM2_UART_BUS, DSM2_BAUD_RATE, 0.1)){
		printf("Error, failed to initialize UART%d for DSM2\n", DSM2_UART_BUS);
	}
	
	pthread_create(&serial_parser_thread, NULL, serial_parser, (void*) NULL);
		
	// display instructions
	printf("\nRaw dsm2 data should display below if the transmitter and\n");
	printf("receiver are paired and working. Move all channels through\n");
	printf("their range of motion and the minimum and maximum values will\n");
	printf("be recorded. When you are finished moving all channels,\n");
	printf("press ENTER to save the data or any other key to abort.\n\n");		
	
	// start listening
	listening = 1;
	pthread_t  listening_thread;
	pthread_create(&listening_thread, NULL, calibration_listen_func, (void*) NULL);
	
	// wait for user to hit enter
	ret = continue_or_quit();
	
	//stop listening
	listening=0;
	stop_dsm2_service();
	pthread_join(listening_thread, NULL);
	
	
	// abort if user hit something other than enter
	if(ret<0){
		printf("aborting calibrate_dsm2 routine\n");
		return -1;
	}
	
	//if it looks like new data came in, write calibration file
	if((rc_mins[0]==0) || (rc_mins[0]==rc_maxes[0])){ 
		printf("no new data recieved, exiting\n");
		return -1;
	}
	
	// construct a new file path string
	char file_path[100];
	strcpy (file_path, CONFIG_DIRECTORY);
	strcat (file_path, DSM2_CAL_FILE);
	
	// open for writing
	FILE* cal;
	cal = fopen(file_path, "w");
	if (cal == NULL) {
		mkdir(CONFIG_DIRECTORY, 0777);
		cal = fopen(file_path, "w");
		if (cal == 0){
			printf("could not open config directory\n");
			printf(CONFIG_DIRECTORY);
			printf("\n");
			return -1;
		}
	}

	// if new data was captures for a channel, write data to cal file
	// otherwise fill in defaults for unused channels in case
	// a higher channel radio is used in the future with this cal file
	for(i=0;i<MAX_DSM2_CHANNELS;i++){
		if((rc_mins[i]==0) || (rc_mins[i]==rc_maxes[i])){
			fprintf(cal, "%d %d\n",DEFAULT_MIN, DEFAULT_MAX);
		}
		else{
			fprintf(cal, "%d %d\n", rc_mins[i], rc_maxes[i]);
		}
	}
	fclose(cal);
	printf("New calibration file written\n");
	printf("use test_dsm2 to confirm\n");
	return 0;
}


