/*******************************************************************************
* dsm2.c
*
* This file contains all dsm2 related functions and is compiled into 
* robotics_cape.so but kept here separately for tidyness.
*
*******************************************************************************/

#include <useful_includes.h>
#include <robotics_cape.h>
#include <robotics_cape_revD_defs.h>
#include <sys/select.h> // for read timeout

// used for setting interrupt input pin
#include "../simple_gpio/simple_gpio.h" 


// uncomment debug defines to print raw data for debugging
//#define DEBUG_DSM2
//#define DEBUG_DSM2_RAW


#define MAX_DSM2_CHANNELS 9
#define GPIO_PIN_BIND 30 //P9.11 gpio_0[30]
#define PINMUX_PATH "/sys/devices/ocp.3/P9_11_pinmux.13/state"
#define PAUSE 115	//microseconds
#define DEFAULT_MIN 1142
#define DEFAULT_MAX 1858

/*******************************************************************************
* Local Global Variables
*******************************************************************************/
int running;
int rc_channels[MAX_DSM2_CHANNELS];
int rc_maxes[MAX_DSM2_CHANNELS];
int rc_mins[MAX_DSM2_CHANNELS];
int num_channels; // actual number of channels being sent
int resolution; // 10 or 11
int tty4_fd;
int new_dsm2_flag;
int dsm2_frame_rate;
uint64_t last_time;
pthread_t uart4_thread;
int listening; // for calibration routine only
int (*dsm2_ready_func)();

/*******************************************************************************
* Local Function Declarations
*******************************************************************************/
int write_default_dsm2_cal_file();
void* uart4_checker(void *ptr); //background thread
void* listen_func(void *params);

/*******************************************************************************
* int initialize_dsm2()
* 
* returns -1 for failure or 0 for success
* This starts the background thread uart4_thread which listens
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
		printf("Creating default file for now\n");
		write_default_dsm2_cal_file();
		cal = fopen(file_path, "r");
		for(i=0;i<MAX_DSM2_CHANNELS;i++){
			fscanf(cal,"%d %d", &rc_mins[i],&rc_maxes[i]);
		}
	}
	else{
		for(i=0;i<MAX_DSM2_CHANNELS;i++){
			fscanf(cal,"%d %d", &rc_mins[i],&rc_maxes[i]);
		}
		printf("DSM2 Calibration Loaded\n");
	}
	fclose(cal);
	
	dsm2_frame_rate = 0; // zero until mode is detected on first packet
	running = 1; // lets uarts 4 thread know it can run
	num_channels = 0;
	last_time = 0;
	set_new_dsm2_data_func(&null_func);
	pthread_create(&uart4_thread, NULL, uart4_checker, (void*) NULL);
	printf("DSM2 Thread Started\n");
	
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
	uint64_t current_time = microsSinceEpoch();
	
	return (int)((current_time-last_time)/1000);
}

/*******************************************************************************
* @ void* uart4_checker(void *ptr)
* 
* This is a local function that is started as a background thread by 
* initialize_dsm2(). This monitors the serial port and itnerprets data
* for each packet, it determines 10 or 11 bit resolution
* Radios with more than 7 channels split data across multiple packets. Thus, 
* new data is not committed untill a full set of channel data is received.
*******************************************************************************/
void* uart4_checker(void *ptr){
	fd_set set; // for select()
	int ret;  // return value for select()
	struct timeval timeout;
	char buf[64]; // large serial buffer to catch doubled up packets
	int i,j;
	int new_values[MAX_DSM2_CHANNELS]; // hold new values before committing
	
	//set up sart/stop bit and 115200 baud
	struct termios config;
	memset(&config,0,sizeof(config));
	config.c_iflag=0;
	config.c_iflag=0;
    config.c_oflag=0;
    config.c_cflag= CS8|CREAD|CLOCAL;   // 8n1, see termios.h for more info
    config.c_lflag=0;
    config.c_cc[VTIME]=0; // no timeout condition
	config.c_cc[VMIN]=1;  // only return if something is in the buffer
	
	// open for blocking reads
	if ((tty4_fd = open(UART4_PATH, O_RDWR | O_NOCTTY)) < 0) {
		printf("error opening uart4\n");
	}
	// Spektrum and Orange recievers are 115200 baud
	if(cfsetispeed(&config, B115200) < 0) {
		printf("cannot set uart4 baud rate\n");
		return NULL;
	}
	if(tcsetattr(tty4_fd, TCSAFLUSH, &config) < 0) { 
		printf("cannot set uart4 attributes\n");
		return NULL;
	}
	

	// running will become 0 when stop_dsm2_service() is called
	// mostly likely that will be called by cleanup_cape()
	while(running){
		
		memset(&buf, 0, sizeof(buf)); // clear buffer
		
		//i = read(tty4_fd,&buf,sizeof(buf)); // blocking read
		FD_ZERO(&set); /* clear the set */
		FD_SET(tty4_fd, &set); /* add our file descriptor to the set */
		timeout.tv_sec = 0;
		timeout.tv_usec = 100000; // 0.1 second timeout
		ret = select(tty4_fd + 1, &set, NULL, NULL, &timeout);
		if(ret == -1){
			#ifdef DEBUG_DSM2
			printf("ERROR with select(tty4_fd)\n");
			#endif
			goto end;
		}
		else if(ret == 0){
			#ifdef DEBUG_DSM2
			printf("UART4 DSM2 timeout, transmitter probably off\n");
			#endif
			goto end;
		}
		else{
			i=read(tty4_fd, &buf, 64); /* there was data to read */
		}
		
		if(i<0){
			#ifdef DEBUG_DSM2
			printf("error, read returned -1\n");
			#endif
			
			goto end;
		}
		else if(i>16){
			#ifdef DEBUG_DSM2
			printf("error: read too many bytes.\n");
			#endif
			goto end;
		}
		
		//incomplete packet read, wait and read the rest of the buffer
		else if(i<16){	
			// packet takes about 1.4ms at 115200 baud. sleep for 2.0
			usleep(2000); 
			//read the rest
			j = read(tty4_fd,&buf[i],sizeof(buf)-i); // blocking read
			
			//if the rest of the packet failed to arrive, error
			if(j+i != 16){
				#ifdef DEBUG_DSM2
				printf("error: wrong sized packet\n");
				#endif
				
				goto end;
			}
		}
		// if we've gotten here, we have a 16 byte packet
		tcflush(tty4_fd,TCIOFLUSH);
		#ifdef DEBUG_DSM2
			printf("read %d bytes, ", j+i);
		#endif
		
		#ifdef DEBUG_DSM2_RAW
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
					#ifdef DEBUG_DSM2
					printf("2048/11ms ");
					#endif
					
					mode = 11;
					goto read_packet;
				}
			}
		}
		#ifdef DEBUG_DSM2
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
					printf("dsm2 mode incorrect\n");
					goto end;
				}
				
				#ifdef DEBUG_DSM2
				printf("%d %d  ",ch_id,value);
				#endif
				
				if((ch_id+1)>9){
					#ifdef DEBUG_DSM2
					printf("error: bad channel id\n");
					#endif
					
					#ifndef DEBUG_DSM2
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
				#ifdef DEBUG_DSM2
				printf("waiting for rest of data in next packet\n");
				#endif
				break;
			}
		}
		if(is_complete){
			#ifdef DEBUG_DSM2
			printf("all data complete now\n");
			#endif
			new_dsm2_flag=1;
			resolution = mode;
			last_time = microsSinceEpoch();
			for(i=0;i<num_channels;i++){
				rc_channels[i]=new_values[i];
				new_values[i]=0;
			}
			// run the dsm2 ready function.
			// this is null unless user changed it
			dsm2_ready_func();
		}
		
		#ifdef DEBUG_DSM2
		printf("\n");
		#endif
		
end:
		// wait a tiny bit. this isn't necessary since the blocking read on 
		// the serial bus hands back resources to the cpu. However, in case
		// something goes wrong with that this prevents spinlock
		usleep(2000);
	}
	return NULL;
}


int stop_dsm2_service(){
	int ret = 0;
	
	if (running){
		running = 0; // this tells uart4_thread loop to stop
		// allow up to 1 second for thread cleanup
		struct timespec thread_timeout;
		clock_gettime(CLOCK_REALTIME, &thread_timeout);
		thread_timeout.tv_sec += 1;
		int thread_err = 0;
		thread_err = pthread_timedjoin_np(uart4_thread, NULL, &thread_timeout);
		if(thread_err == ETIMEDOUT){
			printf("WARNING: dsm2 uart4_thread exit timeout\n");
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
* int write_default_dsm2_cal_file()
*
* internal function, writes default values to the config file. Generally only
* used once the first time initialize_dsm2 is called after a clean install
*******************************************************************************/
int write_default_dsm2_cal_file(){
	int i;
	// construct a new file path string
	char file_path[100];
	strcpy (file_path, CONFIG_DIRECTORY);
	strcat (file_path, DSM2_CAL_FILE);
	
	// open for writing
	FILE* cal;
	cal = fopen(file_path, "w+");
	if (cal == NULL) {
		mkdir(CONFIG_DIRECTORY, 0777);
		cal = fopen(file_path, "w");
		if (cal == 0){
			printf("could not open config directory: ");
			printf(CONFIG_DIRECTORY);
			printf("\n");
			return -1;
		}
	}

	for(i=0;i<MAX_DSM2_CHANNELS;i++){
		fprintf(cal, "%d %d\n",DEFAULT_MIN, DEFAULT_MAX);
	}
	fflush(cal);
	fclose(cal);
	return 0;
}


/*******************************************************************************
* int calibrate_dsm2()
*
* routine for measuring the min and max values from a transmitter on each
* channel and save to disk for future use.
* If a channel isn't used by the transmitter then default values are saved.
* if the user forgot to move one of the channels during the calibration process
* then defualt values are also saved.
*******************************************************************************/
int calibrate_dsm2(){
	int i;
	
	printf("\n\nTurn on your Transmitter and connect receiver.\n");
	printf("Move all sticks and switches through their range of motion.\n");
	printf("Raw min/max positions will display below.\n");
	printf("Press Enter to save and exit.\n\n");

	
	// start listening
	listening = 1;
	pthread_t  listening_thread;
	pthread_create(&listening_thread, NULL, listen_func, (void*) NULL);
	
	// wait for user to hit enter
	while(getchar() != '\n'){
	}
	
	//stop listening
	listening=0;
	pthread_join(listening_thread, NULL);
	
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

/*******************************************************************************
* void *listen_func(void *params)
*
* this is started as a background thread by calibrate_dsm2(). Only used during
* calibration to monitor data as it comes in.
*******************************************************************************/
void *listen_func(void *params){
	//wait for data to start
	printf("waiting for dsm2 connection");
	while(!is_new_dsm2_data()){
		if(get_state()==EXITING || listening==0){
			return 0;
		}
		usleep(5000); 
	}
	
	//start limits at current value
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
