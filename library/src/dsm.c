/**
 * @file dsm.c
 *
 *
 * @author     James Strawson
 * @date       3/7/2018
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h> // for system()
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <rc/pthread.h>
#include <rc/pinmux.h>
#include <rc/time.h>
#include <rc/uart.h>
#include <rc/gpio.h>
#include <rc/dsm.h>

#define PAUSE		115	// microseconds
#define DELAY_SPEKTRUM	200000
#define DELAY_ORANGE	50000

// don't ask me why, but this is the default range for spektrum and orange
#define DEFAULT_MIN	1142
#define DEFAULT_MAX	1858

#define DSM_PIN		30	//gpio0.30	P9.11
#define DSM_UART_BUS	4
#define DSM_BAUD_RATE	115200
#define DSM_PACKET_SIZE	16

static int running;
static int rc_channels[RC_MAX_DSM_CHANNELS];
static int rc_maxes[RC_MAX_DSM_CHANNELS];
static int rc_mins[RC_MAX_DSM_CHANNELS];
static int range[RC_MAX_DSM_CHANNELS];
static int center[RC_MAX_DSM_CHANNELS];
static int num_channels; // actual number of channels being sent
static int resolution; // 10 or 11
static int new_dsm_flag;
static int dsm_frame_rate;
static uint64_t last_time;
static pthread_t parse_thread;
static int listening; // for calibration routine only
static void (*new_data_callback)();
static int rc_is_dsm_active_flag;
static int init_flag=0;


/**
 * This returns a string (char*) of '1' and '0' representing a character. For
 * example, print "00101010" with printf(__byte_to_binary(42));
 *
 * @param[in]  c     character
 *
 * @return     { description_of_the_return_value }
 */
char* __byte_to_binary(unsigned char c)
{
	static char b[9];
	unsigned char x = (unsigned char)c; //cast to unsigned
	b[0] = '\0';
	unsigned char z;
	for (z = 128; z > 0; z >>= 1){
		if(x&z) strcat(b, "1");
		else strcat(b, "0");
	}
	return b;
}

/**
 * This is a blocking function which returns 1 if the user presses ENTER. it
 * returns 0 on any other keypress. If ctrl-C is pressed it will additionally
 * set the global state to EXITITING and return -1. This is a useful function
 * for checking if the user wishes to continue with a process or quit.
 *
 * @return     { description_of_the_return_value }
 */
int __continue_or_quit()
{
	// set stdin to non-canonical raw mode to capture all button presses
	fflush(stdin);
	if(system("stty raw")!=0){
		fprintf(stderr,"ERROR in continue_or_quit setting stty raw\n");
		return -1;
	}
	int c = getchar();
	int ret;
	if(c==3){
		ret = -1;
	}
	else if(c=='\r' || c=='\n'){
		ret = 1;
	}
	else{
		ret = 0;
	}
	// put stdin back to normal canonical mode
	if(system("stty cooked")!=0){
		fprintf(stderr,"ERROR in continue_or_quit setting stty cooked\n");
		return -1;
	}
	printf("\n");
	return ret;
}

/**
 * This is a local function that is started as a background thread by
 * rc_initialize_dsm(). This monitors the serial port and interprets data for
 * each packet, it determines 10 or 11 bit resolution Radios with more than 7
 * channels split data across multiple packets. Thus, new data is not committed
 * until a full set of channel data is received.
 *
 * @param[in]  <unnamed>  { parameter_description }
 *
 * @return     { description_of_the_return_value }
 */
void* __parser_func(__attribute__ ((unused)) void* ptr){
	char buf[DSM_PACKET_SIZE];
	int i, ret, available;
	int new_values[RC_MAX_DSM_CHANNELS]; // hold new values before committing
	int detection_packets_left; // use first 4 packets just for detection
	unsigned char ch_id;
	int16_t value;
	int is_complete;
	unsigned char max_channel_id_1024 = 0; // max channel assuming 1024 decoding
	unsigned char max_channel_id_2048 = 0; // max channel assuming 2048 decoding
	char channels_detected_1024[RC_MAX_DSM_CHANNELS];
	char channels_detected_2048[RC_MAX_DSM_CHANNELS];
	memset(channels_detected_1024,0,RC_MAX_DSM_CHANNELS);
	memset(channels_detected_2048,0,RC_MAX_DSM_CHANNELS);

	/********************************************************************
	* First packets that come in are read just to detect resolution and channels
	* start assuming 1024 bit resolution, if any of the first 4 packets appear
	* to break 1024 mode then swap to 2048
	*****************************************************************/
DETECTION_START:
	rc_uart_flush(DSM_UART_BUS); // flush first
	detection_packets_left = 4;
	init_flag=1;
	while(detection_packets_left>0 && running){
		rc_usleep(5000);
		available = rc_uart_bytes_available(DSM_UART_BUS);

		// nothing yet, go back to sleep
		if(available == 0) continue;
		// halfway through packet, sleep for 1ms
		if(available <  DSM_PACKET_SIZE){
			rc_usleep(1000);
			available = rc_uart_bytes_available(DSM_UART_BUS);
		}
		// read or flush depending on bytes available
		if(available == DSM_PACKET_SIZE){
			ret = rc_uart_read_bytes(DSM_UART_BUS, buf, DSM_PACKET_SIZE);
			if(ret!=DSM_PACKET_SIZE){
				#ifdef DEBUG
					printf("WARNING: read the wrong number of bytes: %d\n", ret);
				#endif
				rc_uart_flush(DSM_UART_BUS); // flush
				continue;
			}
		}
		else{
			// got out of sync or read nonsense, flush and try again
			rc_uart_flush(DSM_UART_BUS); // flush
			continue;
		}

		// first check each channel id assuming 1024/22ms mode
		// where the channel id lives in 0b01111000 mask
		// if one doesn't make sense, must be using 2048/11ms mode
		#ifdef DEBUG
			printf("1024-mode: ");
		#endif
		for(i=1;i<8;i++){
			// last few words in buffer are often all 1's, ignore those
			if((buf[2*i]!=0xFF) || (buf[(2*i)+1]!=0xFF)){
				// grab channel id from first byte assuming 1024 mode
				ch_id = (buf[i*2]&0b01111100)>>2;
				if(ch_id>max_channel_id_1024){
					max_channel_id_1024 = ch_id;
				}
				if(ch_id<RC_MAX_DSM_CHANNELS){
					channels_detected_1024[ch_id] = 1;
				}
				#ifdef DEBUG
					printf("%d ", ch_id);
				#endif
			}
		}

		#ifdef DEBUG
			printf("   2048-mode: ");
		#endif

		for(i=1;i<8;i++){
			// last few words in buffer are often all 1's, ignore those
			if((buf[2*i]!=0xFF) || (buf[(2*i)+1]!=0xFF)){
				// now grab assuming 2048 mode
				ch_id = (buf[i*2]&0b01111000)>>3;
				if(ch_id>max_channel_id_2048){
					 max_channel_id_2048 = ch_id;
				}
				if(ch_id<RC_MAX_DSM_CHANNELS){
					channels_detected_2048[ch_id] = 1;
				}
				#ifdef DEBUG
					printf("%d ", ch_id);
				#endif
			}
		}

		// raw debug mode spits out all ones and zeros
		#ifdef DEBUG
		printf("ret=%d : ", ret);
		for(i=0; i<(DSM_PACKET_SIZE/2); i++){
			printf(__byte_to_binary(buf[2*i]));
			printf(" ");
			printf(__byte_to_binary(buf[(2*i)+1]));
			printf("   ");
		}
		printf("\n");
		#endif


		detection_packets_left --;
	}

	// do an exit check here since there is a jump above this code
	if(!running) return 0;

	/***************************************************************************
	* now determine which mode from detection data
	***************************************************************************/
	if(max_channel_id_1024 >= RC_MAX_DSM_CHANNELS){
		// probbaly 2048 if 1024 was invalid
		resolution = 2048;

		// still do some checks
		if(max_channel_id_2048 >= RC_MAX_DSM_CHANNELS){
			fprintf(stderr,"WARNING: too many DSM channels detected, trying again\n");
			goto DETECTION_START;
		}
		else num_channels = max_channel_id_2048+1;
		// now make sure every channel was actually detected up to max
		for(i=0;i<num_channels;i++){
			if(channels_detected_2048[i]==0){
				fprintf(stderr,"WARNING: Missing DSM channel, trying again\n");
				goto DETECTION_START;
			}
		}
	}
	// if not 2048, must be 1024 but do some checks anyway
	else{
		resolution = 1024;

		// still do some checks
		if(max_channel_id_1024 >= RC_MAX_DSM_CHANNELS){
			fprintf(stderr,"WARNING: too many DSM channels detected, trying again\n");
			goto DETECTION_START;
		}
		else num_channels = max_channel_id_1024+1;
		// now make sure every channel was actually detected up to max
		for(i=0;i<num_channels;i++){
			if(channels_detected_1024[i]==0){
				fprintf(stderr,"WARNING: Missing DSM channel, trying again\n");
				goto DETECTION_START;
			}
		}
	}

	/***************************************************************************
	* normal operation loop
	***************************************************************************/
START_NORMAL_LOOP:
	while(running){
		rc_usleep(5000);
		available = rc_uart_bytes_available(DSM_UART_BUS);

		// nothing yet, go back to sleep
		if(available == 0) continue;
		// halfway through packet, sleep for 1ms
		if(available <  DSM_PACKET_SIZE){
			rc_usleep(1000);
			available = rc_uart_bytes_available(DSM_UART_BUS);
		}

		#ifdef DEBUG
			fprintf(stderr,"bytes available: %d\n", available);
		#endif

		// read or flush depending on bytes available
		if(available == DSM_PACKET_SIZE){
			ret = rc_uart_read_bytes(DSM_UART_BUS, buf, DSM_PACKET_SIZE);
			if(ret!=DSM_PACKET_SIZE){
				#ifdef DEBUG
					fprintf(stderr,"WARNING: read the wrong number of bytes: %d\n", ret);
				#endif
				rc_is_dsm_active_flag=0;
				rc_uart_flush(DSM_UART_BUS); // flush
				continue;
			}
		}
		else{
			// got out of sync, flush and try again
			rc_uart_flush(DSM_UART_BUS); // flush
			continue;
		}

		// raw debug mode spits out all ones and zeros
		#ifdef DEBUG
		pfprintf(stderr,"ret=%d : ", ret);
		for(i=0; i<(DSM_PACKET_SIZE/2); i++){
			fprintf(stderr,__byte_to_binary(buf[2*i]));
			fprintf(stderr," ");
			fprintf(stderr,__byte_to_binary(buf[(2*i)+1]));
			fprintf(stderr,"   ");
		}
		fprintf(stderr,"\n");
		#endif


		// okay, must have a full packet now
		// packet is 16 bytes, 8 words long
		// first word doesn't have channel data, so iterate through last 7 words
		for(i=1;i<=7;i++){
			// unused words are 0xFF
			// skip if one of them
			if(buf[2*i]!=0xFF || buf[(2*i)+1]!=0xFF){
				// grab channel id from first byte
				// and value from both bytes
				if(resolution == 1024){
					ch_id = (buf[i*2]&0b01111100)>>2;
					// grab value from least 11 bytes
					value = ((buf[i*2]&0b00000011)<<8) + buf[(2*i)+1];
					value += 989; // shift range so 1500 is neutral
				}
				else if(resolution == 2048){
					ch_id = (buf[i*2]&0b01111000)>>3;
					// grab value from least 11 bytes
					value = ((buf[i*2]&0b00000111)<<8) + buf[(2*i)+1];
					// extra bit of precision means scale is off by factor of
					// two, also add 989 to center channels around 1500
					value = (value/2) + 989;
				}
				else{
					fprintf(stderr,"ERROR: dsm resolution incorrect\n");
					return NULL;
				}

				#ifdef DEBUG
				fprintf(stderr,"%d %d  ",ch_id,value);
				#endif

				if((ch_id+1)>RC_MAX_DSM_CHANNELS){
					#ifdef DEBUG
					fprintf(stderr,"ERROR in DSM background service, received bad channel id\n");
					#endif
					goto START_NORMAL_LOOP;
				}
				// record new value
				new_values[ch_id] = value;
			}
		}

		// check if a complete set of channel data has been received
		// otherwise wait for another packet with more data
		is_complete = 1;
		for(i=0;i<num_channels;i++){
			if (new_values[i]==0){
				is_complete=0;
				#ifdef DEBUG
				fprintf(stderr,"waiting for rest of data in next packet\n");
				#endif
				break;
			}
		}
		if(is_complete){
			#ifdef DEBUG
			fprintf(stderr,"all data complete now\n");
			#endif
			new_dsm_flag=1;
			rc_is_dsm_active_flag=1;
			last_time = rc_nanos_since_boot();
			for(i=0;i<num_channels;i++){
				rc_channels[i]=new_values[i];
				new_values[i]=0;// put local values array back to 0
			}
			// run the dsm ready function.
			// this is null unless user changed it
			if(new_data_callback!=NULL) new_data_callback();
		}

		#ifdef DEBUG
		printf("\n");
		#endif
	}
	return NULL;
}

/**
 * this is started as a background thread by rc_dsm_calibrate_routine(). Only
 * used during calibration to monitor data as it comes in.
 *
 * @return     NULL
 */
void* __calibration_listen_func(__attribute__ ((unused)) void *ptr)
{
	int j, raw;
	//wait for data to start
	printf("waiting for dsm connection");
	rc_dsm_ch_raw(1); // flush the data ready flag with a read
	while(!rc_dsm_is_new_data()){
		if(listening==0) return NULL;
		rc_usleep(5000);
	}

	//start limits at first value
	for(j=0;j<RC_MAX_DSM_CHANNELS;j++){
		rc_mins[j]=rc_dsm_ch_raw(j+1);
		rc_maxes[j]=rc_dsm_ch_raw(j+1);
	}

	// record limits until user presses enter
	while(listening){
		printf("\r");
		if(rc_dsm_is_new_data()){
			for(j=0;j<RC_MAX_DSM_CHANNELS;j++){
				raw = rc_dsm_ch_raw(j+1);
				//record only non-zero channels
				if(raw > 0){
					if(raw>rc_maxes[j]){
						rc_maxes[j] = raw;
					}
					else if(raw<rc_mins[j]){
						rc_mins[j] = raw;
					}
					printf("%d:%d ",j+1,raw);
				}
			}
			fflush(stdout);
		}
		rc_usleep(10000);
	}
	return 0;
}


int rc_dsm_init()
{
	int i;
	//if calibration file exists, load it and start spektrum thread
	FILE* fd;

	// open for reading
	fd = fopen(RC_DSM_CALIBRATION_FILE, "r");

	if(fd==NULL){
		fprintf(stderr,"\ndsm Calibration File Doesn't Exist Yet\n");
		fprintf(stderr,"Run calibrate_dsm example to create one\n");
		fprintf(stderr,"Using default values for now\n");
		for(i=0;i<RC_MAX_DSM_CHANNELS;i++){
			rc_mins[i]=DEFAULT_MIN;
			rc_maxes[i]=DEFAULT_MAX;
		}
	}
	else{
		for(i=0;i<RC_MAX_DSM_CHANNELS;i++){
			if(fscanf(fd,"%d %d", &rc_mins[i],&rc_maxes[i])!=2){
				perror("ERROR in rc_dsm_init reading calibration data");
				return -1;
			}
			range[i] = rc_maxes[i]-rc_mins[i];
			center[i] = (rc_maxes[i]+rc_mins[i])/2.0;
		}
		#ifdef DEBUG
		printf("DSM Calibration Loaded\n");
		#endif
		fclose(fd);
	}

	if(rc_pinmux_set(DSM_PIN, PINMUX_UART)){
		fprintf(stderr,"ERROR in rc_dsm_init, failed to set pinmux\n");
		return -1;
	}

	dsm_frame_rate = 0; // zero until mode is detected on first packet
	running = 1; // lets uarts 4 thread know it can run
	num_channels = 0;
	last_time = 0;
	rc_is_dsm_active_flag = 0;
	rc_dsm_set_callback(NULL);

	// 0.5s timeout disable canonical (0), 1 stop bit (1), disable parity (0)
	if(rc_uart_init(DSM_UART_BUS, DSM_BAUD_RATE, 0.5, 0, 1, 0)){
		fprintf(stderr,"ERROR in rc_dsm_init, failed to init uart bus\n");
		return -1;
	}

	if(rc_pthread_create(&parse_thread, __parser_func, NULL, SCHED_OTHER, 0)){
		fprintf(stderr,"ERROR in rc_dsm_init, failed to start thread\n");
		return -1;
	}

	#ifdef DEBUG
	printf("dsm Thread Started\n");
	#endif

	rc_usleep(10000); // let thread start
	return 0;
}


int rc_dsm_cleanup()
{
	int ret;
	// just return if not running
	if(!running){
		init_flag;
		return 0;
	}
	// tell parser loop to stop
	running = 0;
	// allow up to 1 second for thread cleanup
	ret=rc_pthread_timed_join(parse_thread,NULL,1.0);
	if(ret==-1){
		fprintf(stderr,"ERORR in rc_dsm_cleanup, problem joining thread for pin\n");
	}
	else if(ret==1){
		fprintf(stderr,"ERROR in rc_dsm_cleanup, thread exit timeout\n");
		fprintf(stderr,"most likely cause is your callback function is stuck and didn't return\n");
	}
	init_flag=0;
	return ret;
}


int rc_dsm_ch_raw(int ch)
{
	if(init_flag==0){
		fprintf(stderr,"ERROR in rc_dsm_ch_raw, call rc_dsm_init first\n");
		return -1;
	}
	if(ch<1 || ch>RC_MAX_DSM_CHANNELS){
		fprintf(stderr,"ERROR in rc_dsm_ch_raw channel must be between 1 & %d",RC_MAX_DSM_CHANNELS);
		return -1;
	}
	new_dsm_flag = 0;
	return rc_channels[ch-1];
}


float rc_dsm_ch_normalized(int ch)
{
	if(init_flag==0){
		fprintf(stderr,"ERROR in rc_dsm_ch_normalized, call rc_dsm_init first\n");
		return -1.0;
	}
	if(ch<1 || ch>RC_MAX_DSM_CHANNELS){
		fprintf(stderr,"ERROR in rc_dsm_ch_raw channel must be between 1 & %d",RC_MAX_DSM_CHANNELS);
		return -1.0;
	}
	if(range!=0 && rc_channels[ch-1]!=0) {
		new_dsm_flag = 0;
		return 2*(rc_channels[ch-1]-center[ch-1])/range[ch-1];
	}
	return 0;
}


int rc_dsm_is_new_data()
{
	if(init_flag==0){
		fprintf(stderr,"ERROR in rc_dsm_is_new_data, call rc_dsm_init first\n");
		return 0;
	}
	return new_dsm_flag;
}


void rc_dsm_set_callback(void (*func)(void))
{
	if(init_flag==0){
		fprintf(stderr,"ERROR in rc_dsm_set_callback, call rc_dsm_init first\n");
	}
	new_data_callback = func;
	return;
}


int rc_dsm_is_connection_active()
{
	if(init_flag==0){
		fprintf(stderr,"ERROR in rc_dsm_is_connection_active, call rc_dsm_init first\n");
		return 0;
	}
	return rc_is_dsm_active_flag;
}


int64_t rc_dsm_nanos_since_last_packet(){
	if(init_flag==0){
		fprintf(stderr,"ERROR in rc_dsm_nanos_since_last_packet, call rc_dsm_init first\n");
		return -1;
	}
	// if global variable last_time ==0 then no packet must have arrived yet
	if(last_time==0) return -1;
	return rc_nanos_since_boot()-last_time;
}


int rc_dsm_resolution()
{
	if(init_flag==0){
		fprintf(stderr,"ERROR in rc_dsm_resolution, call rc_dsm_init first\n");
		return -1;
	}
	return resolution;
}


int rc_dsm_channels()
{
	if(init_flag==0){
		fprintf(stderr,"ERROR in rc_dsm_channels, call rc_dsm_init first\n");
		return -1;
	}
	return num_channels;
}





int rc_dsm_bind_routine()
{
	int value, delay, i;
	char c = 0; // for reading user input
	// default to dsmx 11ms mode for most applications
	int pulses = 9;

	// first set the pin as input (pulldown) to detect when receiver is attached
	if(rc_pinmux_set(DSM_PIN, PINMUX_GPIO_PD)<0){
		fprintf(stderr,"ERROR in rc_dsm_bind_routine, pinmux helper not enabled for P9_11\n");
		return -1;
	}

	// configure gpio pin as input
	if(rc_gpio_init(DSM_PIN, GPIOHANDLE_REQUEST_INPUT)==-1){
		fprintf(stderr,"ERROR in rc_dsm_bind_routine initializing gpio pin %d as input\n", DSM_PIN);
		return -1;
	}

	// give user instructions
	printf("\n\nYou must choose which DSM mode to request from your transmitter\n");
	printf("Note that your transmitter may actually bind in a different mode\n");
	printf("depending on how it is configured.\n");
	printf("We suggest option 1 for 6-channel dsm radios,\n");
	printf("option 4 for 7-9 channel DSMX radios\n");
	printf("and option 5 for 6-channel Orange and JR radios\n");
	printf("\n");
	printf("1: Spektrum  DSM2 10-bit 22ms framerate\n");
	printf("2: Spektrum  DSM2 11-bit 11ms framerate\n");
	printf("3: Spektrum  DSMX 10-bit 22ms framerate\n");
	printf("4: Spektrum  DSMX 11-bit 11ms framerate\n");
	printf("5: Orange/JR DSM2 10-bit 22ms framerate\n");
	printf("\n");
	printf("Enter mode 1-5: ");

	// wait for user input
enter:
	c = getchar();

	switch(c){
		case '1':
			pulses = 3;
			delay = DELAY_SPEKTRUM;
			break;
		case '2':
			pulses = 5;
			delay = DELAY_SPEKTRUM;
			break;
		case '3':
			pulses = 7;
			delay = DELAY_SPEKTRUM;
			break;
		case '4':
			pulses = 9;
			delay = DELAY_SPEKTRUM;
			break;
		case '5':
			pulses = 9;
			delay = DELAY_ORANGE;
			break;
		case '\n':
			goto enter;
			break;
		default:
			fprintf(stderr, "invalid entry, try again\n");
			getchar();
			goto enter;
			break;
	}

	printf("Using mode %c\n", c);

	// wait for user to hit enter before continuing
	printf("\nDisconnect your dsm satellite receiver if it is still connected\n");
	printf("Plug it into the cape quickly and firmly to begin binding.\n");

	// wait for the receiver to be disconnected
	value = 1;
	while(value==1){ //pin will go low when disconnected
		value=rc_gpio_get_value(DSM_PIN);
		if(value==-1){
			fprintf(stderr,"ERROR in rc_dsm_bind_routine, failed to read gpio value\n");
			return -1;
		}
		rc_usleep(500);
	}
	rc_usleep(100000);

	//wait for the receiver to be plugged in
	//receiver will pull pin up when connected
	while(value==0){
		value=rc_gpio_get_value(DSM_PIN);
		rc_usleep(500);
	}

	// now configure gpio pin as output
	rc_gpio_cleanup(DSM_PIN);
	if(rc_gpio_init(DSM_PIN, GPIOHANDLE_REQUEST_OUTPUT)==-1){
		fprintf(stderr,"ERROR in rc_dsm_bind_routine initializing gpio pin %d as output\n", DSM_PIN);
		return -1;
	}
	if(rc_gpio_set_value(DSM_PIN, 1)==-1){
		fprintf(stderr,"ERROR in rc_dsm_bind_routine, failed to write to gpio\n");
		return -1;
	}

	// wait as long as possible before sending pulses
	// in case the user plugs in the receiver slowly at an angle
	// which would delay the power pin from connecting
	rc_usleep(delay);

	for(i=0; i<pulses; i++){
		rc_gpio_set_value(DSM_PIN, 0);
		rc_usleep(PAUSE);
		rc_gpio_set_value(DSM_PIN, 1);
		rc_usleep(PAUSE);
	}

	rc_usleep(1000000);

	// swap pinmux from GPIO back to uart
	if(rc_pinmux_set(DSM_PIN, PINMUX_UART)){
		fprintf(stderr,"ERROR in rc_dsm_bind_routine, failed to put pin back to UART mode\n");
		return -1;
	}

	// all done
	printf("\n\n\nYour receiver should now be blinking. If not try again.\n");
	printf("This is a finicky process and may require a few attempts, don't be discouraged!\n\n");
	printf("If the receiver LED is blinking, turn on your transmitter in bind mode.\n");
	printf("After binding completes use rc_test_dsm to confirm functionality.\n\n");

	return 0;
}


int rc_dsm_calibrate_routine()
{
	int i,ret;
	FILE* fd;

	dsm_frame_rate = 0; // zero until mode is detected on first packet
	running = 1; // lets uarts 4 thread know it can run
	num_channels = 0;
	last_time = 0;
	rc_is_dsm_active_flag = 0;
	new_data_callback=NULL;

	// make sure directory and calibration file exist and are writable first
	ret = mkdir(RC_DSM_CALIBRATION_DIR, 0777);
	// error check, EEXIST is okay, we want directory to exist!
	if(ret==-1 && errno!=EEXIST){
		perror("ERROR in rc_dsm_calibration_routine making calibration file directory");
		return -1;
	}
	fd = fopen(RC_DSM_CALIBRATION_FILE, "w+");
	if(fd == NULL){
		perror("ERROR in rc_dsm_calibration_routine opening calibration file for writing");
		return -1;
	}
	fclose(fd);


	// 0.5s timeout disable canonical (0), 1 stop bit (1), disable parity (0)
	if(rc_uart_init(DSM_UART_BUS, DSM_BAUD_RATE, 0.5, 0, 1, 0)){
		fprintf(stderr,"ERROR in rc_dsm_calibrate_routine, failed to init uart bus\n");
		return -1;
	}

	pthread_create(&parse_thread, NULL, __parser_func, (void*) NULL);

	// display instructions
	printf("\nRaw dsm data should display below if the transmitter and\n");
	printf("receiver are paired and working. Move all channels through\n");
	printf("their range of motion and the minimum and maximum values will\n");
	printf("be recorded. When you are finished moving all channels,\n");
	printf("press ENTER to save the data or any other key to abort.\n\n");

	// start listening
	listening = 1;
	pthread_t  listening_thread;
	pthread_create(&listening_thread, NULL, __calibration_listen_func, (void*) NULL);

	// wait for user to hit enter
	ret = __continue_or_quit();

	// stop listening
	listening=0;
	pthread_join(listening_thread, NULL);


	// abort if user hit something other than enter
	if(ret<0){
		fprintf(stderr,"aborting calibrate_dsm routine\n");
		return -1;
	}

	// if it looks like no new data came in exit
	if((rc_mins[0]==0) || (rc_mins[0]==rc_maxes[0])){
		fprintf(stderr,"no new data recieved, exiting\n");
		return -1;
	}

	// open for writing
	fd = fopen(RC_DSM_CALIBRATION_FILE, "w");
	if(fd == NULL){
		perror("ERROR in rc_dsm_calibration_routine opening calibration file for writing");
		return -1;
	}

	// if new data was captures for a channel, write data to cal file
	// otherwise fill in defaults for unused channels in case
	// a higher channel radio is used in the future with this cal file
	for(i=0;i<RC_MAX_DSM_CHANNELS;i++){
		if((rc_mins[i]==0) || (rc_mins[i]==rc_maxes[i])){
			fprintf(fd, "%d %d\n",DEFAULT_MIN, DEFAULT_MAX);
		}
		else{
			fprintf(fd, "%d %d\n", rc_mins[i], rc_maxes[i]);
		}
	}
	fclose(fd);
	printf("New calibration file written\n");
	printf("use rc_test_dsm to confirm\n");
	return 0;
}

