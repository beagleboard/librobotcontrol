/**
 * @file sbus.c
 *
 *	       Description of the SBUS protocol can be found here:
 *	       https://github.com/uzh-rpg/rpg_quadrotor_control/wiki/SBUS-Protocol
 *
 *	       On the X4R-SB board, there is an extra 4-pin connector
 *	       (P6). The DSM2 connector should be connected as follows:
 *	       BBB.DSM2.pin.3 (SBUS_TX) -> X4R.P6.pin.3 (UART4_RX)
 *	       BBB.DSM2.pin.4 (GND)     -> X4R.P6.pin.2 (GND)
 *
 *	       Additional description of how to get non-inverted UART
 *	       signal from a X4R-SB (X4R with SBUS) receiver can be
 *	       found here:
 *	       https://www.getfpv.com/learn/fpv-diy-repairs-and-mods/get-uninverted-sbus-frsky-receivers/
 *
 *	       The X4R-SB cannot run reliabily from 3.3V but can be
 *	       fed from 5V or 6V servo rails. Run a servo cable
 *	       (without signal wire) from BBB servo-rails to any port
 *	       on the X4R - NOTE: be careful to connect correctly.
 *
 *	       Based on James Strawson's DSM code.
 *
 * @author     Per Dalgas Jakobsen
 * @date       2019-03-04
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h> // for system()
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <math.h>
#include <rc/pthread.h>
#include <rc/pinmux.h>
#include <rc/time.h>
#include <rc/uart.h>
#include <rc/gpio.h>
#include "common.h"

#ifdef RC_AUTOPILOT_EXT
#include "../include/rc/sbus.h"
#else
#include <rc/sbus.h>
#endif

#define SBUS_CALIBRATION_FILE	"sbus.cal"

#define PAUSE		115	// microseconds
#define TOL		0.0001

// don't ask me why, but this is the default range for spektrum and orange
#define DEFAULT_MIN	1142
#define DEFAULT_MAX	1858
#define DEFAULT_CENTER	1500

#define SBUS_PINMUX_ID	30
#define SBUS_PIN	0,30	//gpio0.30	P9.11
#define SBUS_UART_BUS	4
#define SBUS_BAUD_RATE	100000
#define SBUS_PACKET_SIZE	25
#define UART_TIMEOUT_S	0.2
#define CONNECTION_LOST_TIMEOUT_NS 300000000

#define SBUS_HEADER_OFS	0
#define SBUS_CHANNEL_DATA_OFS	1
#define SBUS_FLAGS_OFS	23
#define SBUS_FOOTER_OFS	24

#define SBUS_FLAG_CH17_MASK	0x01
#define SBUS_FLAG_CH18_MASK	0x02
#define SBUS_FLAG_FRAME_LOST_MASK	0x04
#define SBUS_FLAG_FAILSAFE_MASK	0x08

#define SBUS_HEADER_VALUE	0x0f
#define SBUS_FOOTER_VALUE	0x00


static volatile int running;
static volatile int analog_channels[RC_MAX_SBUS_ANALOG_CHANNELS];
static volatile int binary_channels[RC_MAX_SBUS_BINARY_CHANNELS];
static volatile int frame_lost = 1;
static volatile int failsafe = 1;

static volatile int new_sbus_flag;
static volatile uint64_t last_time;
static volatile int listening; // for calibration routine only
static volatile int active_flag=0;
static volatile int init_flag=0;
static volatile uint32_t total_errors=0;
static volatile uint32_t lost_frames=0;
static volatile uint32_t signal_quality=0;

static volatile int maxes[RC_MAX_SBUS_ANALOG_CHANNELS];
static volatile int mins[RC_MAX_SBUS_ANALOG_CHANNELS];
static volatile int centers[RC_MAX_SBUS_ANALOG_CHANNELS];
static volatile double range_up[RC_MAX_SBUS_ANALOG_CHANNELS];
static volatile double range_down[RC_MAX_SBUS_ANALOG_CHANNELS];

static void (*new_data_callback)();
static void (*disconnect_callback)();

static pthread_t parse_thread;


/**
 * This returns a string (char*) of '1' and '0' representing a character. For
 * example, print "00101010" with printf(__byte_to_binary(42));
 *
 * @param[in]  c     character
 *
 * @return     { description_of_the_return_value }
 */
#ifdef DEBUG
static char* __byte_to_binary(unsigned char c)
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
#endif

/**
 * This is a blocking function which returns 1 if the user presses ENTER. it
 * returns 0 on any other keypress. If ctrl-C is pressed it will additionally
 * set the global state to EXITITING and return -1. This is a useful function
 * for checking if the user wishes to continue with a process or quit.
 *
 * @return     { description_of_the_return_value }
 */
static int __continue_or_quit(void)
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
 * This is a local function which given an SBUS-packet and channel ID,
 * returns the value of that channel. Channel data is encoded as 16 x
 * 11-bits.
 *
 * @return     { channel value }
 */
static int get_channel_value (uint8_t buf[], int channel)
{
	const int bit_number = channel * 11;
	const int first_byte = SBUS_CHANNEL_DATA_OFS + (bit_number / 8);
	const int first_bit  = bit_number % 8;

	uint32_t value;
	value = buf [first_byte];
	value |= ((uint32_t)buf [first_byte + 1]) << 8;
	if (first_bit + 11 > 2*8) {
		/* Channel is spread over 3 data bytes. */
		value |= ((uint32_t)buf [first_byte + 2]) << 16;
	}

	return (value >> first_bit) & 0x7ff;
}


/**
 * This is a local function that is started as a background thread by
 * rc_initialize_sbus(). This monitors the serial port and interprets data for
 * each packet.
 *
 * @param[in]  <unnamed>  { parameter_description }
 *
 * @return     { description_of_the_return_value }
 */
static void* __parser_func(__attribute__ ((unused)) void* ptr){
	uint8_t buf[SBUS_PACKET_SIZE];
	int i, ret;
	int16_t value;

	new_sbus_flag=0;
	init_flag=1;

	while (running) {
		// check for timeouts
		if (active_flag != 0 && rc_sbus_nanos_since_last_packet() > CONNECTION_LOST_TIMEOUT_NS) {
			active_flag = 0;
			if (disconnect_callback!=NULL) disconnect_callback();
		}

		if (signal_quality > 0) {
			signal_quality--;
		}

		memset (buf, 0, SBUS_PACKET_SIZE);
		rc_uart_flush (SBUS_UART_BUS); // flush
		ret = rc_uart_read_bytes (SBUS_UART_BUS, buf, SBUS_PACKET_SIZE);
		if (ret != SBUS_PACKET_SIZE) {
			#ifdef DEBUG
				fprintf(stderr, "WARNING: read the wrong number of bytes: %d\n", ret);
			#endif
			rc_uart_flush (SBUS_UART_BUS); // flush
			total_errors++;
			lost_frames++;
			continue;
		}

		if ((buf [SBUS_HEADER_OFS] != SBUS_HEADER_VALUE) ||
		    (buf [SBUS_FOOTER_OFS] != SBUS_FOOTER_VALUE)) {
			#ifdef DEBUG
			fprintf(stderr, "WARNING: Header/Footer incorrect: %x, %x\n",
				buf [SBUS_HEADER_OFS], buf [SBUS_FOOTER_OFS]);
			#endif
			rc_uart_flush (SBUS_UART_BUS); // flush
			total_errors++;
			lost_frames++;
			continue;
		}

		// raw debug mode spits out all ones and zeros
		#ifdef DEBUG
		fprintf (stderr, "ret=%d : ", ret);
		for (i = 0; i < (SBUS_PACKET_SIZE / 2); i++){
			fprintf (stderr, __byte_to_binary (buf[2*i]));
			fprintf (stderr, " ");
			fprintf (stderr, __byte_to_binary (buf[(2*i)+1]));
			fprintf (stderr, "   ");
		}
		fprintf (stderr, "\n");
		#endif

		// Get all "analog" channels
		for (i = 0; i < RC_MAX_SBUS_ANALOG_CHANNELS; i++){
			value = get_channel_value (buf, i);
                        #ifdef DEBUG
			fprintf (stderr,"%d %d  ",i ,value);
                        #endif

			// record new value
			analog_channels [i] = value;
		}

		// Get all "binary" channels
		binary_channels [0] = buf [SBUS_FLAGS_OFS] & SBUS_FLAG_CH17_MASK ? 1 : 0;
		binary_channels [1] = buf [SBUS_FLAGS_OFS] & SBUS_FLAG_CH18_MASK ? 1 : 0;
                #ifdef DEBUG
		fprintf (stderr,"17 %d  18 %d\n", binary_channels [0], binary_channels [1]);
                #endif

		frame_lost = buf [SBUS_FLAGS_OFS] & SBUS_FLAG_FRAME_LOST_MASK ? 1 : 0;
		failsafe   = buf [SBUS_FLAGS_OFS] & SBUS_FLAG_FAILSAFE_MASK ? 1 : 0;
                #ifdef DEBUG
		fprintf (stderr,"frame_lost %d  failsafe %d\n", frame_lost, failsafe);
                #endif

		if (frame_lost) {
			lost_frames++;
		} else {
			if (signal_quality >= 99) {
				signal_quality = 100;
			} else {
				// Increment and compensate for initial decrement.
				signal_quality += 2;
			}
		}

		new_sbus_flag=1;
		active_flag=1;
		last_time = rc_nanos_since_boot();

		// run the dsm ready function.
		// this is null unless user changed it
		if(new_data_callback!=NULL) new_data_callback();

                #ifdef DEBUG
		printf("\n");
		#endif
	}
	return NULL;
}

/**
 * this is started as a background thread by rc_sbus_calibrate_routine(). Only
 * used during calibration to monitor data as it comes in.
 *
 * @return     NULL
 */
static void* __calibration_listen_func(__attribute__ ((unused)) void *ptr)
{
	int j, raw;
	//wait for data to start
	printf("waiting for sbus connection");
	new_sbus_flag =0; // flush the data ready flag with a read
	while(!rc_sbus_is_new_data()){
		if(listening==0) return NULL;
		rc_usleep(5000);
	}

	//start limits at first value
	for(j=0;j<RC_MAX_SBUS_ANALOG_CHANNELS;j++){
		mins[j]=analog_channels[j];
		maxes[j]=analog_channels[j];
	}

	// record limits until user presses enter
	while(listening){
		printf("\r");
		if(rc_sbus_is_new_data()){
			for(j=0;j<RC_MAX_SBUS_ANALOG_CHANNELS;j++){
				raw = analog_channels[j];
				//record only non-zero analog channels
				if(raw > 0){
					if(raw>maxes[j]){
						maxes[j] = raw;
					}
					else if(raw<mins[j]){
						mins[j] = raw;
					}
					printf("Ch%d=%4d ",j+1,raw);
				}
			}
			printf("Ch17=%d Ch18=%d",binary_channels[0],binary_channels[1]);
			fflush(stdout);
		}
		rc_usleep(10000);
	}

	// record zeros
	for(j=0;j<RC_MAX_SBUS_ANALOG_CHANNELS;j++){
		raw = analog_channels[j];
		//record only non-zero analog channels
		if(raw > 0) centers[j]=raw;
	}


	return 0;
}


int rc_sbus_init(void)
{
	int i;
	//if calibration file exists, load it and start spektrum thread
	FILE* fd;

	// open for reading
	fd = fopen(CALIBRATION_DIR SBUS_CALIBRATION_FILE, "r");

	if(fd==NULL){
		fprintf(stderr,"\nsbus Calibration File Doesn't Exist Yet\n");
		fprintf(stderr,"Run calibrate_sbus example to create one\n");
		fprintf(stderr,"Using default values for now\n");
		for(i=0;i<RC_MAX_SBUS_ANALOG_CHANNELS;i++){
			mins[i]=DEFAULT_MIN;
			maxes[i]=DEFAULT_MAX;
			centers[i]=DEFAULT_CENTER;
		}
	}
	else{
		for(i=0;i<RC_MAX_SBUS_ANALOG_CHANNELS;i++){
			if(fscanf(fd,"%d %d %d", &mins[i],&maxes[i],&centers[i])!=3){
				fprintf(stderr, "ERROR in rc_sbus_init reading calibration data\n");
				fprintf(stderr, "Malformed calibration file, deleting and using defaults\n");
				//fclose(fd);
				remove(CALIBRATION_DIR SBUS_CALIBRATION_FILE);
				for(i=0;i<RC_MAX_SBUS_ANALOG_CHANNELS;i++){
					mins[i]=DEFAULT_MIN;
					maxes[i]=DEFAULT_MAX;
					centers[i]=DEFAULT_CENTER;
				}
				break;
			}
		}
		#ifdef DEBUG
		printf("SBUS Calibration Loaded\n");
		#endif
		fclose(fd);
	}

	// configure range and center for future use
	for(i=0;i<RC_MAX_SBUS_ANALOG_CHANNELS;i++){
		// check for two-position switch and throttle modes
		if((centers[i]<(mins[i]+50)  && centers[i]>(mins[i]-50)) || \
		   (centers[i]<(maxes[i]+50) && centers[i]>(maxes[i]-50))){
			centers[i] = mins[i];
			range_up[i] = maxes[i]-mins[i];
			range_down[i] = maxes[i]-mins[i];
		}
		// otherwise normal mode
		else{
			range_up[i] = maxes[i]-centers[i];
			range_down[i] = centers[i]-mins[i];
		}
		#ifdef DEBUG
		printf("channel %d range %f center %d\n", i, range_up[i],centers[i]);
		#endif
	}

	if(rc_pinmux_set(SBUS_PINMUX_ID, PINMUX_UART)){
		fprintf(stderr,"ERROR in rc_sbus_init, failed to set pinmux\n");
		return -1;
	}

	running = 1; // lets uarts 4 thread know it can run
	last_time = 0;
	active_flag = 0;
	new_data_callback=NULL;
	disconnect_callback=NULL;
	new_sbus_flag=0;
	total_errors=0;
	lost_frames=0;
	signal_quality=0;

	// 0.2s timeout, disable canonical (0), 2 stop bit (1), even parity (0)
	if(rc_uart_init(SBUS_UART_BUS, SBUS_BAUD_RATE, UART_TIMEOUT_S, 0, 2, 1)){
		fprintf(stderr,"ERROR in rc_sbus_init, failed to init uart bus\n");
		return -1;
	}

	if(rc_pthread_create(&parse_thread, __parser_func, NULL, SCHED_OTHER, 0)){
		fprintf(stderr,"ERROR in rc_sbus_init, failed to start thread\n");
		return -1;
	}

	#ifdef DEBUG
	printf("sbus Thread created\n");
	#endif

	// wait for thread to start
	for(i=0;i<20;i++){
		rc_usleep(20000);
		if(init_flag) break;
	}
	return 0;
}


int rc_sbus_cleanup(void)
{
	int ret;
	// just return if not running
	if(!running){
	init_flag=0;
		return 0;
	}
	// tell parser loop to stop
	running = 0;
	// allow up to 1 second for thread cleanup
	ret=rc_pthread_timed_join(parse_thread,NULL,1.0);
	if(ret==-1){
		fprintf(stderr,"ERORR in rc_sbus_cleanup, problem joining thread for pin\n");
	}
	else if(ret==1){
		fprintf(stderr,"ERROR in rc_sbus_cleanup, thread exit timeout\n");
		fprintf(stderr,"most likely cause is your callback function is stuck and didn't return\n");
	}
	init_flag=0;
	return ret;
}


int rc_sbus_ch_raw(int ch)
{
	if(init_flag==0){
		fprintf(stderr,"ERROR in rc_sbus_ch_raw, call rc_sbus_init first\n");
		return -1;
	}
	if(ch<1 || ch>RC_MAX_SBUS_ANALOG_CHANNELS){
		fprintf(stderr,"ERROR in rc_sbus_ch_raw channel must be between 1 & %d",RC_MAX_SBUS_ANALOG_CHANNELS);
		return -1;
	}
	new_sbus_flag = 0;
	return analog_channels[ch-1];
}


int rc_sbus_ch_binary(int ch)
{
	if(init_flag==0){
		fprintf(stderr,"ERROR in rc_sbus_ch_binary, call rc_sbus_init first\n");
		return -1;
	}
	if(ch<1 || ch>RC_MAX_SBUS_BINARY_CHANNELS){
		fprintf(stderr,"ERROR in rc_sbus_ch_binary channel must be between 1 & %d",RC_MAX_SBUS_BINARY_CHANNELS);
		return -1;
	}
	new_sbus_flag = 0;
	return binary_channels[ch-1];
}


double rc_sbus_ch_normalized(int ch)
{
	if(init_flag==0){
		fprintf(stderr,"ERROR in rc_sbus_ch_normalized, call rc_sbus_init first\n");
		return -1.0;
	}
	if(ch<1 || ch>RC_MAX_SBUS_ANALOG_CHANNELS){
		fprintf(stderr,"ERROR in rc_sbus_ch_raw channel must be between 1 & %d",RC_MAX_SBUS_ANALOG_CHANNELS);
		return -1.0;
	}
	// return 0 if there was a weird condition
	if(fabs(range_up[ch-1]) < TOL || fabs(range_down[ch-1]) < TOL || analog_channels[ch-1]==0) return 0.0f;

	// mark data as read
	new_sbus_flag = 0;

	if(analog_channels[ch-1]==centers[ch-1]) return 0.0;
	if(analog_channels[ch-1]>centers[ch-1]) return (analog_channels[ch-1]-centers[ch-1])/range_up[ch-1];
	return (analog_channels[ch-1]-centers[ch-1])/range_down[ch-1];
}


int rc_sbus_is_new_data(void)
{
	if(init_flag==0){
		fprintf(stderr,"ERROR in rc_sbus_is_new_data, call rc_sbus_init first\n");
		return 0;
	}
	return new_sbus_flag;
}


void rc_sbus_set_callback(void (*func)(void))
{
	if(init_flag==0){
		fprintf(stderr,"ERROR in rc_sbus_set_callback, call rc_sbus_init first\n");
	}
	new_data_callback = func;
	return;
}

void rc_sbus_set_disconnect_callback(void (*func)(void))
{
	if(init_flag==0){
		fprintf(stderr,"ERROR in rc_sbus_set_disconnect_callback, call rc_sbus_init first\n");
	}
	disconnect_callback = func;
	return;
}


int rc_sbus_is_connection_active(void)
{
	if(init_flag==0){
		fprintf(stderr,"ERROR in rc_sbus_is_connection_active, call rc_sbus_init first\n");
		return 0;
	}
	return active_flag;
}


int64_t rc_sbus_nanos_since_last_packet(void){
	if(init_flag==0){
		fprintf(stderr,"ERROR in rc_sbus_nanos_since_last_packet, call rc_sbus_init first\n");
		return -1;
	}
	// if global variable last_time ==0 then no packet must have arrived yet
	if(last_time==0) return -1;
	return rc_nanos_since_boot()-last_time;
}


uint32_t rc_sbus_lost_frames (void)
{
	return lost_frames;
}


uint32_t rc_sbus_total_errors (void)
{
	return total_errors;
}


int rc_sbus_signal_quality (void)
{
	return signal_quality;
}


int rc_sbus_calibrate_routine(void)
{
	int i,ret;
	FILE* fd;

	running = 1; // lets uarts 4 thread know it can run
	last_time = 0;
	active_flag = 0;
	new_data_callback=NULL;
	disconnect_callback=NULL;

	// make sure directory and calibration file exist and are writable first
	ret = mkdir(CALIBRATION_DIR, 0777);
	// error check, EEXIST is okay, we want directory to exist!
	if(ret==-1 && errno!=EEXIST){
		perror("ERROR in rc_sbus_calibration_routine making calibration file directory");
		return -1;
	}

	// 0.5s timeout disable canonical (0), 1 stop bit (1), disable parity (0)
	if(rc_uart_init(SBUS_UART_BUS, SBUS_BAUD_RATE, 0.5, 0, 1, 0)){
		fprintf(stderr,"ERROR in rc_sbus_calibrate_routine, failed to init uart bus\n");
		return -1;
	}

	pthread_create(&parse_thread, NULL, __parser_func, (void*) NULL);

	// wait for thread to start
	i=0;
	while(init_flag==0){
		rc_usleep(10000);
		if(i>10){
			fprintf(stderr, "ERROR in rc_sbus_calibrate_routine, timeout waiting for parser thread to start\n");
			return -1;
		}
		i++;
	}

	// display instructions
	printf("\nRaw sbus data should display below if the transmitter and\n");
	printf("receiver are paired and working. Move all channels through\n");
	printf("their range of motion and the minimum and maximum values will\n");
	printf("be recorded. When you are finished moving all channels,\n");
	printf("return 3-position switches and sticks to their natural\n");
	printf("zero-position which will be recorded.\n\n");
	printf("Two position switches can be left in either position, and sliding\n");
	printf("throttle sticks should be left at the bottom of their travel.\n");
	printf("If there is a RATE switch, make sure it's in the HIGH position.\n\n");
	printf("If there is a DISARM switch which fixes the throttle position, leave\n");
	printf("it in the ARMED state and DO NOT TOUCH IT during calibration\n");
	printf("Press ENTER to save data or any other key to abort.\n\n");

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
		fprintf(stderr,"aborting calibrate_sbus routine\n");
		return -1;
	}

	// if it looks like no new data came in exit
	if((mins[0]==0) || (mins[0]==maxes[0])){
		fprintf(stderr,"no new data recieved, exiting\n");
		return -1;
	}

	// open for writing
	fd = fopen(CALIBRATION_DIR SBUS_CALIBRATION_FILE, "w");
	if(fd == NULL){
		perror("ERROR in rc_sbus_calibration_routine opening calibration file for writing");
		return -1;
	}

	// if new data was captures for a channel, write data to cal file
	// otherwise fill in defaults for unused channels in case
	// a higher channel radio is used in the future with this cal file
	for(i=0;i<RC_MAX_SBUS_ANALOG_CHANNELS;i++){
		if((mins[i]==0) || (mins[i]==maxes[i])){
			fprintf(fd, "%d %d %d\n",DEFAULT_MIN, DEFAULT_MAX, DEFAULT_CENTER);
		}
		else{
			fprintf(fd, "%d %d %d\n", mins[i], maxes[i], centers[i]);
		}
	}
	fclose(fd);
	printf("New calibration file written\n");
	printf("use rc_test_sbus to confirm\n");
	return 0;
}

