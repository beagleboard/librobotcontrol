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


/*
Supporting library for Robotics Cape Features
Strawson Design - 2014
*/

#include "robotics_cape.h"

//// Button pins
// gpio # for gpio_a.b = (32*a)+b
#define PAUSE_BTN 69 	//gpio2.5 P8.9
#define MODE_BTN  68	//gpio2.4 P8.10


//// gpio output pins 
#define RED_LED P8_7	//gpio2.2
#define GRN_LED P8_8	//gpio2.3

#define MDIR1A    P9_12	//gpio1.28  P9.12
#define MDIR1B    P9_13	//gpio0.31	P9.13
#define MDIR2A    P9_15	//gpio1.16  P9.15
#define MDIR2B    P8_38	//gpio2.15  P8.38
#define MDIR4A    P8_45	//gpio2.6   P8.45
#define MDIR4B    P8_46	//gpio2.7   P8.46
#define MDIR3B    P8_43	//gpio2.8   P8.43
#define MDIR3A    P8_44	//gpio2.9   P8.44
#define MOT_STBY  P9_41	//gpio0.20  P9.41
#define PAIRING_PIN P9_11  //gpio0.30 P9.11
#define SPI1_SS1_GPIO_PIN P9_28  // P9.28 gpio3.17
#define SPI1_SS2_GPIO_PIN P9_23   // P9.23 gpio1.17

								 
//// eQep and pwmss registers, more in tipwmss.h
#define PWM0_BASE   0x48300000
#define PWM1_BASE   0x48302000
#define PWM2_BASE   0x48304000
#define EQEP_OFFSET  0x180

//// MPU-9150 defs
#define MPU_ADDR 0x68

#define POLL_TIMEOUT (3 * 1000) /* 3 seconds */
#define INTERRUPT_PIN 117  //gpio3.21 P9.25

#define UART4_PATH "/dev/ttyO4"


#define ARRAY_SIZE(array) sizeof(array)/sizeof(array[0]) 

// local function declarations
int initialize_button_handlers();
void* pause_pressed_handler(void* ptr);
void* pause_unpressed_handler(void* ptr);
void* mode_pressed_handler(void* ptr);
void* mode_unpressed_handler(void* ptr);

// state variable for loop and thread control
enum state_t state = UNINITIALIZED;

enum state_t get_state(){
	return state;
}

int set_state(enum state_t new_state){
	state = new_state;
	return 0;
}


								 
// buttons
int pause_btn_state, mode_btn_state;
int (*imu_interrupt_func)();
int (*pause_unpressed_func)();
int (*pause_pressed_func)();
int (*mode_unpressed_func)();
int (*mode_pressed_func)();

//function pointers for events initialized to null_func()
//instead of containing a null pointer
int null_func(){
	return 0;
}

int set_pause_pressed_func(int (*func)(void)){
	pause_pressed_func = func;
	return 0;
}
int set_pause_unpressed_func(int (*func)(void)){
	pause_unpressed_func = func;
	return 0;
}
int set_mode_pressed_func(int (*func)(void)){
	mode_pressed_func = func;
	return 0;
}
int set_mode_unpressed_func(int (*func)(void)){
	mode_unpressed_func = func;
	return 0;
}

int get_pause_button_state(){
	return pause_btn_state;
}

int get_mode_button_state(){
	return mode_btn_state;
}


// pwm stuff
// motors 1-4 driven by pwm 1A, 1B, 2A, 2B respectively
char pwm_files[][64] = {"/sys/devices/ocp.3/pwm_test_P9_14.12/",
							 "/sys/devices/ocp.3/pwm_test_P9_16.13/",
							 "/sys/devices/ocp.3/pwm_test_P8_19.14/",
							 "/sys/devices/ocp.3/pwm_test_P8_13.15/"
};
FILE *pwm_duty_pointers[4]; //store pointers to 4 pwm channels for frequent writes
int pwm_period_ns=0; //stores current pwm period in nanoseconds


// eQEP Encoder mmap arrays
volatile char *pwm_map_base[3];

// DSM2 Spektrum radio & UART4
int rc_channels[RC_CHANNELS];
int rc_maxes[RC_CHANNELS];
int rc_mins[RC_CHANNELS];
int tty4_fd;
int new_dsm2_flag;
int dsm2_frame_rate;
void* uart4_checker(void *ptr); //background thread

// PRU Servo Control shared memory pointer
#define PRU_NUM 	 1
#define PRU_BIN_LOCATION "/usr/bin/pru_servo.bin"
#define PRU_LOOP_INSTRUCTIONS	48		// instructions per PRU servo timer loop
static unsigned int *prusharedMem_32int_ptr;

int initialize_cape(){
	FILE *fd; 			// opened and closed for each file
	char path[128]; // buffer to store file path string
	int i = 0; 			// general use counter
	
	printf("\n");

	// check if another project was using resources
	// kill that process cleanly with sigint if so
	fd = fopen(LOCKFILE, "r");
	if (fd != NULL) {
		int old_pid;
		fscanf(fd,"%d", &old_pid);
		if(old_pid != 0){
			printf("warning, shutting down existing robotics project\n");
			kill((pid_t)old_pid, SIGINT);
			sleep(1);
		}
		// close and delete the old file
		fclose(fd);
		remove(LOCKFILE);
	}
	
	
	// create new lock file with process id
	fd = fopen(LOCKFILE, "ab+");
	if (fd < 0) {
		printf("\n error opening LOCKFILE for writing\n");
		return -1;
	}
	pid_t current_pid = getpid();
	printf("Current Process ID: %d\n", (int)current_pid);
	fprintf(fd,"%d",(int)current_pid);
	fflush(fd);
	fclose(fd);
	
	// map /dev/mem for adc, gpio, & eqep functions
	init_mmap();
	printf("/dev/mem mapped\n");
	
	// start adc
	//adc_init_mmap();
	
	
	// export a single pin from GPIO bank 3
	// just to initialize the final bank
	gpio_export(113);

	// start with slave pins high (deselected)
	deselect_spi1_slave(1);	
	deselect_spi1_slave(2);	
	

	//Set up PWM
	printf("Initializing PWM\n");
	i=0;
	for(i=0; i<4; i++){
		strcpy(path, pwm_files[i]);
		strcat(path, "polarity");
		fd = fopen(path, "a");
		if(fd<0){
			printf("PWM polarity not available in /sys/class/devices/ocp.3\n");
			return -1;
		}
		//set correct polarity such that 'duty' is time spent HIGH
		fprintf(fd,"%c",'0');
		fflush(fd);
		fclose(fd);
	}
	
	//leave duty cycle file open for future writes
	for(i=0; i<4; i++){
		strcpy(path, pwm_files[i]);
		strcat(path, "duty");
		pwm_duty_pointers[i] = fopen(path, "a");
	}
	
	//read in the pwm period defined in device tree overlay .dts
	strcpy(path, pwm_files[0]);
	strcat(path, "period");
	fd = fopen(path, "r");
	if(fd<0){
		printf("PWM period not available in /sys/class/devices/ocp.3\n");
		return -1;
	}
	fscanf(fd,"%i", &pwm_period_ns);
	fclose(fd);
	
	disable_motors();
	
	
	// mmap pwm modules to get fast access to eQep encoder position
	// see mmap_eqep example program for more mmap and encoder info
	printf("Initializing eQep Encoders\n");
	int dev_mem;
	if ((dev_mem = open("/dev/mem", O_RDWR | O_SYNC))==-1){
	  printf("Could not open /dev/mem \n");
	  return -1;
	}
	pwm_map_base[0] = mmap(0,getpagesize(),PROT_READ|PROT_WRITE,MAP_SHARED,dev_mem,PWM0_BASE);
	pwm_map_base[1] = mmap(0,getpagesize(),PROT_READ|PROT_WRITE,MAP_SHARED,dev_mem,PWM1_BASE);
	pwm_map_base[2] = mmap(0,getpagesize(),PROT_READ|PROT_WRITE,MAP_SHARED,dev_mem,PWM2_BASE);
	if(pwm_map_base[0] == (void *) -1) {
		printf("Unable to mmap pwm \n");
		return(-1);
	}
	close(dev_mem);
	
	// Test eqep and reset position
	for(i=1;i<3;i++){
		if(set_encoder_pos(i,0)){
			printf("failed to access eQep register\n");
			printf("eQep driver not loaded\n");
			return -1;
		}
	}
	
	//set up function pointers for button press events
	printf("starting button interrupts\n");
	set_pause_pressed_func(&null_func);
	set_pause_unpressed_func(&null_func);
	set_mode_pressed_func(&null_func);
	set_mode_unpressed_func(&null_func);
	initialize_button_handlers();
	
	// Load binary into PRU
	printf("Starting PRU servo controller\n");
	if(initialize_pru_servos()){
		printf("WARNING: PRU init FAILED");
	}
	
	// Print current battery voltage
	printf("Battery Voltage = %fV\n", getBattVoltage());
	
	// Start Signal Handler
	printf("Enabling exit signal handler\n");
	signal(SIGINT, ctrl_c);	
	
	// all done
	set_state(PAUSED);
	printf("\nRobotics Cape Initialized\n");

	return 0;
}

// set a motor direction and power
// motor is from 1 to 4
// duty is from -1 to +1
int set_motor(int motor, float duty){
	uint8_t a,b;
	
	if(state == UNINITIALIZED){
		initialize_cape();
	}

	//check that the duty cycle is within +-1
	if (duty>1){
		duty = 1;
	}
	else if(duty<-1){
		duty=-1;
	}
	//switch the direction pins to H-bridge
	if (duty>=0){
	 	a=HIGH;
		b=LOW;
	}
	else{
		a=LOW;
		b=HIGH;
		duty=-duty;
	}
	
	// set gpio direction outputs
	switch(motor){
		case 1:
			digitalWrite(MDIR1A, a);
			digitalWrite(MDIR1B, b);
			break;
		case 2:
			digitalWrite(MDIR2A, b);
			digitalWrite(MDIR2B, a);
			break;
		case 3:
			digitalWrite(MDIR3A, b);
			digitalWrite(MDIR3B, a);
			break;
		case 4:
			digitalWrite(MDIR4A, a);
			digitalWrite(MDIR4B, b);
			break;
		default:
			printf("enter a motor value between 1 and 4\n");
			return -1;
	}
	// send pwm duty
	fprintf(pwm_duty_pointers[motor-1], "%d", (int)(duty*pwm_period_ns));	
	fflush(pwm_duty_pointers[motor-1]);
	
	return 0;
}

int kill_pwm(){
	int ch;
	if(pwm_duty_pointers[0] == NULL){
		//printf("opening pwm duty files\n");
		char path[128];
		int i = 0;
		for(i=0; i<4; i++){
			strcpy(path, pwm_files[i]);
			strcat(path, "duty");
			pwm_duty_pointers[i] = fopen(path, "a");
		}
		//printf("opened pwm duty files\n");
	}
	for(ch=0;ch<4;ch++){
		fprintf(pwm_duty_pointers[ch], "%d", 0);	
		fflush(pwm_duty_pointers[ch]);
	}
	return 0;
}

int enable_motors(){
	kill_pwm();
	return digitalWrite(MOT_STBY, HIGH);
}

int disable_motors(){
	kill_pwm();
	return digitalWrite(MOT_STBY, LOW);
}

//// eQep Encoder read/write
long int get_encoder_pos(int ch){
	if(ch<1 || ch>3){
		printf("Encoder Channel must be in 1, 2, or 3\n");
		return -1;
	}
	return  *(unsigned long*)(pwm_map_base[ch-1] + EQEP_OFFSET +QPOSCNT);
}

int set_encoder_pos(int ch, long value){
	if(ch<1 || ch>3){
		printf("Encoder Channel must be 1, 2 or 3\n");
		return -1;
	}
	*(unsigned long*)(pwm_map_base[ch-1] + EQEP_OFFSET +QPOSCNT) = value;
	return 0;
}


//// LED functions
//  and be HIGH or LOW
int setGRN(uint8_t i){
	return digitalWrite(GRN_LED, i);
}
int setRED(uint8_t i){
	return digitalWrite(RED_LED, i);
}
int getGRN(){
	return digitalRead(GRN_LED);
}
int getRED(){
	return digitalRead(RED_LED);
}


//// ADC Functions
int get_adc_raw(int p){
	if(p<0 || p>6){
		printf("analog pin must be in 0-6\n");
		return -1;
	}
	return analogRead((uint8_t)p);
}

float get_adc_volt(int p){
	if(p<0 || p>6){
		printf("analog pin must be in 0-6\n");
		return -1;
	}
	int raw_adc = analogRead((uint8_t)p);
	return raw_adc * 1.8 / 4095.0;
}


float getBattVoltage(){
	// float v_adc = get_adc_volt(6);
	// return v_adc*11.0; 
	
	int raw_adc;
	FILE *AIN6_fd;
	AIN6_fd = fopen("/sys/devices/ocp.3/helper.16/AIN6", "r");
	if(AIN6_fd < 0){
		printf("error reading adc\n");
		return -1;
	}
	fscanf(AIN6_fd, "%i", &raw_adc);
	fclose(AIN6_fd);
	// times 11 for the voltage divider, divide by 1000 to go from mv to V
	return (float)raw_adc*11.0/1000.0; 
}

//// Read 6-16V DC input jack voltage
float getJackVoltage(){
	// float v_adc = get_adc_volt(5);
	// return v_adc*11.0; 
	
	int raw_adc;
	FILE *AIN5_fd;
	AIN5_fd = fopen("/sys/devices/ocp.3/helper.16/AIN5", "r");
	if(AIN5_fd < 0){
		printf("error reading adc\n");
		return -1;
	}
	fscanf(AIN5_fd, "%i", &raw_adc);
	fclose(AIN5_fd);
	// times 11 for the voltage divider, divide by 1000 to go from mv to V
	return (float)raw_adc*11.0/1000.0; 
}

/***********************************************************************
*	int initialize_button_interrups()
*	start 4 threads to handle 4 interrupt routines for pressing and
*	releasing the two buttons.
************************************************************************/
int initialize_button_handlers(){
	pthread_t pause_pressed_thread;
	pthread_t pause_unpressed_thread;
	pthread_t mode_pressed_thread;
	pthread_t mode_unpressed_thread;
	struct sched_param params;
	
	params.sched_priority = sched_get_priority_max(SCHED_FIFO)/2;
	
	pthread_create(&pause_pressed_thread, NULL,			 \
				pause_pressed_handler, (void*) NULL);
	pthread_create(&pause_unpressed_thread, NULL,			 \
				pause_unpressed_handler, (void*) NULL);
	pthread_create(&mode_pressed_thread, NULL,			 \
					mode_pressed_handler, (void*) NULL);
	pthread_create(&mode_unpressed_thread, NULL,			 \
					mode_unpressed_handler, (void*) NULL);
	
	// apply medium priority to all threads
	pthread_setschedparam(pause_pressed_thread, SCHED_FIFO, &params);
	pthread_setschedparam(pause_unpressed_thread, SCHED_FIFO, &params);
	pthread_setschedparam(mode_pressed_thread, SCHED_FIFO, &params);
	pthread_setschedparam(mode_unpressed_thread, SCHED_FIFO, &params);
	 
	return 0;
}

/***********************************************************************
*	void* pause_pressed_handler(void* ptr) 
*	wait on falling edge of pause button
************************************************************************/
void* pause_pressed_handler(void* ptr){
	int fd;
    fd = open("/dev/input/event1", O_RDONLY);
    struct input_event ev;
	while (get_state() != EXITING){
        read(fd, &ev, sizeof(struct input_event));
		// uncomment printf to see how event codes work
		// printf("type %i key %i state %i\n", ev.type, ev.code, ev.value); 
        if(ev.type==1 && ev.code==1){ //only new data
			if(ev.value == 1){
				pause_btn_state = PRESSED;
				(*pause_pressed_func)();
			}
		}
		usleep(10000); // wait
    }
	return NULL;
}

/***********************************************************************
*	void* pause_unpressed_handler(void* ptr) 
*	wait on rising edge of pause button
************************************************************************/
void* pause_unpressed_handler(void* ptr){
	int fd;
    fd = open("/dev/input/event1", O_RDONLY);
    struct input_event ev;
	while (get_state() != EXITING){
        read(fd, &ev, sizeof(struct input_event));
		// uncomment printf to see how event codes work
		// printf("type %i key %i state %i\n", ev.type, ev.code, ev.value); 
        if(ev.type==1 && ev.code==1){ //only new data
			if(ev.value == 0){
			
				pause_btn_state = UNPRESSED;
				(*pause_unpressed_func)();
			}
		}
		usleep(10000); // wait
    }
	return NULL;
}

/***********************************************************************
*	void* mode_pressed_handler(void* ptr) 
*	wait on falling edge of mode button
************************************************************************/
void* mode_pressed_handler(void* ptr){
	int fd;
    fd = open("/dev/input/event1", O_RDONLY);
    struct input_event ev;
	while (get_state() != EXITING){
        read(fd, &ev, sizeof(struct input_event));
		// uncomment printf to see how event codes work
		// printf("type %i key %i state %i\n", ev.type, ev.code, ev.value); 
        if(ev.type==1 && ev.code==2){ //only new data
			if(ev.value == 1){
				mode_btn_state = PRESSED;
				(*mode_pressed_func)();
			}
		}
		usleep(10000); // wait
    }
	return NULL;
}

/***********************************************************************
*	void* mode_unpressed_handler(void* ptr) 
*	wait on rising edge of mode button
************************************************************************/
void* mode_unpressed_handler(void* ptr){
	int fd;
    fd = open("/dev/input/event1", O_RDONLY);
    struct input_event ev;
	while (get_state() != EXITING){
        read(fd, &ev, sizeof(struct input_event));
		// uncomment printf to see how event codes work
		// printf("type %i key %i state %i\n", ev.type, ev.code, ev.value); 
        if(ev.type==1 && ev.code==2){ //only new data
			if(ev.value == 0){
				mode_btn_state = UNPRESSED; //pressed
				(*mode_unpressed_func)();
			}
		}
		usleep(10000); // wait
    }
	return NULL;
}


////  DSM2  Spektrum  RC Stuff  
int initialize_dsm2(){
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
		printf("Use calibrate_dsm2 example to create one\n");
		return -1;
	}
	else{
		int i;
		for(i=0;i<RC_CHANNELS;i++){
			fscanf(cal,"%d %d", &rc_mins[i],&rc_maxes[i]);
			//printf("%d %d\n", rc_mins[i],rc_maxes[i]);
		}
		printf("DSM2 Calibration Loaded\n");
	}
	fclose(cal);
	
	dsm2_frame_rate = 0; // zero until mode is detected on first packet
	pthread_t uart4_thread;
	pthread_create(&uart4_thread, NULL, uart4_checker, (void*) NULL);
	printf("DSM2 Thread Started\n");
	return 0;
}

float get_dsm2_ch_normalized(int ch){
	if(ch<1 || ch > RC_CHANNELS){
		printf("please enter a channel between 1 & %d",RC_CHANNELS);
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

int get_dsm2_ch_raw(int ch){
	if(ch<1 || ch > RC_CHANNELS){
		printf("please enter a channel between 1 & %d",RC_CHANNELS);
		return -1;
	}
	else{
		new_dsm2_flag = 0;
		return rc_channels[ch-1];
	}
}

int is_new_dsm2_data(){
	return new_dsm2_flag;
}

int get_dsm2_frame_rate(){
	return dsm2_frame_rate;
}

// uncomment defines to print raw data for debugging
//#define DEBUG_DSM2
//#define DEBUG_DSM2_RAW
void* uart4_checker(void *ptr){
	
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
	if ((tty4_fd = open (UART4_PATH, O_RDWR | O_NOCTTY)) < 0) {
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

	while(1){
		char buf[64]; // large serial buffer to catch doubled up packets
		int i,j;
		
		memset(&buf, 0, sizeof(buf)); // clear buffer
		
		i = read(tty4_fd,&buf,sizeof(buf)); // blocking read
		
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
				if((ch_id+1)>9){
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
					
					// extra bit of precision means scale is off by factor of two
					// also add 989 to center channels around 1500
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
				rc_channels[ch_id] = value;
			}
		}

		// indicate new a new packet has been processed
		new_dsm2_flag=1;
		
		#ifdef DEBUG_DSM2
		printf("\n");
		#endif
		
end:
		// wait a bit, then check the buffer again
		usleep(2000);
	}
	return NULL;
}



//// MPU9150 IMU ////
int initialize_imu(int sample_rate, signed char orientation[9]){
	printf("Initializing IMU\n");
	//set up gpio interrupt pin connected to imu
	if(gpio_export(INTERRUPT_PIN)){
		printf("can't export gpio %d \n", INTERRUPT_PIN);
		return (-1);
	}
	gpio_set_dir(INTERRUPT_PIN, INPUT_PIN);
	gpio_set_edge(INTERRUPT_PIN, "falling");  // Can be rising, falling or both
		
	linux_set_i2c_bus(1);

	
	if (mpu_init(NULL)) {
		printf("\nmpu_init() failed\n");
		return -1;
	}
 
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
	mpu_set_sample_rate(sample_rate);
	
	// compass run at 100hz max. 
	if(sample_rate > 100){
		// best to sample at a fraction of the gyro/accel
		mpu_set_compass_sample_rate(sample_rate/2);
	}
	else{
		mpu_set_compass_sample_rate(sample_rate);
	}
	mpu_set_lpf(188); //as little filtering as possible
	dmp_load_motion_driver_firmware(sample_rate);

	dmp_set_orientation(inv_orientation_matrix_to_scalar(orientation));
  	dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL 
						| DMP_FEATURE_SEND_CAL_GYRO);

	dmp_set_fifo_rate(sample_rate);
	
	if (mpu_set_dmp_state(1)) {
		printf("\nmpu_set_dmp_state(1) failed\n");
		return -1;
	}
	if(loadGyroCalibration()){
		printf("\nGyro Calibration File Doesn't Exist Yet\n");
		printf("Use calibrate_gyro example to create one\n");
		printf("Using 0 offset for now\n");
	};
	
	// set the imu_interrupt_thread to highest priority since this is used
	// for real-time control with time-sensitive IMU data
	set_imu_interrupt_func(&null_func);
	pthread_t imu_interrupt_thread;
	struct sched_param params;
	pthread_create(&imu_interrupt_thread, NULL, imu_interrupt_handler, (void*) NULL);
	params.sched_priority = sched_get_priority_max(SCHED_FIFO);
	pthread_setschedparam(imu_interrupt_thread, SCHED_FIFO, &params);
	
	return 0;
}

int setXGyroOffset(int16_t offset) {
	uint16_t new = offset;
	const unsigned char msb = (unsigned char)((new&0xff00)>>8);
	const unsigned char lsb = (new&0x00ff);//get LSB
	//printf("writing: 0x%x 0x%x 0x%x \n", new, msb, lsb);
	linux_i2c_write(MPU_ADDR, MPU6050_RA_XG_OFFS_USRH, 1, &msb);
	return linux_i2c_write(MPU_ADDR, MPU6050_RA_XG_OFFS_USRL, 1, &lsb);
}

int setYGyroOffset(int16_t offset) {
	uint16_t new = offset;
	const unsigned char msb = (unsigned char)((new&0xff00)>>8);
	const unsigned char lsb = (new&0x00ff);//get LSB
	//printf("writing: 0x%x 0x%x 0x%x \n", new, msb, lsb);
	linux_i2c_write(MPU_ADDR, MPU6050_RA_YG_OFFS_USRH, 1, &msb);
	return linux_i2c_write(MPU_ADDR, MPU6050_RA_YG_OFFS_USRL, 1, &lsb);
}

int setZGyroOffset(int16_t offset) {
	uint16_t new = offset;
	const unsigned char msb = (unsigned char)((new&0xff00)>>8);
	const unsigned char lsb = (new&0x00ff);//get LSB
	//printf("writing: 0x%x 0x%x 0x%x \n", new, msb, lsb);
	linux_i2c_write(MPU_ADDR, MPU6050_RA_ZG_OFFS_USRH, 1, &msb);
	return linux_i2c_write(MPU_ADDR, MPU6050_RA_ZG_OFFS_USRL, 1, &lsb);
}

int loadGyroCalibration(){
	FILE *cal;
	char file_path[100];

	// construct a new file path string
	strcpy (file_path, CONFIG_DIRECTORY);
	strcat (file_path, GYRO_CAL_FILE);
	
	// open for reading
	cal = fopen(file_path, "r");
	if (cal == 0) {
		// calibration file doesn't exist yet
		return -1;
	}
	else{
		int xoffset, yoffset, zoffset;
		fscanf(cal,"%d\n%d\n%d\n", &xoffset, &yoffset, &zoffset);
		if(setXGyroOffset((int16_t)xoffset)){
			printf("problem setting gyro offset\n");
			return -1;
		}
		if(setYGyroOffset((int16_t)yoffset)){
			printf("problem setting gyro offset\n");
			return -1;
		}
		if(setZGyroOffset((int16_t)zoffset)){
			printf("problem setting gyro offset\n");
			return -1;
		}
	}
	fclose(cal);
	return 0;
}

// IMU interrupt stuff
// see test_imu and calibrate_gyro for examples
int set_imu_interrupt_func(int (*func)(void)){
	imu_interrupt_func = func;
	return 0;
}

void* imu_interrupt_handler(void* ptr){
	struct pollfd fdset[1];
	char buf[MAX_BUF];
	int imu_gpio_fd = gpio_fd_open(INTERRUPT_PIN);
	fdset[0].fd = imu_gpio_fd;
	fdset[0].events = POLLPRI; // high-priority interrupt
	// keep running until the program closes
	while(get_state() != EXITING) {
		// system hangs here until IMU FIFO interrupt
		poll(fdset, 1, POLL_TIMEOUT);        
		if (fdset[0].revents & POLLPRI) {
			lseek(fdset[0].fd, 0, SEEK_SET);  
			read(fdset[0].fd, buf, MAX_BUF);
			// user selectable with set_inu_interrupt_func() defined above
			imu_interrupt_func(); 
		}
	}
	gpio_fd_close(imu_gpio_fd);
	return 0;
}

//// PRU Servo Control
int initialize_pru_servos(){
	// start pru
    prussdrv_init();

    // Open PRU Interrupt
    if (prussdrv_open(PRU_EVTOUT_0)){
        printf("prussdrv_open open failed\n");
        return -1;
    }

    // Get the interrupt initialized
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
    prussdrv_pruintc_init(&pruss_intc_initdata);

	// get pointer to PRU shared memory
	void* sharedMem = NULL;
    prussdrv_map_prumem(PRUSS0_SHARED_DATARAM, &sharedMem);
    prusharedMem_32int_ptr = (unsigned int*) sharedMem;
	memset(prusharedMem_32int_ptr, 0, SERVO_CHANNELS*4);
	
	// launch servo binary
	prussdrv_exec_program(PRU_NUM, PRU_BIN_LOCATION);

    return(0);
}

int send_servo_pulse_us(int ch, float us){
	// Sanity Checks
	if(ch<1 || ch>SERVO_CHANNELS){
		printf("ERROR: Servo Channel must be between 1 & %d \n", SERVO_CHANNELS);
		return -1;
	}
	if(prusharedMem_32int_ptr == NULL){
		printf("ERROR: PRU servo Controller not initialized\n");
		return -1;
	}

	// PRU runs at 200Mhz. find #loops needed
	unsigned int num_loops = ((us*200)/PRU_LOOP_INSTRUCTIONS); 
	
	// write to PRU shared memory
	prusharedMem_32int_ptr[ch-1] = num_loops;
	return 0;
}

int send_servo_pulse_normalized(int ch, float input){
	if(ch<1 || ch>SERVO_CHANNELS){
		printf("ERROR: Servo Channel must be between 1 & %d \n", SERVO_CHANNELS);
		return -1;
	}
	if(input<0.0 || input>1.0){
		printf("ERROR: normalized input must be between 0 & 1\n");
		return -1;
	}
	float micros = SERVO_MIN_US + (input*(SERVO_MAX_US-SERVO_MIN_US));
	return send_servo_pulse_us(ch, micros);
}


//// helpful functions
char *byte_to_binary(unsigned char x){
    static char b[9];
    b[0] = '\0';
    unsigned char z;
    for (z = 128; z > 0; z >>= 1){
        if(x&z)
			strcat(b, "1");
		else
			strcat(b, "0");
    }
    return b;
}

// find the difference between two timespec structs
// used with sleep_ns() function and accurately timed loops
timespec diff(timespec start, timespec end){
	timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000L+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

uint64_t microsSinceEpoch(){
	struct timeval tv;
	uint64_t micros = 0;
	gettimeofday(&tv, NULL);  
	micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	return micros;
}


// Mavlink easy setup for UDP
// This function mostly taken from Bryan Godbolt's mavlink_udp example
struct sockaddr_in initialize_mavlink_udp(char gc_ip_addr[],  int *udp_sock){
	struct sockaddr_in gcAddr;
	char target_ip[100];
	struct sockaddr_in locAddr;
	int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	
	strcpy(target_ip, gc_ip_addr);
	memset(&locAddr, 0, sizeof(locAddr));
	locAddr.sin_family = AF_INET;
	locAddr.sin_addr.s_addr = INADDR_ANY;
	locAddr.sin_port = htons(14551); //uav listening port for mavlink
	/* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */ 
	if (-1 == bind(sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr))){
		perror("error bind failed");
		close(sock);
		exit(EXIT_FAILURE);
    }

	memset(&gcAddr, 0, sizeof(&gcAddr));
	gcAddr.sin_family = AF_INET;
	gcAddr.sin_addr.s_addr = inet_addr(target_ip);
	gcAddr.sin_port = htons(14550);
	*udp_sock = sock;
	printf("Initialized Mavlink with Ground Control address ");
	printf(target_ip);
	printf("\n");
	return gcAddr;
}


//// SPI1   see test_adns9800 example
int initialize_spi1(){ // returns a file descriptor to spi device
	int spi1_fd;
	spi1_fd = open("/dev/spidev2.0", O_RDWR); // actually spi1
    if (spi1_fd<=0) {  
        printf("/dev/spidev2.0 not found\n"); 
        return -1; 
    } 
	digitalWrite(SPI1_SS1_GPIO_PIN, HIGH);
	digitalWrite(SPI1_SS2_GPIO_PIN, HIGH);
	
	return spi1_fd;
}

int select_spi1_slave(int slave){
	switch(slave){
		case 1:
			digitalWrite(SPI1_SS2_GPIO_PIN, HIGH);
			return digitalWrite(SPI1_SS1_GPIO_PIN, LOW);
		case 2:
			digitalWrite(SPI1_SS1_GPIO_PIN, HIGH);
			return digitalWrite(SPI1_SS2_GPIO_PIN, LOW);
		default:
			printf("SPI slave number must be 1 or 2\n");
			return -1;
	}
}	
int deselect_spi1_slave(int slave){
	switch(slave){
		case 1:
			return digitalWrite(SPI1_SS1_GPIO_PIN, HIGH);
		case 2:
			return digitalWrite(SPI1_SS2_GPIO_PIN, HIGH);
		default:
			printf("SPI slave number must be 1 or 2\n");
			return -1;
	}
}	

// catch Ctrl-C signal and change system state
// all threads should watch for get_state()==EXITING and shut down cleanly
void ctrl_c(int signo){
	if (signo == SIGINT){
		set_state(EXITING);
		printf("\nreceived SIGINT Ctrl-C\n");
 	}
}

// cleanup_cape() should be at the end of every main() function
int cleanup_cape(){
	set_state(EXITING);
	usleep(500000); // let final threads clean up
	FILE* fd;
	// clean up the lockfile if it still exists
	fd = fopen(LOCKFILE, "r");
	if (fd != NULL) {
		// close and delete the old file
		fclose(fd);
		remove(LOCKFILE);
	}
	 
	setGRN(0);
	setRED(0);	
	disable_motors();
	deselect_spi1_slave(1);	
	deselect_spi1_slave(2);	
	prussdrv_pru_disable(PRU_NUM);
    prussdrv_exit();
	printf("\nExiting Cleanly\n");
	return 0;
}