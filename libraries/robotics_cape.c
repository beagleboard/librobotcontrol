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

//#define DEBUG

#define _GNU_SOURCE 	// to enable macros in pthread
#include <pthread.h>    // multi-threading
#include <errno.h>		// pthread error codes

#include <robotics_cape.h>
#include <robotics_cape_revC_defs.h>

/**************************************
* Local Global Variables
***************************************/
int rc_channels[RC_CHANNELS];
int rc_maxes[RC_CHANNELS];
int rc_mins[RC_CHANNELS];
int tty4_fd;
int new_dsm2_flag;
int dsm2_frame_rate;
enum state_t state = UNINITIALIZED;
int pause_btn_state, mode_btn_state;
static unsigned int *prusharedMem_32int_ptr;

/**************************************
* local function declarations
***************************************/
int is_cape_loaded();
int initialize_button_handlers();
int (*imu_interrupt_func)();
int (*pause_unpressed_func)();
int (*pause_pressed_func)();
int (*mode_unpressed_func)();
int (*mode_pressed_func)();

/**************************************
* local thread function declarations
***************************************/
void* pause_pressed_handler(void* ptr);
void* pause_unpressed_handler(void* ptr);
void* mode_pressed_handler(void* ptr);
void* mode_unpressed_handler(void* ptr);
void* imu_interrupt_handler(void* ptr);
void* uart4_checker(void *ptr); //background thread

/**************************************
* local thread structs
***************************************/
pthread_t pause_pressed_thread;
pthread_t pause_unpressed_thread;
pthread_t mode_pressed_thread;
pthread_t mode_unpressed_thread;
pthread_t imu_interrupt_thread;
pthread_t uart4_thread;


/*****************************************************************
* int initialize_cape()
* sets up necessary hardware and software
* should be the first thing your program calls
*****************************************************************/
int initialize_cape(){
	FILE *fd; 		
	printf("\n");

	// check if another project was using resources
	// kill that process cleanly with sigint if so
	#ifdef DEBUG
		printf("checking for existing PID_FILE\n");
	#endif
	fd = fopen(PID_FILE, "r");
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
		remove(PID_FILE);
	}
	
	// create new pid file with process id
	#ifdef DEBUG
		printf("opening PID_FILE\n");
	#endif
	fd = fopen(PID_FILE, "ab+");
	if (fd < 0) {
		printf("\n error opening PID_FILE for writing\n");
		return -1;
	}
	pid_t current_pid = getpid();
	printf("Current Process ID: %d\n", (int)current_pid);
	fprintf(fd,"%d",(int)current_pid);
	fflush(fd);
	fclose(fd);
	
	// check the device tree overlay is actually loaded
	if (is_cape_loaded() != 1){
		printf("ERROR: Device tree overlay not loaded by cape manager\n");
		return -1;
	}
	
	// initialize mmap io libs
	printf("initializing GPIO\n");
	if(initialize_gpio()){
		printf("mmap_gpio_adc.c failed to initialize gpio\n");
		return -1;
	}
	printf("Initializing ADC\n");
	if(initialize_adc()){
		printf("mmap_gpio_adc.c failed to initialize adc\n");
		return -1;
	}
	printf("Initializing eQEP\n");
	if(init_eqep(0)){
		printf("mmap_pwmss.c failed to initialize eQEP\n");
		return -1;
	}
	if(init_eqep(1)){
		printf("mmap_pwmss.c failed to initialize eQEP\n");
		return -1;
	}
	if(init_eqep(2)){
		printf("mmap_pwmss.c failed to initialize eQEP\n");
		return -1;
	}
	printf("Initializing PWM\n");
	if(init_pwm(1)){
		printf("mmap_pwmss.c failed to initialize PWMSS 0\n");
		return -1;
	}
	if(init_pwm(2)){
		printf("mmap_pwmss.c failed to initialize PWMSS 1\n");
		return -1;
	}
	
	// start some gpio pins at defaults
	deselect_spi1_slave(1);	
	deselect_spi1_slave(2);
	disable_motors();
	
	//set up function pointers for button press events
	printf("starting button interrupts\n");
	initialize_button_handlers();
	
	// Load binary into PRU
	printf("Starting PRU servo controller\n");
	if(initialize_pru_servos()){
		printf("ERROR: PRU init FAILED\n");
		return -1;
	}
	
	// Start Signal Handler
	printf("Enabling exit signal handler\n");
	signal(SIGINT, ctrl_c);	
	
	// Print current battery voltage
	printf("Battery Voltage: %2.2fV\n", getBattVoltage());
	
	// all done
	set_state(PAUSED);
	printf("\nRobotics Cape Initialized\n");

	return 0;
}

/*********************************************************************************
*	int cleanup_cape()
*	shuts down library and hardware functions cleanly
*	you should call this before your main() function returns
**********************************************************************************/
int cleanup_cape(){
	set_state(EXITING);
	
	// allow up to 3 seconds for thread cleanup
	struct timespec thread_timeout;
	clock_gettime(CLOCK_REALTIME, &thread_timeout);
	thread_timeout.tv_sec += 3;
	int thread_err = 0;
	thread_err = pthread_timedjoin_np(pause_pressed_thread, NULL, &thread_timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: pause_pressed_thread exit timeout\n");
	}
	thread_err = 0;
	thread_err = pthread_timedjoin_np(pause_unpressed_thread, NULL, &thread_timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: pause_unpressed_thread exit timeout\n");
	}
	thread_err = 0;
	thread_err = pthread_timedjoin_np(mode_pressed_thread, NULL, &thread_timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: mode_pressed_thread exit timeout\n");
	}
	thread_err = 0;
	thread_err = pthread_timedjoin_np(mode_unpressed_thread, NULL, &thread_timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: mode_unpressed_thread exit timeout\n");
	}
	thread_err = 0;
	thread_err = pthread_timedjoin_np(imu_interrupt_thread, NULL, &thread_timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: imu_interrupt_thread exit timeout\n");
	}
	
	#ifdef DEBUG
	printf("turning off GPIOs & PWM\n");
	#endif
	setGRN(0);
	setRED(0);	
	disable_motors();
	deselect_spi1_slave(1);	
	deselect_spi1_slave(2);	
	
	#ifdef DEBUG
	printf("turning off PRU\n");
	#endif
	prussdrv_pru_disable(PRU_SERVO_NUM);
    prussdrv_exit();
	
	#ifdef DEBUG
	printf("deleting PID file\n");
	#endif
	FILE* fd;
	// clean up the pid_file if it still exists
	fd = fopen(PID_FILE, "r");
	if (fd != NULL) {
		// close and delete the old file
		fclose(fd);
		remove(PID_FILE);
	}
	
	printf("\nExiting Cleanly\n");
	return 0;
}


/*****************************************************************
* enum state_t get_state()
* returns the high-level robot state variable
* use this for managing how your threads start and stop
*****************************************************************/
enum state_t get_state(){
	return state;
}


/*****************************************************************
* int set_state(enum state_t new_state)
* sets the high-level robot state variable
* use this for managing how your threads start and stop
*****************************************************************/
int set_state(enum state_t new_state){
	state = new_state;
	return 0;
}

/*****************************************************************
* int null_func()
* function pointers for events initialized to null_func()
* instead of containing a null pointer
*****************************************************************/
int null_func(){
	return 0;
}


/*****************************************************************
* int set_motor(int motor, float duty)
* 
* set a motor direction and power
* motor is from 1 to 4, duty is from -1.0 to +1.0
*****************************************************************/
int set_motor(int motor, float duty){
	uint8_t a,b;
	
	if(state == UNINITIALIZED){
		initialize_cape();
	}

	//check that the duty cycle is within +-1
	if (duty>1.0){
		duty = 1.0;
	}
	else if(duty<-1.0){
		duty=-1.0;
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
	
	// set gpio direction outputs & duty
	switch(motor){
		case 1:
			digitalWrite(MDIR1A, a);
			digitalWrite(MDIR1B, b);
			set_pwm_duty(1, 'A', duty);
			break;
		case 2:
			digitalWrite(MDIR2A, b);
			digitalWrite(MDIR2B, a);
			set_pwm_duty(1, 'B', duty);
			break;
		case 3:
			digitalWrite(MDIR3A, b);
			digitalWrite(MDIR3B, a);
			set_pwm_duty(2, 'A', duty);
			break;
		case 4:
			digitalWrite(MDIR4A, a);
			digitalWrite(MDIR4B, b);
			set_pwm_duty(2, 'B', duty);
			break;
		default:
			printf("enter a motor value between 1 and 4\n");
			return -1;
	}
	return 0;
}

/*****************************************************************
* int set_motor_all(float duty)
* 
* applies the same duty cycle argument to all 4 motors
*****************************************************************/
int set_motor_all(float duty){
	int i;
	for(i=1;i<=MOTOR_CHANNELS; i++){
		set_motor(i, duty);
	}
	return 0;
}

/*****************************************************************
* int kill_pwm()
* 
* disable pwm output for all motors
* used in shutdown sequence
*****************************************************************/
int kill_pwm(){
	set_pwm_duty(1, 'A', 0.0);
	set_pwm_duty(1, 'B', 0.0);
	set_pwm_duty(2, 'A', 0.0);
	set_pwm_duty(2, 'B', 0.0);
	return 0;
}

/*****************************************************************
* enable_motors()
* 
* turns on the standby pin to enable the h-bridge ICs
* returns 0 on success
*****************************************************************/
int enable_motors(){
	kill_pwm();
	return digitalWrite(MOT_STBY, HIGH);
}

/*****************************************************************
* int disable_motors()
* 
* turns off the standby pin to disable the h-bridge ICs
* and disables PWM output signals, returns 0 on success
*****************************************************************/
int disable_motors(){
	kill_pwm();
	return digitalWrite(MOT_STBY, LOW);
}


/*****************************************************************
* int get_encoder_pos(int ch)
* 
* returns the encoder counter position
*****************************************************************/
int get_encoder_pos(int ch){
	if(ch>3 || ch<1){
		printf("encoder channel must be 1,2, or 3\n");
		return -1;
	}
	return read_eqep(ch-1);
}

/*****************************************************************
* int set_encoder_pos(int ch, int val)
* 
* sets the encoder counter position
*****************************************************************/
int set_encoder_pos(int ch, int val){
	if(ch>3 || ch<1){
		printf("encoder channel must be 1,2, or 3\n");
		return -1;
	}
	return write_eqep(ch, val);
}


/*****************************************************************
* int setGRN(uint8_t i)
* 
* turn on or off the green LED on robotics cape
* takes in a value either HIGH or LOW
*****************************************************************/
int setGRN(uint8_t i){
	return digitalWrite(GRN_LED, i);
}

/*****************************************************************
* int setRED(uint8_t i)
* 
* turn on or off the red LED on robotics cape
* takes in a value either HIGH or LOW
*****************************************************************/
int setRED(uint8_t i){
	return digitalWrite(RED_LED, i);
}

/*****************************************************************
* int getGRN(uint8_t i)
* 
* returns the status of the green LED on robotics cape
* takes in a value either HIGH or LOW
*****************************************************************/
int getGRN(){
	return digitalRead(GRN_LED);
}

/*****************************************************************
* int getRED(uint8_t i)
* 
* returns the status of the red LED on robotics cape
* takes in a value either HIGH or LOW
*****************************************************************/
int getRED(){
	return digitalRead(RED_LED);
}


//// ADC Functions
/*****************************************************************
* int get_adc_raw(int p)
*
* returns the raw adc reading
*****************************************************************/
int get_adc_raw(int p){
	if(p<0 || p>6){
		printf("analog pin must be in 0-6\n");
		return -1;
	}
	return analogRead((uint8_t)p);
}

/*****************************************************************
* float get_adc_volt(int p)
* 
* returns an actual voltage for an adc channel
*****************************************************************/
float get_adc_volt(int p){
	if(p<0 || p>6){
		printf("analog pin must be in 0-6\n");
		return -1;
	}
	int raw_adc = analogRead((uint8_t)p);
	return raw_adc * 1.8 / 4095.0;
}

/*****************************************************************
* float getBattVoltage()
* 
* returns the LiPo battery voltage on the robotics cape
* this accounts for the voltage divider ont he cape
*****************************************************************/
float getBattVoltage(){
	float v_adc = get_adc_volt(6);
	return v_adc*11.0; 
}

/*****************************************************************
* float getJackVoltage()
* 
* returns the DC power jack voltage on the robotics cape
* this accounts for the voltage divider ont he cape
*****************************************************************/
float getJackVoltage(){
	float v_adc = get_adc_volt(5);
	return v_adc*11.0; 
}

/*********************************************************************************
*	int initialize_button_interrups()
*
*	start 4 threads to handle 4 interrupt routines for pressing and
*	releasing the two buttons.
**********************************************************************************/
int initialize_button_handlers(){
	
	#ifdef DEBUG
	printf("setting up mode & pause gpio pins\n");
	#endif
	//set up mode pin
	if(gpio_export(MODE_BTN)){
		printf("can't export gpio %d \n", MODE_BTN);
		return (-1);
	}
	gpio_set_dir(MODE_BTN, INPUT_PIN);
	gpio_set_edge(MODE_BTN, "both");  // Can be rising, falling or both
	
	//set up pause pin
	if(gpio_export(PAUSE_BTN)){
		printf("can't export gpio %d \n", PAUSE_BTN);
		return (-1);
	}
	gpio_set_dir(PAUSE_BTN, INPUT_PIN);
	gpio_set_edge(PAUSE_BTN, "both");  // Can be rising, falling or both
	
	#ifdef DEBUG
	printf("starting button handling threads\n");
	#endif
	struct sched_param params;
	pthread_attr_t attr;
	params.sched_priority = sched_get_priority_max(SCHED_FIFO)/2;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
   
	set_pause_pressed_func(&null_func);
	set_pause_unpressed_func(&null_func);
	set_mode_pressed_func(&null_func);
	set_mode_unpressed_func(&null_func);
	
	
	pthread_create(&pause_pressed_thread, &attr,			 \
				pause_pressed_handler, (void*) NULL);
	pthread_create(&pause_unpressed_thread, &attr,			 \
				pause_unpressed_handler, (void*) NULL);
	pthread_create(&mode_pressed_thread, &attr,			 \
					mode_pressed_handler, (void*) NULL);
	pthread_create(&mode_unpressed_thread, &attr,			 \
					mode_unpressed_handler, (void*) NULL);
	
	// apply medium priority to all threads
	pthread_setschedparam(pause_pressed_thread, SCHED_FIFO, &params);
	pthread_setschedparam(pause_unpressed_thread, SCHED_FIFO, &params);
	pthread_setschedparam(mode_pressed_thread, SCHED_FIFO, &params);
	pthread_setschedparam(mode_unpressed_thread, SCHED_FIFO, &params);
	 
	return 0;
}

/*********************************************************************************
*	void* pause_pressed_handler(void* ptr)
* 
*	wait on falling edge of pause button
**********************************************************************************/
void* pause_pressed_handler(void* ptr){
	struct pollfd fdset[1];
	char buf[MAX_BUF];
	int gpio_fd = gpio_fd_open(PAUSE_BTN);
	fdset[0].fd = gpio_fd;
	fdset[0].events = POLLPRI; // high-priority interrupt
	// keep running until the program closes
	while(get_state() != EXITING) {
		// system hangs here until FIFO interrupt
		poll(fdset, 1, POLL_TIMEOUT);        
		if (fdset[0].revents & POLLPRI) {
			lseek(fdset[0].fd, 0, SEEK_SET);  
			read(fdset[0].fd, buf, MAX_BUF);
			if(get_pause_button_state()==PRESSED){
				pause_pressed_func(); 
			}
		}
	}
	gpio_fd_close(gpio_fd);
	return 0;
}

/*********************************************************************************
*	void* pause_unpressed_handler(void* ptr) 
*
*	wait on rising edge of pause button
**********************************************************************************/
void* pause_unpressed_handler(void* ptr){
	struct pollfd fdset[1];
	char buf[MAX_BUF];
	int gpio_fd = gpio_fd_open(PAUSE_BTN);
	fdset[0].fd = gpio_fd;
	fdset[0].events = POLLPRI; // high-priority interrupt
	// keep running until the program closes
	while(get_state() != EXITING) {
		// system hangs here until FIFO interrupt
		poll(fdset, 1, POLL_TIMEOUT);        
		if (fdset[0].revents & POLLPRI) {
			lseek(fdset[0].fd, 0, SEEK_SET);  
			read(fdset[0].fd, buf, MAX_BUF);
			if(get_pause_button_state()==UNPRESSED){
				pause_unpressed_func(); 
			}
		}
	}
	gpio_fd_close(gpio_fd);
	return 0;
}

/*********************************************************************************
*	void* mode_pressed_handler(void* ptr) 
*	wait on falling edge of mode button
**********************************************************************************/
void* mode_pressed_handler(void* ptr){
	struct pollfd fdset[1];
	char buf[MAX_BUF];
	int gpio_fd = gpio_fd_open(MODE_BTN);
	fdset[0].fd = gpio_fd;
	fdset[0].events = POLLPRI; // high-priority interrupt
	// keep running until the program closes
	while(get_state() != EXITING) {
		// system hangs here until FIFO interrupt
		poll(fdset, 1, POLL_TIMEOUT);        
		if (fdset[0].revents & POLLPRI) {
			lseek(fdset[0].fd, 0, SEEK_SET);  
			read(fdset[0].fd, buf, MAX_BUF);
			if(get_mode_button_state()==PRESSED){
				mode_pressed_func(); 
			}
		}
	}
	gpio_fd_close(gpio_fd);
	return 0;
}

/*********************************************************************************
*	void* mode_unpressed_handler(void* ptr) 
*	wait on rising edge of mode button
**********************************************************************************/
void* mode_unpressed_handler(void* ptr){
	struct pollfd fdset[1];
	char buf[MAX_BUF];
	int gpio_fd = gpio_fd_open(MODE_BTN);
	fdset[0].fd = gpio_fd;
	fdset[0].events = POLLPRI; // high-priority interrupt
	// keep running until the program closes
	while(get_state() != EXITING) {
		// system hangs here until FIFO interrupt
		poll(fdset, 1, POLL_TIMEOUT);        
		if (fdset[0].revents & POLLPRI) {
			lseek(fdset[0].fd, 0, SEEK_SET);  
			read(fdset[0].fd, buf, MAX_BUF);
			if(get_mode_button_state()==UNPRESSED){
				mode_unpressed_func(); 
			}
		}
	}
	gpio_fd_close(gpio_fd);
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
	if(digitalRead(PAUSE_BTN)==HIGH){
		return UNPRESSED;
	}
	else{
		return PRESSED;
	}
}

int get_mode_button_state(){
	if(digitalRead(MODE_BTN)==HIGH){
		return UNPRESSED;
	}
	else{
		return PRESSED;
	}
}


////  DSM2  Spektrum  RC Stuff 
/*****************************************************************
* 
* 
* 
*****************************************************************/ 
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
	pthread_create(&uart4_thread, NULL, uart4_checker, (void*) NULL);
	printf("DSM2 Thread Started\n");
	return 0;
}

/*****************************************************************
* 
* 
* 
*****************************************************************/
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

/*****************************************************************
* 
* 
* 
*****************************************************************/
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

/*****************************************************************
* 
* 
* 
*****************************************************************/
int is_new_dsm2_data(){
	return new_dsm2_flag;
}

/*****************************************************************
* 
* 
* 
*****************************************************************/
int get_dsm2_frame_rate(){
	return dsm2_frame_rate;
}

// uncomment defines to print raw data for debugging
//#define DEBUG_DSM2
//#define DEBUG_DSM2_RAW
/*****************************************************************
* 
* 
* 
*****************************************************************/
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


/*****************************************************************
* int initialize_imu(int sample_rate, signed char orientation[9])
* 
* set up the imu and interrupt handler
*****************************************************************/
int initialize_imu(int sample_rate, signed char orientation[9]){
	printf("Initializing IMU\n");
	//set up gpio interrupt pin connected to imu
	if(gpio_export(INTERRUPT_PIN)){
		printf("can't export gpio %d \n", INTERRUPT_PIN);
		return (-1);
	}
	gpio_set_dir(INTERRUPT_PIN, INPUT_PIN);
	gpio_set_edge(INTERRUPT_PIN, "falling");  // Can be rising, falling or both
		
	//linux_set_i2c_bus(1);
	linux_set_i2c_bus(2);

	
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
	
	struct sched_param params;
	pthread_create(&imu_interrupt_thread, NULL, imu_interrupt_handler, (void*) NULL);
	params.sched_priority = sched_get_priority_max(SCHED_FIFO);
	pthread_setschedparam(imu_interrupt_thread, SCHED_FIFO, &params);
	
	return 0;
}

/*****************************************************************
* 
* 
* 
*****************************************************************/
int setXGyroOffset(int16_t offset) {
	uint16_t new = offset;
	const unsigned char msb = (unsigned char)((new&0xff00)>>8);
	const unsigned char lsb = (new&0x00ff);//get LSB
	//printf("writing: 0x%x 0x%x 0x%x \n", new, msb, lsb);
	linux_i2c_write(MPU_ADDR, MPU6050_RA_XG_OFFS_USRH, 1, &msb);
	return linux_i2c_write(MPU_ADDR, MPU6050_RA_XG_OFFS_USRL, 1, &lsb);
}

/*****************************************************************
* 
* 
* 
*****************************************************************/
int setYGyroOffset(int16_t offset) {
	uint16_t new = offset;
	const unsigned char msb = (unsigned char)((new&0xff00)>>8);
	const unsigned char lsb = (new&0x00ff);//get LSB
	//printf("writing: 0x%x 0x%x 0x%x \n", new, msb, lsb);
	linux_i2c_write(MPU_ADDR, MPU6050_RA_YG_OFFS_USRH, 1, &msb);
	return linux_i2c_write(MPU_ADDR, MPU6050_RA_YG_OFFS_USRL, 1, &lsb);
}

/*****************************************************************
* 
* 
* 
*****************************************************************/
int setZGyroOffset(int16_t offset) {
	uint16_t new = offset;
	const unsigned char msb = (unsigned char)((new&0xff00)>>8);
	const unsigned char lsb = (new&0x00ff);//get LSB
	//printf("writing: 0x%x 0x%x 0x%x \n", new, msb, lsb);
	linux_i2c_write(MPU_ADDR, MPU6050_RA_ZG_OFFS_USRH, 1, &msb);
	return linux_i2c_write(MPU_ADDR, MPU6050_RA_ZG_OFFS_USRL, 1, &lsb);
}

/*****************************************************************
* 
* 
* 
*****************************************************************/
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
/*****************************************************************
* 
* 
* 
*****************************************************************/
int set_imu_interrupt_func(int (*func)(void)){
	imu_interrupt_func = func;
	return 0;
}

/*****************************************************************
* 
* 
* 
*****************************************************************/
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
/*****************************************************************
* 
* 
* 
*****************************************************************/
int initialize_pru_servos(){
	// enbale clock signal to PRU
	int dev_mem;
	dev_mem = open("/dev/mem", O_RDWR);
	if(dev_mem == -1) {
		printf("Unable to open /dev/mem\n");
		return -1;
	}
	volatile char *cm_per_base;
	cm_per_base=mmap(0,CM_PER_PAGE_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,dev_mem,CM_PER);
	if(cm_per_base == (void *) -1) {
		printf("Unable to mmap cm_per\n");
		return -1;
	}
	*(uint16_t*)(cm_per_base + CM_PER_PRU_ICSS_CLKCTRL) |= MODULEMODE_ENABLE;
	*(uint16_t*)(cm_per_base + CM_PER_PRU_ICSS_CLKSTCTRL) |= MODULEMODE_ENABLE;
	close(dev_mem);
	
	
	// start pru
	#ifdef DEBUG
		printf("calling prussdrv_init()\n");
	#endif
    prussdrv_init();
	
    // Open PRU Interrupt
	#ifdef DEBUG
		printf("calling prussdrv_open\n");
	#endif
    if (prussdrv_open(PRU_EVTOUT_0)){
        printf("prussdrv_open open failed\n");
        return -1;
    }

    // Get the interrupt initialized
	#ifdef DEBUG
		printf("calling prussdrv_pruintc_init\n");
	#endif
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
    prussdrv_pruintc_init(&pruss_intc_initdata);

	// get pointer to PRU shared memory
	void* sharedMem = NULL;
    prussdrv_map_prumem(PRUSS0_SHARED_DATARAM, &sharedMem);
    prusharedMem_32int_ptr = (unsigned int*) sharedMem;
	memset(prusharedMem_32int_ptr, 0, SERVO_CHANNELS*4);
	
	// launch servo binary
	prussdrv_exec_program(PRU_SERVO_NUM, PRU_SERVO_BIN_LOCATION);

    return(0);
}

/*****************************************************************
* 
* 
* 
*****************************************************************/
int send_servo_pulse_us(int ch, int us){
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
	unsigned int num_loops = ((us*200.0)/PRU_SERVO_LOOP_INSTRUCTIONS); 
	
	// write to PRU shared memory
	prusharedMem_32int_ptr[ch-1] = num_loops;
	return 0;
}

/*****************************************************************
* 
* 
* 
*****************************************************************/
int send_servo_pulse_us_all(int us){
	int i;
	for(i=1;i<=SERVO_CHANNELS; i++){
		send_servo_pulse_us(i, us);
	}
	return 0;
}

/*****************************************************************
* 
* 
* 
*****************************************************************/
int send_servo_pulse_normalized(int ch, float input){
	if(ch<1 || ch>SERVO_CHANNELS){
		printf("ERROR: Servo Channel must be between 1 & %d \n", SERVO_CHANNELS);
		return -1;
	}
	if(input<-1.0 || input>1.0){
		printf("ERROR: normalized input must be between -1 & 1\n");
		return -1;
	}
	float micros = SERVO_MID_US + (input*(SERVO_EXTENDED_RANGE/2));
	return send_servo_pulse_us(ch, micros);
}

/*****************************************************************
* 
* 
* 
*****************************************************************/
int send_servo_pulse_normalized_all(float input){
	int i;
	for(i=1;i<=SERVO_CHANNELS; i++){
		send_servo_pulse_normalized(i, input);
	}
	return 0;
}

/*****************************************************************
* 
* 
* 
*****************************************************************/
int send_esc_pulse_normalized(int ch, float input){
	if(ch<1 || ch>SERVO_CHANNELS){
		printf("ERROR: Servo Channel must be between 1 & %d \n", SERVO_CHANNELS);
		return -1;
	}
	if(input<0.0 || input>1.0){
		printf("ERROR: normalized input must be between 0 & 1\n");
		return -1;
	}
	float micros = SERVO_MID_US + ((input-0.5)*SERVO_NORMAL_RANGE);
	return send_servo_pulse_us(ch, micros);
}

/*****************************************************************
* 
* 
* 
*****************************************************************/
int send_esc_pulse_normalized_all(float input){
	int i;
	for(i=1;i<=SERVO_CHANNELS; i++){
		send_esc_pulse_normalized(i, input);
	}
	return 0;
}

/*****************************************************************
* 
* 
* 
*****************************************************************/
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
/*****************************************************************
* 
* 
* 
*****************************************************************/
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

/*****************************************************************
* uint64_t microsSinceEpoch()
* 
* handy function for getting current time in microseconds
* so you don't have to deal with timespec structs
*****************************************************************/
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

	memset(&gcAddr, 0, sizeof(gcAddr));
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

/*********************************************************************************
*	saturate_float(float* val, float min, float max)
*
*	bounds val to +- limit
*	return one if saturation occurred, otherwise zero
**********************************************************************************/
int saturate_float(float* val, float min, float max){
	if(*val>max){
		*val = max;
		return 1;
	}
	else if(*val<min){	
		*val = min;
		return 1;
	}
	return 0;
}

/*********************************************************************************
*	is_cape_loaded()
*
*	check to make sure robotics cape overlay is loaded
*	return 1 if cape is loaded
*	return -1 if cape_mgr is missing
* 	return 0 if mape_mgr is present but cape is missing
**********************************************************************************/
int is_cape_loaded(){
	int ret;
	
	// first check if the old (Wheezy) location of capemanager exists
	if(system("ls /sys/devices/ | grep -q \"bone_capemgr\"")==0){
		#ifdef DEBUG
		printf("checking /sys/devices/bone_capemgr*/slots\n");
		#endif
		ret = system("grep -q "CAPE_NAME" /sys/devices/bone_capemgr*/slots");
	}
	else if(system("ls /sys/devices/platform/ | grep -q \"bone_capemgr\"")==0){
		#ifdef DEBUG
		printf("checking /sys/devices/platform/bone_capemgr*/slots\n");
		#endif
		ret = system("grep -q "CAPE_NAME" /sys/devices/platform/bone_capemgr*/slots");
	}
	else{
		printf("Cannot find bone_capemgr*/slots\n");
		return -1;
	}
	
	if(ret == 0){
		#ifdef DEBUG
		printf("Cape Loaded\n");
		#endif
		return 1;
	} 
	
	#ifdef DEBUG
	printf("Cape NOT Loaded\n");
	printf("grep returned %d\n", ret);
	#endif
	
	return 0;
}

