/*******************************************************************************
	robtics_cape.c

Copyright (c) 2016, James Strawson
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
*******************************************************************************/

// #define DEBUG

#include "useful_includes.h"
#include "robotics_cape.h"
#include "robotics_cape_defs.h"
#include "simple_gpio/simple_gpio.h"// used for setting interrupt input pin
#include "mmap/mmap_gpio_adc.h"		// used for fast gpio functions
#include "mmap/mmap_pwmss.h"		// used for fast pwm functions
#include "simple_pwm/simple_pwm.h" 	// for configuring pwm

// defines
#define PRU_ADDR		0x4A300000		// Start of PRU memory Page 184 am335x TRM
#define PRU_LEN			0x80000			// Length of PRU memory
#define PRU_SHAREDMEM	0x10000			// Offset to shared memory

/*******************************************************************************
* Global Variables
*******************************************************************************/
enum state_t state = UNINITIALIZED;
int pause_btn_state, mode_btn_state;
static unsigned int *prusharedMem_32int_ptr;


/*******************************************************************************
* local function declarations
*******************************************************************************/
int is_cape_loaded();
int initialize_button_handlers();
int (*pause_released_func)();
int (*pause_pressed_func)();
int (*mode_released_func)();
int (*mode_pressed_func)();
int initialize_pru();
void shutdown_signal_handler(int signo);

/*******************************************************************************
* local thread function declarations
*******************************************************************************/
void* pause_pressed_handler(void* ptr);
void* pause_released_handler(void* ptr);
void* mode_pressed_handler(void* ptr);
void* mode_released_handler(void* ptr);

/*******************************************************************************
* local thread structs
*******************************************************************************/
pthread_t pause_pressed_thread;
pthread_t pause_released_thread;
pthread_t mode_pressed_thread;
pthread_t mode_released_thread;


/*******************************************************************************
* int initialize_cape()
* sets up necessary hardware and software
* should be the first thing your program calls
*******************************************************************************/
int initialize_cape(){
	FILE *fd; 

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
	fprintf(fd,"%d",(int)current_pid);
	fflush(fd);
	fclose(fd);
	
	// Start Signal Handler
	#ifdef DEBUG
	printf("Initializing exit signal handler\n");
	#endif
	signal(SIGINT, shutdown_signal_handler);	
	signal(SIGTERM, shutdown_signal_handler);	

	// // check the device tree overlay is actually loaded
	// if (is_cape_loaded() != 1){
		// printf("ERROR: Device tree overlay not loaded by cape manager\n");
		// return -1;
	// }
	
	// initialize mmap io libs
	#ifdef DEBUG
	printf("Initializing: ");
	printf("GPIO");
	fflush(stdout);
	#endif

	//export all GPIO output pins
	gpio_export(RED_LED);
	gpio_set_dir(RED_LED, OUTPUT_PIN);
	gpio_export(GRN_LED);
	gpio_set_dir(GRN_LED, OUTPUT_PIN);
	gpio_export(MDIR1A);
	gpio_set_dir(MDIR1A, OUTPUT_PIN);
	gpio_export(MDIR1B);
	gpio_set_dir(MDIR1B, OUTPUT_PIN);
	gpio_export(MDIR2A);
	gpio_set_dir(MDIR2A, OUTPUT_PIN);
	gpio_export(MDIR2B);
	gpio_set_dir(MDIR2B, OUTPUT_PIN);
	gpio_export(MDIR3A);
	gpio_set_dir(MDIR3A, OUTPUT_PIN);
	gpio_export(MDIR3B);
	gpio_set_dir(MDIR3B, OUTPUT_PIN);
	gpio_export(MDIR4A);
	gpio_set_dir(MDIR4A, OUTPUT_PIN);
	gpio_export(MDIR4B);
	gpio_set_dir(MDIR4B, OUTPUT_PIN);
	gpio_export(MOT_STBY);
	gpio_set_dir(MOT_STBY, OUTPUT_PIN);
	gpio_export(PAIRING_PIN);
	gpio_set_dir(PAIRING_PIN, OUTPUT_PIN);
	gpio_export(INTERRUPT_PIN);
	gpio_set_dir(INTERRUPT_PIN, INPUT_PIN);
	gpio_export(SERVO_PWR);
	gpio_set_dir(SERVO_PWR, OUTPUT_PIN);
	
	if(initialize_mmap_gpio()){
		printf("mmap_gpio_adc.c failed to initialize gpio\n");
		return -1;
	}
	
	#ifdef DEBUG
	printf(" ADC");
	fflush(stdout);
	#endif
	if(initialize_mmap_adc()){
		printf("mmap_gpio_adc.c failed to initialize adc\n");
		return -1;
	}

	#ifdef DEBUG
	printf(" eQEP");
	fflush(stdout);
	#endif
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
	
	// setup pwm driver
	#ifdef DEBUG
	printf(" PWM");
	fflush(stdout);
	#endif
	if(simple_init_pwm(1,PWM_FREQ)){
		printf("simple_pwm.c failed to initialize PWMSS 1\n");
		return -1;
	}
	if(simple_init_pwm(2,PWM_FREQ)){
		printf("simple_pwm.c failed to initialize PWMSS 2\n");
		return -1;
	}
	
	// start some gpio pins at defaults
	deselect_spi1_slave(1);	
	deselect_spi1_slave(2);
	disable_motors();
	
	//set up function pointers for button press events
	#ifdef DEBUG
	printf(" Buttons");
	fflush(stdout);
	#endif
	initialize_button_handlers();
	
	// start PRU
	#ifdef DEBUG
	printf(" PRU\n");
	fflush(stdout);
	#endif
	initialize_pru();
	
	// Print current battery voltage
	#ifdef DEBUG
	printf("Battery: %2.2fV  ", get_battery_voltage());
	printf("Process ID: %d\n", (int)current_pid);
	#endif

	// all done
	set_state(PAUSED);

	return 0;
}

/*******************************************************************************
*	int cleanup_cape()
*	shuts down library and hardware functions cleanly
*	you should call this before your main() function returns
*******************************************************************************/
int cleanup_cape(){
	// just in case the user forgot, set state to exiting
	set_state(EXITING);
	
	// announce we are starting cleanup process
	printf("\nExiting Cleanly\n");
	
	//allow up to 3 seconds for thread cleanup
	struct timespec thread_timeout;
	clock_gettime(CLOCK_REALTIME, &thread_timeout);
	thread_timeout.tv_sec += 3;
	int thread_err = 0;
	thread_err = pthread_timedjoin_np(pause_pressed_thread, NULL, \
															&thread_timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: pause_pressed_thread exit timeout\n");
	}
	thread_err = 0;
	thread_err = pthread_timedjoin_np(pause_released_thread, NULL, \
															&thread_timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: pause_released_thread exit timeout\n");
	}
	thread_err = 0;
	thread_err = pthread_timedjoin_np(mode_pressed_thread, NULL, \
															&thread_timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: mode_pressed_thread exit timeout\n");
	}
	thread_err = 0;
	thread_err = pthread_timedjoin_np(mode_released_thread, NULL, \
															&thread_timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: mode_released_thread exit timeout\n");
	}
	
	
	#ifdef DEBUG
	printf("turning off GPIOs & PWM\n");
	#endif
	set_led(GREEN,LOW);
	set_led(RED,LOW);	
	disable_motors();
	deselect_spi1_slave(1);	
	deselect_spi1_slave(2);	
	disable_servo_power_rail();
	
	#ifdef DEBUG
	printf("stopping dsm2 service\n");
	#endif
	stop_dsm2_service();	
	
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
	return 0;
}


/*******************************************************************************
* @ state_t get_state()
*
* returns the high-level robot state variable
* use this for managing how your threads start and stop
*******************************************************************************/
state_t get_state(){
	return state;
}


/*******************************************************************************
* @ int set_state(state_t new_state)
*
* sets the high-level robot state variable
* use this for managing how your threads start and stop
*******************************************************************************/
int set_state(state_t new_state){
	state = new_state;
	return 0;
}

/*******************************************************************************
* @ int print_state()
* 
* Prints the textual name of the state to the screen.
*******************************************************************************/
int print_state(){
	switch(state){
	case UNINITIALIZED:
		printf("UNINITIALIZED");
		break;
	case PAUSED:
		printf("PAUSED");
		break;
	case RUNNING:
		printf("RUNNING");
		break;
	case EXITING:
		printf("EXITING");
		break;
	default:
		printf("ERROR: invalid state\n");
		return -1;
	}
	return 0;
}


/*******************************************************************************
* @ int set_led(led_t led, int state)
* 
* turn on or off the green or red LED on robotics cape
* if state is 0, turn led off, otherwise on.
* we suggest using the names HIGH or LOW
*******************************************************************************/
int set_led(led_t led, int state){
	int val;
	if(state) val = HIGH;
	else val = LOW;
	
	switch(led){
	case GREEN:
		return mmap_gpio_write(GRN_LED, val);
		break;
	case RED:
		return mmap_gpio_write(RED_LED, val);
		break;
	default:
		printf("LED must be GREEN or RED\n");
		break;
	}
	return -1;
}

/*******************************************************************************
* int get_led_state(led_t led)
* 
* returns the state of the green or red LED on robotics cape
* state is LOW(0), or HIGH(1)
*******************************************************************************/
int get_led_state(led_t led){
	int ret= -1;
	switch(led){
	case GREEN:
		gpio_get_value(GRN_LED, &ret);
		break;
	case RED:
		gpio_get_value(RED_LED, &ret);
		break;
	default:
		printf("LED must be GREEN or RED\n");
		ret = -1;
		break;
	}
	return ret;
}

/*******************************************************************************
* blink_led()
*	
* Flash an LED at a set frequency for a finite period of time.
* This is a blocking call and only returns after flashing.
*******************************************************************************/
int blink_led(led_t led, float hz, float period){
	const int delay_us = 1000000.0/(2.0*hz); 
	const int blinks = period*2.0*hz;
	int i;
	int toggle = 0;
	
	for(i=0;i<blinks;i++){
		toggle = !toggle;
		if(get_state()==EXITING) break;
		set_led(led,toggle);
		// wait for next blink
		usleep(delay_us);
	}
	
	set_led(led, 0); // make sure it is left off
	return 0;
}

/*******************************************************************************
*	int initialize_button_interrups()
*
*	start 4 threads to handle 4 interrupt routines for pressing and
*	releasing the two buttons.
*******************************************************************************/
int initialize_button_handlers(){
	
	#ifdef DEBUG
	printf("\nsetting up mode & pause gpio pins\n");
	#endif
	//set up mode pi
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
	set_pause_released_func(&null_func);
	set_mode_pressed_func(&null_func);
	set_mode_released_func(&null_func);
	
	
	pthread_create(&pause_pressed_thread, &attr,			 \
				pause_pressed_handler, (void*) NULL);
	pthread_create(&pause_released_thread, &attr,			 \
				pause_released_handler, (void*) NULL);
	pthread_create(&mode_pressed_thread, &attr,			 \
					mode_pressed_handler, (void*) NULL);
	pthread_create(&mode_released_thread, &attr,			 \
					mode_released_handler, (void*) NULL);
	
	// apply medium priority to all threads
	pthread_setschedparam(pause_pressed_thread, SCHED_FIFO, &params);
	pthread_setschedparam(pause_released_thread, SCHED_FIFO, &params);
	pthread_setschedparam(mode_pressed_thread, SCHED_FIFO, &params);
	pthread_setschedparam(mode_released_thread, SCHED_FIFO, &params);
	 
	return 0;
}

/*******************************************************************************
*	void* pause_pressed_handler(void* ptr)
* 
*	wait on falling edge of pause button
*******************************************************************************/
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
			if(get_pause_button()==PRESSED){
				pause_pressed_func(); 
			}
		}
	}
	gpio_fd_close(gpio_fd);
	return 0;
}

/*******************************************************************************
* @ void* pause_released_handler(void* ptr) 
*
* wait on rising edge of pause button
*******************************************************************************/
void* pause_released_handler(void* ptr){
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
			if(get_pause_button()==RELEASED){
				pause_released_func(); 
			}
		}
	}
	gpio_fd_close(gpio_fd);
	return 0;
}

/*******************************************************************************
*	void* mode_pressed_handler(void* ptr) 
*	wait on falling edge of mode button
*******************************************************************************/
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
			if(get_mode_button()==PRESSED){
				mode_pressed_func(); 
			}
		}
	}
	gpio_fd_close(gpio_fd);
	return 0;
}

/*******************************************************************************
*	void* mode_released_handler(void* ptr) 
*	wait on rising edge of mode button
*******************************************************************************/
void* mode_released_handler(void* ptr){
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
			if(get_mode_button()==RELEASED){
				mode_released_func(); 
			}
		}
	}
	gpio_fd_close(gpio_fd);
	return 0;
}

/*******************************************************************************
*	button function assignments
*******************************************************************************/
int set_pause_pressed_func(int (*func)(void)){
	pause_pressed_func = func;
	return 0;
}
int set_pause_released_func(int (*func)(void)){
	pause_released_func = func;
	return 0;
}
int set_mode_pressed_func(int (*func)(void)){
	mode_pressed_func = func;
	return 0;
}
int set_mode_released_func(int (*func)(void)){
	mode_released_func = func;
	return 0;
}

/*******************************************************************************
*	button_state_t get_pause_button()
*******************************************************************************/
button_state_t get_pause_button(){
	if(mmap_gpio_read(PAUSE_BTN)==HIGH){
		return RELEASED;
	}
	else{
		return PRESSED;
	}
}

/********************************************************************************
*	button_state_t get_mode_button()
*******************************************************************************/
button_state_t get_mode_button(){
	if(mmap_gpio_read(MODE_BTN)==HIGH){
		return RELEASED;
	}
	else{
		return PRESSED;
	}
}

/*******************************************************************************
* enable_motors()
* 
* turns on the standby pin to enable the h-bridge ICs
* returns 0 on success
*******************************************************************************/
int enable_motors(){
	set_motor_free_spin_all();
	return mmap_gpio_write(MOT_STBY, HIGH);
}

/*******************************************************************************
* int disable_motors()
* 
* turns off the standby pin to disable the h-bridge ICs
* and disables PWM output signals, returns 0 on success
*******************************************************************************/
int disable_motors(){
	set_motor_free_spin_all();
	return mmap_gpio_write(MOT_STBY, LOW);
}

/*******************************************************************************
* int set_motor(int motor, float duty)
* 
* set a motor direction and power
* motor is from 1 to 4, duty is from -1.0 to +1.0
*******************************************************************************/
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
			mmap_gpio_write(MDIR1A, a);
			mmap_gpio_write(MDIR1B, b);
			set_pwm_duty(1, 'A', duty);
			break;
		case 2:
			mmap_gpio_write(MDIR2A, b);
			mmap_gpio_write(MDIR2B, a);
			set_pwm_duty(1, 'B', duty);
			break;
		case 3:
			mmap_gpio_write(MDIR3A, b);
			mmap_gpio_write(MDIR3B, a);
			set_pwm_duty(2, 'A', duty);
			break;
		case 4:
			mmap_gpio_write(MDIR4A, a);
			mmap_gpio_write(MDIR4B, b);
			set_pwm_duty(2, 'B', duty);
			break;
		default:
			printf("enter a motor value between 1 and 4\n");
			return -1;
	}
	return 0;
}

/*******************************************************************************
* int set_motor_all(float duty)
* 
* applies the same duty cycle argument to all 4 motors
*******************************************************************************/
int set_motor_all(float duty){
	int i;
	for(i=1;i<=MOTOR_CHANNELS; i++){
		set_motor(i, duty);
	}
	return 0;
}

/*******************************************************************************
* int set_motor_free_spin(int motor)
* 
* This puts one or all motor outputs in high-impedance state which lets the 
* motor spin freely as if it wasn't connected to anything.
*******************************************************************************/
int set_motor_free_spin(int motor){
	
	// set gpio direction outputs & duty
	switch(motor){
		case 1:
			mmap_gpio_write(MDIR1A, 0);
			mmap_gpio_write(MDIR1B, 0);
			set_pwm_duty(1, 'A', 0.0);
			break;
		case 2:
			mmap_gpio_write(MDIR2A, 0);
			mmap_gpio_write(MDIR2B, 0);
			set_pwm_duty(1, 'B', 0.0);
			break;
		case 3:
			mmap_gpio_write(MDIR3A, 0);
			mmap_gpio_write(MDIR3B, 0);
			set_pwm_duty(2, 'A', 0.0);
			break;
		case 4:
			mmap_gpio_write(MDIR4A, 0);
			mmap_gpio_write(MDIR4B, 0);
			set_pwm_duty(2, 'B', 0.0);
			break;
		default:
			printf("enter a motor value between 1 and 4\n");
			return -1;
	}
	return 0;
}

/*******************************************************************************
* @ int set_motor_free_spin_all()
*******************************************************************************/
int set_motor_free_spin_all(){
	int i;
	for(i=1;i<=MOTOR_CHANNELS; i++){
		set_motor_free_spin(i);
	}
	return 0;
}

/*******************************************************************************
* int set_motor_brake(int motor)
* 
* These will connect one or all motor terminal pairs together which
* makes the motor fight against its own back EMF turning it into a brake.
*******************************************************************************/
int set_motor_brake(int motor){

	// set gpio direction outputs & duty
	switch(motor){
		case 1:
			mmap_gpio_write(MDIR1A, 1);
			mmap_gpio_write(MDIR1B, 1);
			set_pwm_duty(1, 'A', 0.0);
			break;
		case 2:
			mmap_gpio_write(MDIR2A, 1);
			mmap_gpio_write(MDIR2B, 1);
			set_pwm_duty(1, 'B', 0.0);
			break;
		case 3:
			mmap_gpio_write(MDIR3A, 1);
			mmap_gpio_write(MDIR3B, 1);
			set_pwm_duty(2, 'A', 0.0);
			break;
		case 4:
			mmap_gpio_write(MDIR4A, 1);
			mmap_gpio_write(MDIR4B, 1);
			set_pwm_duty(2, 'B', 0.0);
			break;
		default:
			printf("enter a motor value between 1 and 4\n");
			return -1;
	}
	return 0;
}

/*******************************************************************************
* @ int set_motor_brake_all()
*******************************************************************************/
int set_motor_brake_all(){
	int i;
	for(i=1;i<=MOTOR_CHANNELS; i++){
		set_motor_brake(i);
	}
	return 0;
}



/*******************************************************************************
* int get_encoder_pos(int ch)
* 
* returns the encoder counter position
*******************************************************************************/
int get_encoder_pos(int ch){
	if(ch<1 || ch>4){
		printf("Encoder Channel must be from 1 to 4\n");
		return -1;
	}
	// 4th channel is counted by the PRU not eQEP
	if(ch==4){
		return (int) prusharedMem_32int_ptr[8];
	}
	
	// first 3 channels counted by eQEP
	return  read_eqep(ch-1);
}


/*******************************************************************************
* int set_encoder_pos(int ch, int val)
* 
* sets the encoder counter position
*******************************************************************************/
int set_encoder_pos(int ch, int val){
	if(ch<1 || ch>4){
		printf("Encoder Channel must be from 1 to 4\n");
		return -1;
	}
	// 4th channel is counted by the PRU not eQEP
	if(ch==4){
		prusharedMem_32int_ptr[8] = val;
		return 0;
	}
	// else write to eQEP
	return write_eqep(ch-1, val);
}



/*******************************************************************************
* float get_battery_voltage()
* 
* returns the LiPo battery voltage on the robotics cape
* this accounts for the voltage divider ont he cape
*******************************************************************************/
float get_battery_voltage(){
	float v = (get_adc_volt(LIPO_ADC_CH)*V_DIV_RATIO)-LIPO_OFFSET; 
	if(v<0.3) v = 0.0;
	return v;
}

/*******************************************************************************
* float get_dc_jack_voltage()
* 
* returns the DC power jack voltage on the robotics cape
* this accounts for the voltage divider ont he cape
*******************************************************************************/
float get_dc_jack_voltage(){
	float v = (get_adc_volt(DC_JACK_ADC_CH)*V_DIV_RATIO)-DC_JACK_OFFSET; 
	if(v<0.3) v = 0.0;
	return v;
}

/*******************************************************************************
* int get_adc_raw(int ch)
*
* returns the raw adc reading
*******************************************************************************/
int get_adc_raw(int ch){
	if(ch<0 || ch>6){
		printf("analog pin must be in 0-6\n");
		return -1;
	}
	return mmap_adc_read_raw((uint8_t)ch);
}

/*******************************************************************************
* float get_adc_volt(int ch)
* 
* returns an actual voltage for an adc channel
*******************************************************************************/
float get_adc_volt(int ch){
	if(ch<0 || ch>6){
		printf("analog pin must be in 0-6\n");
		return -1;
	}
	int raw_adc = mmap_adc_read_raw((uint8_t)ch);
	return raw_adc * 1.8 / 4095.0;
}



/*******************************************************************************
* int initialize_pru()
* 
* Enables the PRU and gets a pointer to the PRU shared memory which is used by 
* the servo and encoder functions in this C file.
*******************************************************************************/
int initialize_pru(){
	
	unsigned int	*pru;		// Points to start of PRU memory.
	int	fd;
	
	fd = open ("/dev/mem", O_RDWR | O_SYNC);
	if (fd == -1) {
		printf ("ERROR: could not open /dev/mem.\n\n");
		return 1;
	}
	pru = mmap (0, PRU_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PRU_ADDR);
	if (pru == MAP_FAILED) {
		printf ("ERROR: could not map memory.\n\n");
		return 1;
	}
	close(fd);

	// set global shared memory pointer
	prusharedMem_32int_ptr = pru + PRU_SHAREDMEM/4;	// Points to start of shared memory

	// zero out the 8 servo channels and encoder channel
	memset(prusharedMem_32int_ptr, 0, 9*4);

	// reset each core
	system("echo 4a334000.pru0 > /sys/bus/platform/drivers/pru-rproc/unbind  > /dev/null");
	system("echo 4a334000.pru0 > /sys/bus/platform/drivers/pru-rproc/bind > /dev/null");
	system("echo 4a338000.pru1  > /sys/bus/platform/drivers/pru-rproc/unbind > /dev/null");
	system("echo 4a338000.pru1 > /sys/bus/platform/drivers/pru-rproc/bind > /dev/null");

	
    return 0;
}

/*******************************************************************************
* int power_off_pru()
* 
* Enables the PRU and gets a pointer to the PRU shared memory which is used by 
* the servo and encoder functions in this C file.
*******************************************************************************/
int power_off_pru(){
	
	// set pointer to NULL
    prusharedMem_32int_ptr = NULL;

    // unbind both cores
	system("echo 4a334000.pru0 > /sys/bus/platform/drivers/pru-rproc/unbind 2>/dev/null");
	system("echo 4a338000.pru1  > /sys/bus/platform/drivers/pru-rproc/unbind 2> /dev/null");

    return 0;
}

/*******************************************************************************
* int enable_servo_power_rail()
* 
* Turns on the 6V power regulator to the servo power rail.
*******************************************************************************/
int enable_servo_power_rail(){
	return mmap_gpio_write(SERVO_PWR, HIGH);
}

/*******************************************************************************
* int disable_servo_power_rail()
* 
* Turns off the 6V power regulator to the servo power rail.
*******************************************************************************/
int disable_servo_power_rail(){
	return mmap_gpio_write(SERVO_PWR, LOW);
}

/*******************************************************************************
* int send_servo_pulse_us(int ch, int us)
* 
* Sends a single pulse of duration us (microseconds) to a single channel (ch)
* This must be called regularly (>40hz) to keep servo or ESC awake.
*******************************************************************************/
int send_servo_pulse_us(int ch, int us){
	// Sanity Checks
	if(ch<1 || ch>SERVO_CHANNELS){
		printf("ERROR: Servo Channel must be between 1&%d\n", SERVO_CHANNELS);
		return -1;
	} if(prusharedMem_32int_ptr == NULL){
		printf("ERROR: PRU servo Controller not initialized\n");
		return -1;
	}
	// PRU runs at 200Mhz. find #loops needed
	unsigned int num_loops = ((us*200.0)/PRU_SERVO_LOOP_INSTRUCTIONS); 
	// write to PRU shared memory
	prusharedMem_32int_ptr[ch-1] = num_loops;
	return 0;
}

/*******************************************************************************
* int send_servo_pulse_us_all(int us)
* 
* Sends a single pulse of duration us (microseconds) to all channels.
* This must be called regularly (>40hz) to keep servos or ESCs awake.
*******************************************************************************/
int send_servo_pulse_us_all(int us){
	int i;
	for(i=1;i<=SERVO_CHANNELS; i++){
		send_servo_pulse_us(i, us);
	}
	return 0;
}

/*******************************************************************************
* int send_servo_pulse_normalized(int ch, float input)
* 
*
*******************************************************************************/
int send_servo_pulse_normalized(int ch, float input){
	if(ch<1 || ch>SERVO_CHANNELS){
		printf("ERROR: Servo Channel must be between 1&%d\n", SERVO_CHANNELS);
		return -1;
	}
	if(input<-1.5 || input>1.5){
		printf("ERROR: normalized input must be between -1 & 1\n");
		return -1;
	}
	float micros = SERVO_MID_US + (input*(SERVO_NORMAL_RANGE/2));
	return send_servo_pulse_us(ch, micros);
}

/*******************************************************************************
* int send_servo_pulse_normalized_all(float input)
* 
* 
*******************************************************************************/
int send_servo_pulse_normalized_all(float input){
	int i;
	for(i=1;i<=SERVO_CHANNELS; i++){
		send_servo_pulse_normalized(i, input);
	}
	return 0;
}

/*******************************************************************************
* int send_esc_pulse_normalized(int ch, float input)
* 
* 
*******************************************************************************/
int send_esc_pulse_normalized(int ch, float input){
	if(ch<1 || ch>SERVO_CHANNELS){
		printf("ERROR: Servo Channel must be between 1&%d\n", SERVO_CHANNELS);
		return -1;
	}
	if(input<0.0 || input>1.0){
		printf("ERROR: normalized input must be between 0 & 1\n");
		return -1;
	}
	float micros = SERVO_MID_US + ((input-0.5)*SERVO_NORMAL_RANGE);
	return send_servo_pulse_us(ch, micros);
}

/*******************************************************************************
* int send_esc_pulse_normalized_all(float input)
* 
* 
*******************************************************************************/
int send_esc_pulse_normalized_all(float input){
	int i;
	for(i=1;i<=SERVO_CHANNELS; i++){
		send_esc_pulse_normalized(i, input);
	}
	return 0;
}


/*******************************************************************************
* shutdown_signal_handler(int signo)
*
* catch Ctrl-C signal and change system state to EXITING
* all threads should watch for get_state()==EXITING and shut down cleanly
*******************************************************************************/
void shutdown_signal_handler(int signo){
	if (signo == SIGINT){
		set_state(EXITING);
		printf("\nreceived SIGINT Ctrl-C\n");
 	}else if (signo == SIGTERM){
		set_state(EXITING);
		printf("\nreceived SIGTERM\n");
 	}
}


/*******************************************************************************
*	is_cape_loaded()
*
*	check to make sure robotics cape overlay is loaded
*	return 1 if cape is loaded
*	return -1 if cape_mgr is missing
* 	return 0 if mape_mgr is present but cape is missing
*******************************************************************************/
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


/*******************************************************************************
* @ int kill_robot()
*
* This function is used by initialize_cape to make sure any existing program
* using the robotics cape lib is stopped. The user doesn't need to integrate
* this in their own program as initialize_cape calls it. However, the user may
* call the kill_robot example program from the command line to close whatever
* program is running in the background.
*
* return values: 
* -2 : invalid contents in PID_FILE
* -1 : existing project failed to close cleanly and had to be killed
*  0 : No existing program is running
*  1 : An existing program was running but it shut down cleanly.
*******************************************************************************/
int kill_robot(){
	FILE* fd;
	int old_pid, i;
	
	// attempt to open PID file
	fd = fopen(PID_FILE, "r");
	// if the file didn't open, no proejct is runnning in the background
	// so return 0
	if (fd == NULL) {
		return 0;
	}
	
	// otherwise try to read the current process ID
	fscanf(fd,"%d", &old_pid);
	fclose(fd);
	
	// if the file didn't contain a PID number, remove it and 
	// return -1 indicating weird behavior
	if(old_pid == 0){
		remove(PID_FILE);
		return -2;
	}
		
	// attempt a clean shutdown
	kill((pid_t)old_pid, SIGINT);
	
	// check every 0.1 seconds to see if it closed 
	for(i=0; i<30; i++){
		if(access(PID_FILE, F_OK ) != -1) usleep(100000);
		else return 1; // succcess, it shut down properly
	}
	
	// otherwise force kill the program if the PID file never got cleaned up
	kill((pid_t)old_pid, SIGKILL);

	// close and delete the old file
	fclose(fd);
	remove(PID_FILE);
	
	// return -1 indicating the program had to be killed
	return -1;
}


