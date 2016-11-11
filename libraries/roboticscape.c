/*******************************************************************************
* robticscape.c
* 
* This is one of many c-files that are used to build libroboticscape.so
* however it contains the majority of the core components.
*******************************************************************************/

//#define DEBUG

#include "roboticscape-usefulincludes.h"
#include "roboticscape.h"
#include "roboticscape-defs.h"
#include "simple_gpio/gpio_setup.h"
#include "mmap/mmap_gpio_adc.h"		// used for fast gpio functions
#include "mmap/mmap_pwmss.h"		// used for fast pwm functions
#include "other/robotics_pru.h"


#define CAPE_NAME 	"RoboticsCape"
#define MAX_BUF 	512

/*******************************************************************************
* Global Variables
*******************************************************************************/
enum state_t state = UNINITIALIZED;
int pause_btn_state, mode_btn_state;
int mdir1a, mdir2b; // variable gpio pin assignments



/*******************************************************************************
* local function declarations
*******************************************************************************/
int is_cape_loaded();
int initialize_button_handlers();
int (*pause_released_func)();
int (*pause_pressed_func)();
int (*mode_released_func)();
int (*mode_pressed_func)();
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
	kill_robot();
	
	// Start Signal Handler
	#ifdef DEBUG
	printf("Initializing exit signal handler\n");
	#endif
	signal(SIGINT, shutdown_signal_handler);	
	signal(SIGTERM, shutdown_signal_handler);	


	// do any board-specific config
	if(get_bb_model()==BB_BLUE){
		mdir1a = MDIR1A_BLUE;
		mdir2b = MDIR2B_BLUE;
	}
	else{
		mdir1a = MDIR1A;
		mdir2b = MDIR2B;
	}

	// initialize pinmux
	#ifdef DEBUG
	printf("Initializing: PINMUX\n");
	#endif
	set_default_pinmux();

	// initialize gpio pins
	#ifdef DEBUG
	printf("Initializing: GPIO\n");
	#endif
	if(configure_gpio_pins()<0){
		printf("ERROR: failed to configure GPIO\n");
		return -1;
	}

	// now use mmap for fast gpio
	#ifdef DEBUG
	printf("Initializing: MMAP GPIO\n");
	#endif
	if(initialize_mmap_gpio()){
		printf("mmap_gpio_adc.c failed to initialize gpio\n");
		return -1;
	}
	
	// now adc
	#ifdef DEBUG
	printf("Initializing: ADC\n");
	#endif
	if(initialize_mmap_adc()){
		printf("mmap_gpio_adc.c failed to initialize adc\n");
		return -1;
	}

	// now eqep
	#ifdef DEBUG
	printf("Initializing: eQEP\n");
	#endif
	if(init_eqep(0)){
		printf("ERROR: failed to initialize eQEP0\n");
		// return -1;
	}
	if(init_eqep(1)){
		printf("ERROR: failed to initialize eQEP1\n");
		// return -1;
	}
	if(init_eqep(2)){
		printf("ERROR: failed to initialize eQEP2\n");
		// return -1;
	}
	
	// now pwm
	#ifdef DEBUG
	printf("Initializing: PWM\n");
	#endif
	if(simple_init_pwm(1,PWM_FREQ)){
		printf("ERROR: failed to initialize hrpwm1\n");
		return -1;
	}
	if(simple_init_pwm(2,PWM_FREQ)){
		printf("ERROR: failed to initialize PWMSS 2\n");
		return -1;
	}
	
	//set up function pointers for button press events
	#ifdef DEBUG
	printf("Initializing: Buttons\n");
	#endif
	if(initialize_button_handlers()<0){
		printf("ERROR: failed to start button threads\n");
		return -1;
	}
	
	// start PRU
	#ifdef DEBUG
	printf("Initializing: PRU\n");
	#endif
	initialize_pru();

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

	// Print current PID
	#ifdef DEBUG
	printf("Process ID: %d\n", (int)current_pid); 
 	#endif

	// wait to let threads start up
	usleep(10000);
	
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
	manual_deselect_spi_slave(1);	
	manual_deselect_spi_slave(2);	
	disable_servo_power_rail();
	
	#ifdef DEBUG
	printf("stopping dsm service\n");
	#endif
	stop_dsm_service();	
	
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
			// delay debouce
			usleep(500); 
			if(get_pause_button()==PRESSED){
				usleep(500);
				if(get_pause_button()==PRESSED){
					usleep(500);
					if(get_pause_button()==PRESSED){
						pause_pressed_func(); 
					}
				}
			}
			// purge any interrupts that may have stacked up
			lseek(fdset[0].fd, 0, SEEK_SET);  
			read(fdset[0].fd, buf, MAX_BUF);
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
			// delay debouce
			usleep(500); 
			if(get_pause_button()==RELEASED){
				usleep(500);
				if(get_pause_button()==RELEASED){
					usleep(500);
					if(get_pause_button()==RELEASED){
						pause_released_func(); 
					}
				}
			}
			// purge any interrupts that may have stacked up
			lseek(fdset[0].fd, 0, SEEK_SET);  
			read(fdset[0].fd, buf, MAX_BUF);
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
			// delay debouce
			usleep(500); 
			if(get_mode_button()==PRESSED){
				usleep(500);
				if(get_mode_button()==PRESSED){
					usleep(500);
					if(get_mode_button()==PRESSED){
						mode_pressed_func(); 
					}
				}
			}
			// purge any interrupts that may have stacked up
			lseek(fdset[0].fd, 0, SEEK_SET);  
			read(fdset[0].fd, buf, MAX_BUF);
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
			// delay debouce
			usleep(500); 
			if(get_mode_button()==RELEASED){
				usleep(500);
				if(get_mode_button()==RELEASED){
					usleep(500);
					if(get_mode_button()==RELEASED){
						mode_released_func(); 
					}
				}
			}
			// purge any interrupts that may have stacked up
			lseek(fdset[0].fd, 0, SEEK_SET);  
			read(fdset[0].fd, buf, MAX_BUF);
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
			mmap_gpio_write(mdir1a, a);
			mmap_gpio_write(MDIR1B, b);
			mmap_set_pwm_duty(1, 'A', duty);
			break;
		case 2:
			mmap_gpio_write(MDIR2A, b);
			mmap_gpio_write(mdir2b, a);
			mmap_set_pwm_duty(1, 'B', duty);
			break;
		case 3:
			mmap_gpio_write(MDIR3A, b);
			mmap_gpio_write(MDIR3B, a);
			mmap_set_pwm_duty(2, 'A', duty);
			break;
		case 4:
			mmap_gpio_write(MDIR4A, a);
			mmap_gpio_write(MDIR4B, b);
			mmap_set_pwm_duty(2, 'B', duty);
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
			mmap_gpio_write(mdir1a, 0);
			mmap_gpio_write(MDIR1B, 0);
			mmap_set_pwm_duty(1, 'A', 0.0);
			break;
		case 2:
			mmap_gpio_write(MDIR2A, 0);
			mmap_gpio_write(mdir2b, 0);
			mmap_set_pwm_duty(1, 'B', 0.0);
			break;
		case 3:
			mmap_gpio_write(MDIR3A, 0);
			mmap_gpio_write(MDIR3B, 0);
			mmap_set_pwm_duty(2, 'A', 0.0);
			break;
		case 4:
			mmap_gpio_write(MDIR4A, 0);
			mmap_gpio_write(MDIR4B, 0);
			mmap_set_pwm_duty(2, 'B', 0.0);
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
			mmap_gpio_write(mdir1a, 1);
			mmap_gpio_write(MDIR1B, 1);
			mmap_set_pwm_duty(1, 'A', 0.0);
			break;
		case 2:
			mmap_gpio_write(MDIR2A, 1);
			mmap_gpio_write(mdir2b, 1);
			mmap_set_pwm_duty(1, 'B', 0.0);
			break;
		case 3:
			mmap_gpio_write(MDIR3A, 1);
			mmap_gpio_write(MDIR3B, 1);
			mmap_set_pwm_duty(2, 'A', 0.0);
			break;
		case 4:
			mmap_gpio_write(MDIR4A, 1);
			mmap_gpio_write(MDIR4B, 1);
			mmap_set_pwm_duty(2, 'B', 0.0);
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
	if(ch==4) return get_pru_encoder_pos();
	
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
	if(ch==4) return set_pru_encoder_pos(val);

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
	float v = (get_adc_volt(LIPO_ADC_CH)*V_DIV_RATIO)+LIPO_OFFSET; 
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
	float v = (get_adc_volt(DC_JACK_ADC_CH)*V_DIV_RATIO)+DC_JACK_OFFSET; 
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

	// start by checking if a pid file exists
	if(access(PID_FILE, F_OK ) != 0){
		// PID file missing
		return 0;
	}

	// attempt to open PID file
	// if the file didn't open, no project is runnning in the background
	// so return 0
	fd = fopen(PID_FILE, "r");
	if (fd == NULL) return 0;
	
	// try to read the current process ID
	fscanf(fd,"%d", &old_pid);
	fclose(fd);
	
	// if the file didn't contain a PID number, remove it and 
	// return -1 indicating weird behavior
	if(old_pid == 0){
		remove(PID_FILE);
		return -2;
	}

	// check if it's our own pid, if so return 0
	if(old_pid == (int)getpid()) return 0;
	
	// now see if the process for the read pid is still running
	if(getpgid(old_pid) < 0){
		// process not running, remove the pid file
		remove(PID_FILE);
		return 0;
	}

	// process must be running, attempt a clean shutdown
	kill((pid_t)old_pid, SIGINT);
	
	// check every 0.1 seconds to see if it closed 
	for(i=0; i<30; i++){
		if(getpgid(old_pid) >= 0) usleep(100000);
		else{ // succcess, it shut down properly
			remove(PID_FILE);
			return 1; 
		}
	}
	
	// otherwise force kill the program if the PID file never got cleaned up
	kill((pid_t)old_pid, SIGKILL);
	usleep(500000);

	// delete the old PID file if it was left over
	remove(PID_FILE);

	// return -1 indicating the program had to be killed
	return -1;
}


