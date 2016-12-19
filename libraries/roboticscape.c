/*******************************************************************************
* robticscape.c
* 
* This is one of many c-files that are used to build libroboticscape.so
* however it contains the majority of the core components.
*******************************************************************************/

#include "roboticscape-usefulincludes.h"
#include "roboticscape.h"
#include "roboticscape-defs.h"
#include "simple_gpio/gpio_setup.h"
#include "mmap/mmap_gpio_adc.h"		// used for fast gpio functions
#include "mmap/mmap_pwmss.h"		// used for fast pwm functions
#include "other/robotics_pru.h"
#include "other/roboticscape_buttons.h"
#include "other/roboticscape_motors.h"

#define CAPE_NAME	"RoboticsCape"
#define MAX_BUF		512

/*******************************************************************************
* Global Variables
*******************************************************************************/
// global roboticscape state
enum rc_state_t rc_state = UNINITIALIZED;



/*******************************************************************************
* local function declarations
*******************************************************************************/
int is_cape_loaded();
void shutdown_signal_handler(int signo);


/*******************************************************************************
* int initialize_roboticscape()
* sets up necessary hardware and software
* should be the first thing your program calls
*******************************************************************************/
int initialize_roboticscape(){
	FILE *fd; 

	// check if another project was using resources
	// kill that process cleanly with sigint if so
	#ifdef DEBUG
		printf("checking for existing PID_FILE\n");
	#endif
	kill_robot();

	// start state as Uninitialized
	rc_set_state(UNINITIALIZED);
	
	// Start Signal Handler
	#ifdef DEBUG
	printf("Initializing exit signal handler\n");
	#endif
	enable_rc_sig_handler();


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

	// eQep encoder counters
	#ifdef DEBUG
	printf("Initializing: eQEP\n");
	#endif
	// this also zero's out the encoder counters
	if(init_eqep(0)){
		printf("WARNING: failed to initialize eQEP0\n");
	}
	if(init_eqep(1)){
		printf("WARNING: failed to initialize eQEP1\n");
	}
	if(init_eqep(2)){
		printf("WARNING: failed to initialize eQEP2\n");
	}

	// motors
	#ifdef DEBUG
	printf("Initializing: Motors\n");
	#endif
	if(initialize_motors()){
		printf("WARNING: Failed to initialize motors\n");
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

	return 0;
}

/*******************************************************************************
*	int cleanup_roboticscape()
*	shuts down library and hardware functions cleanly
*	you should call this before your main() function returns
*******************************************************************************/
int cleanup_roboticscape(){
	// just in case the user forgot, set state to exiting
	rc_set_state(EXITING);

	// announce we are starting cleanup process
	printf("\nExiting Cleanly\n");
	
	#ifdef DEBUG
	printf("waiting for button handlers to join\n");
	#endif
	wait_for_button_handlers_to_join();

	#ifdef DEBUG
	printf("turning off GPIOs & PWM\n");
	#endif
	rc_set_led(GREEN,LOW);
	rc_set_led(RED,LOW);

	#ifdef DEBUG
	printf("Turning off motors\n");
	#endif
	disable_motors();

	#ifdef DEBUG
	printf("Turning off SPI slaves\n");
	#endif
	manual_deselect_spi_slave(1);
	manual_deselect_spi_slave(2);

	#ifdef DEBUG
	printf("Turning off servo power rail\n");
	#endif
	disable_servo_power_rail();
	
	#ifdef DEBUG
	printf("Stopping dsm service\n");
	#endif
	stop_dsm_service();	
	
	#ifdef DEBUG
	printf("Deleting PID file\n");
	#endif
	FILE* fd;
	// clean up the pid_file if it still exists
	fd = fopen(PID_FILE, "r");
	if (fd != NULL) {
		// close and delete the old file
		fclose(fd);
		remove(PID_FILE);
	}

	#ifdef DEBUG
	printf("end of cleanup_cape\n");
	#endif
	return 0;
}


/*******************************************************************************
* @ rc_state_t rc_get_state()
*
* returns the high-level robot state variable
* use this for managing how your threads start and stop
*******************************************************************************/
rc_state_t rc_get_state(){
	return rc_state;
}


/*******************************************************************************
* @ int rc_set_state(rc_state_t new_state)
*
* sets the high-level robot state variable
* use this for managing how your threads start and stop
*******************************************************************************/
int rc_set_state(rc_state_t new_state){
	rc_state = new_state;
	return 0;
}

/*******************************************************************************
* @ int rc_print_state()
* 
* Prints the textual name of the state to the current state to the screen.
*******************************************************************************/
int rc_print_state(){
	switch(rc_state){
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
* @ int rc_set_led(rc_led_t led, int state)
* 
* turn on or off the green or red LED on robotics cape
* if state is 0, turn led off, otherwise on.
* we suggest using the names HIGH or LOW
*******************************************************************************/
int rc_set_led(rc_led_t led, int state){
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
* int rc_get_led(rc_led_t led)
* 
* returns the state of the green or red LED on robotics cape
* state is LOW(0), or HIGH(1)
*******************************************************************************/
int rc_get_led(rc_led_t led){
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
* rc_blink_led(rc_led_t led, double hz, double period)
*	
* Flash an LED at a set frequency for a finite period of time.
* This is a blocking call and only returns after flashing.
*******************************************************************************/
int rc_blink_led(rc_led_t led, double hz, double period){
	const int delay_us = 1000000.0/(2.0*hz); 
	const int blinks = period*2.0*hz;
	int i;
	int toggle = 0;
	
	for(i=0;i<blinks;i++){
		toggle = !toggle;
		if(rc_get_state()==EXITING) break;
		rc_set_led(led,toggle);
		// wait for next blink
		usleep(delay_us);
	}
	
	rc_set_led(led, 0); // make sure it is left off
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
* double get_battery_voltage()
* 
* returns the LiPo battery voltage on the robotics cape
* this accounts for the voltage divider ont he cape
*******************************************************************************/
double get_battery_voltage(){
	double v = (get_adc_volt(LIPO_ADC_CH)*V_DIV_RATIO)+LIPO_OFFSET; 
	if(v<0.3) v = 0.0;
	return v;
}

/*******************************************************************************
* double get_dc_jack_voltage()
* 
* returns the DC power jack voltage on the robotics cape
* this accounts for the voltage divider ont he cape
*******************************************************************************/
double get_dc_jack_voltage(){
	double v = (get_adc_volt(DC_JACK_ADC_CH)*V_DIV_RATIO)+DC_JACK_OFFSET; 
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
* double get_adc_volt(int ch)
* 
* returns an actual voltage for an adc channel
*******************************************************************************/
double get_adc_volt(int ch){
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
* all threads should watch for rc_get_state()==EXITING and shut down cleanly
*******************************************************************************/
void shutdown_signal_handler(int signo){
	if (signo == SIGINT){
		rc_set_state(EXITING);
		printf("\nreceived SIGINT Ctrl-C\n");
 	}else if (signo == SIGTERM){
		rc_set_state(EXITING);
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

/*******************************************************************************
* @ void disable_rc_sig_handler(
*
* Disables the built-in signal handler. Use only if you want to implement your
* own signal handler. Make sure your handler sets rc_state to EXITING or calls
* cleanup_cape on shutdown to ensure roboticscape library threads close
* cleanly.
*******************************************************************************/
void disable_rc_sig_handler(){
	signal(SIGINT, SIG_DFL);
	signal(SIGKILL, SIG_DFL);
	return;
}

/*******************************************************************************
* @ void enable_rc_sig_handler(
*
* enables the built-in signal handler if it was disabled before. The built-in 
* signal handler is enabled in initialize_roboticscape()
*******************************************************************************/
void enable_rc_sig_handler(){
	signal(SIGINT, shutdown_signal_handler);
	signal(SIGTERM, shutdown_signal_handler);
	return;
}

