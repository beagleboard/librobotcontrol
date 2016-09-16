/*******************************************************************************
* wait_until_ready.c
*
* James Strawson 2016
* Hardware drivers and the bone capemanager initialize in a rather unpredictable
* schedule. This program checks everything necessary for the cape library to run
* in order.
*******************************************************************************/

#include <useful_includes.h>
#include <robotics_cape.h>
#include <robotics_cape_defs.h>
#include "../../libraries/simple_gpio/simple_gpio.h"
#include "../../libraries/simple_pwm/simple_pwm.h"

#define TIMEOUT_S 30
#define START_LOG "/etc/robotics/startup_log.txt"

int is_cape_loaded();
int check_timeout();
int setup_gpio();
int setup_pwm();

uint64_t start_us;




/*******************************************************************************
* int main()
*
*******************************************************************************/
int main(){

	// log start time 
	start_us = micros_since_epoch();
	system("echo start > " START_LOG);

	// delete old pid file if it's left over from an improper shudown
	if(remove(PID_FILE)==0) system("echo 'removed old PID_FILE' > " START_LOG);

	// check capemanager
	while(is_cape_loaded()!=1){
		if(check_timeout()) return 1;
		usleep(500000);
		system("echo 'waiting for overlay' >> " START_LOG);
	}
	system("echo 'cape overlay loaded' >> " START_LOG);

	// export gpio pins
	while(setup_gpio()!=0){
		if(check_timeout()) return 1;
		usleep(500000);
		system("echo 'waiting for gpio' >> " START_LOG);
	}
	system("echo 'gpio pins exported' >> " START_LOG);

	// set up pwm at desired frequnecy
	while(setup_pwm()!=0){
		if(check_timeout()) return 1;
		usleep(500000);
		system("echo 'waiting for pwm' >> " START_LOG);
	}
	system("echo 'pwm initialized' >> " START_LOG);

	cleanup_cape();
	printf("success, cape ready\n");
	system("echo 'success, cape ready' >> " START_LOG);
	return 0;
}

/*******************************************************************************
* is_cape_loaded()
*
* check to make sure robotics cape overlay is loaded
* return 1 if cape is loaded
* return 0 if cape is missing
*******************************************************************************/
int is_cape_loaded(){
	int ret = system("grep -q "CAPE_NAME" /sys/devices/platform/bone_capemgr*/slots");
	if(ret == 0) return 1;
	return 0;
}

/*******************************************************************************
* int check_timeout()
*
* looks and the current time to decide if the timeout has been reached.
* returns 1 if timeout has been reached.
*******************************************************************************/
int check_timeout(){
	uint64_t new_us = micros_since_epoch();
	int seconds = (new_us-start_us)/1000000;
	if(seconds>TIMEOUT_S){
		printf("TIMEOUT REACHED\n");
		return 1;
	}
	return 0;
}

/*******************************************************************************
* int setup_gpio()
*
* exports and sets the direction of each gpio pin
*******************************************************************************/
int setup_gpio(){
	
	//export all GPIO output pins
	if(gpio_export(RED_LED)) return -1;
	if(gpio_set_dir(RED_LED, OUTPUT_PIN)) return -1;
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
	gpio_export(SERVO_PWR);
	gpio_set_dir(SERVO_PWR, OUTPUT_PIN);

	//set up mode pin
	gpio_export(MODE_BTN);
	gpio_set_dir(MODE_BTN, INPUT_PIN);
	gpio_set_edge(MODE_BTN, "both");  // Can be rising, falling or both
	
	//set up pause pin
	gpio_export(PAUSE_BTN);
	gpio_set_dir(PAUSE_BTN, INPUT_PIN);
	gpio_set_edge(PAUSE_BTN, "both");  // Can be rising, falling or both

	// set up IMU interrupt pin
	//set up gpio interrupt pin connected to imu
	gpio_export(IMU_INTERRUPT_PIN);
	gpio_set_dir(IMU_INTERRUPT_PIN, INPUT_PIN);
	gpio_set_edge(IMU_INTERRUPT_PIN, "falling");

	return 0;
}


/*******************************************************************************
* int setup_pwm()
*
* exports and sets the direction of each gpio pin
*******************************************************************************/
int setup_pwm(){
	if(simple_init_pwm(1,PWM_FREQ)) return -1;
	if(simple_init_pwm(2,PWM_FREQ)) return -1;
	return 0;
}