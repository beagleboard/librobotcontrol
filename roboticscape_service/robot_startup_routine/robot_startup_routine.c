/*******************************************************************************
* wait_until_ready.c
*
* James Strawson 2016
* Hardware drivers and the bone capemanager initialize in a rather unpredictable
* schedule. This program checks everything necessary for the cape library to run
* in order.
*******************************************************************************/

#include "../../libraries/roboticscape-usefulincludes.h"
#include "../../libraries/roboticscape.h"
#include "../../libraries/roboticscape-defs.h"
#include "../../libraries/simple_gpio/gpio_setup.h"
#include "../../libraries/other/robotics_pru.h"

#define TIMEOUT_S 5
#define START_LOG "/var/log/roboticscape/startup_log.txt"

int is_cape_loaded();
int check_timeout();
int setup_gpio();
int setup_pwm();
int check_eqep();

uint64_t start_us;


/*******************************************************************************
* int main()
*
*******************************************************************************/
int main(){
	char buf[128];
	float time;

	// log start time 
	start_us = micros_since_epoch();
	system("echo start > " START_LOG);

	// stop a possibly running process and
	// delete old pid file if it's left over from an improper shudown
	kill_robot();

	// export gpio pins
	while(configure_gpio_pins()!=0){
		if(check_timeout()){
			system("echo 'timeout reached while waiting for gpio driver' >> " START_LOG);
			printf("timeout reached while waiting for gpio driver\n");
		 	return -1;
		}
		usleep(500000);
	}
	time = (micros_since_epoch()-start_us)/1000000;
	sprintf(buf, "echo 'time (s): %4.1f GPIO loaded' >> %s",time,START_LOG);
	system(buf);


	// wait for eQEP to load
	while(check_eqep()!=0){
		if(check_timeout()){
			system("echo 'timeout reached while waiting for eQEP driver' >> " START_LOG);
			printf("timeout reached while waiting for eQEP driver\n");
		 	return -1;
		}
		usleep(500000);
	}
	time = (micros_since_epoch()-start_us)/1000000;
	sprintf(buf, "echo 'time (s): %4.1f eQEP loaded' >> %s",time,START_LOG);
	system(buf);


	// set up pwm at desired frequnecy
	while(setup_pwm()!=0){
		if(check_timeout()){
			system("echo 'timeout reached while waiting for pwm driver' >> " START_LOG);
			printf("timeout reached while waiting for pwm driver\n");
		 	return -1;
		}
		usleep(500000);
	}
	time = (micros_since_epoch()-start_us)/1000000;
	sprintf(buf, "echo 'time (s): %4.1f PWM loaded' >> %s",time,START_LOG);
	system(buf);


	//wait for pru
	while(restart_pru()!=0){
		if(check_timeout()){
			system("echo 'timeout reached while waiting for remoteproc pru' >> " START_LOG);
			printf("timeout reached while waiting for remoteproc pru\n");
		 	return 0;
		}
		usleep(500000);
	}
	time = (micros_since_epoch()-start_us)/1000000;
	sprintf(buf, "echo 'time (s): %4.1f PRU rproc loaded' >> %s",time,START_LOG);
	system(buf);


	//wait for pinmux
	while(set_default_pinmux()!=0){
		if(check_timeout()){
			system("echo 'timeout reached while waiting for pinmux' >> " START_LOG);
			printf("timeout reached while waiting for pinmux\n");
		 	return 0;
		}
		usleep(500000);
	}
	time = (micros_since_epoch()-start_us)/1000000;
	sprintf(buf, "echo 'time (s): %4.1f pinmux driver loaded' >> %s",time,START_LOG);
	system(buf);

	printf("startup routine complete\n");
	system("echo 'startup routine complete' >> " START_LOG);
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
		system("echo 'TIMEOUT_REACHED' >> " START_LOG);
		return 1;
	}
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


/*******************************************************************************
* int check_eqep()
*
* checks if eqep is loaded
*******************************************************************************/
int check_eqep(){
	if(access("/sys/devices/platform/ocp/48300000.epwmss/48300180.eqep/enabled", F_OK)) return -1;
	if(access("/sys/devices/platform/ocp/48302000.epwmss/48302180.eqep/enabled", F_OK)) return -1;
	if(access("/sys/devices/platform/ocp/48304000.epwmss/48304180.eqep/enabled", F_OK)) return -1;
	return 0;
}


