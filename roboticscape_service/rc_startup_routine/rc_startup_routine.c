/*******************************************************************************
* rc_startup_routine.c
*
* James Strawson 2016
* Hardware drivers and the bone capemanager initialize in a rather unpredictable
* schedule. This program checks everything necessary for the cape library to run
* in order.
*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <error.h>
#include <unistd.h>
#include "../../libraries/roboticscape.h"
#include "../../libraries/rc_defs.h"
#include "../../libraries/gpio/rc_gpio_setup.h"
#include "../../libraries/other/rc_pru.h"

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
	rc_bb_model_t model;

	// ensure root privaleges until we sort out udev rules
	if(geteuid()!=0){
		fprintf(stderr,"ERROR: Robotics Cape library must be run as root\n");
		return -1;
	}

	// log start time 
	start_us = rc_nanos_since_epoch()/1000 ;
	system("echo start > " START_LOG);

	// stop a possibly running process and
	// delete old pid file if it's left over from an improper shudown
	rc_kill();
	
	// whitelist blue, black, and black wireless only when RC device tree is in use
	model = rc_get_bb_model();
	if(model!=BB_BLACK_RC && model!=BB_BLACK_W_RC && model!=BB_BLUE){
		if(system("grep -q roboticscape /boot/uEnv.txt")!=0){
			fprintf(stderr,"roboticscape service can only run on BB Blue, Black, and Black wireless when the roboticscape device tree is in use.\n");
			return -1;
		}
	}

	// export gpio pins
	while(configure_gpio_pins()!=0){
		if(check_timeout()){
			system("echo 'timeout reached while waiting for gpio driver' >> " START_LOG);
			printf("timeout reached while waiting for gpio driver\n");
		 	return -1;
		}
		rc_usleep(500000);
	}
	time = (rc_nanos_since_epoch()/1000-start_us)/1000000;
	sprintf(buf, "echo 'time (s): %4.1f GPIO loaded' >> %s",time,START_LOG);
	system(buf);


	// wait for eQEP to load
	while(check_eqep()!=0){
		if(check_timeout()){
			system("echo 'timeout reached while waiting for eQEP driver' >> " START_LOG);
			printf("timeout reached while waiting for eQEP driver\n");
		 	return -1;
		}
		rc_usleep(500000);
	}
	time = (rc_nanos_since_epoch()/1000-start_us)/1000000;
	sprintf(buf, "echo 'time (s): %4.1f eQEP loaded' >> %s",time,START_LOG);
	system(buf);


	// set up pwm at desired frequnecy
	while(setup_pwm()!=0){
		if(check_timeout()){
			system("echo 'timeout reached while waiting for pwm driver' >> " START_LOG);
			printf("timeout reached while waiting for pwm driver\n");
		 	return -1;
		}
		rc_usleep(500000);
	}
	time = (rc_nanos_since_epoch()/1000-start_us)/1000000;
	sprintf(buf, "echo 'time (s): %4.1f PWM loaded' >> %s",time,START_LOG);
	system(buf);

	//wait for pinmux
	while(rc_set_default_pinmux()!=0){
		if(check_timeout()){
			system("echo 'timeout reached while waiting for pinmux' >> " START_LOG);
			printf("timeout reached while waiting for pinmux\n");
		 	return 0;
		}
		rc_usleep(500000);
	}
	time = (rc_nanos_since_epoch()/1000-start_us)/1000000;
	sprintf(buf, "echo 'time (s): %4.1f pinmux driver loaded' >> %s",time,START_LOG);
	system(buf);

	//wait for pru
	while(restart_pru()!=0){
		if(check_timeout()){
			system("echo 'timeout reached while waiting for remoteproc pru' >> " START_LOG);
			printf("timeout reached while waiting for remoteproc pru\n");
		 	return 0;
		}
		rc_usleep(500000);
	}
	time = (rc_nanos_since_epoch()/1000-start_us)/1000000;
	sprintf(buf, "echo 'time (s): %4.1f PRU rproc loaded' >> %s",time,START_LOG);
	system(buf);

	printf("roboticscape startup routine complete\n");
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
	uint64_t new_us = rc_nanos_since_epoch()/1000;
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
	if(rc_pwm_init(1,DEFAULT_PWM_FREQ)) return -1;
	if(rc_pwm_init(2,DEFAULT_PWM_FREQ)) return -1;
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


