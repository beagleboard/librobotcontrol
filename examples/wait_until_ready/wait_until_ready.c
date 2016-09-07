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

#define TIMEOUT_S 30

int is_cape_loaded();
int check_timeout();

uint64_t start_us;




/*******************************************************************************
* int main()
*
*******************************************************************************/
int main(){
	// log start time 
	start_us = micros_since_epoch();

	system("echo start > /root/startlog.txt");

	// check capemanager
	while(is_cape_loaded()<1){
		if(check_timeout()) return 1;
		usleep(500000);
	}

	while(initialize_cape()){
		if(check_timeout()) return 1;
		usleep(500000);
	}

	cleanup_cape();
	printf("success, cape ready\n");
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