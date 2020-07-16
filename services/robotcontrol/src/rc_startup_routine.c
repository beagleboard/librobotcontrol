/**
 * @file rc_startup_routine.c
 *
 *
 * This startup routine is called by the robotcontrol systemd service and serves
 * to do some initial setup such as setting permissions on things that udev cant
 * handle such as pwm and gpio
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h> // for mkdir and chmod
#include <sys/types.h> // for mkdir and chmod


#include <rc/model.h>
#include <rc/time.h>
#include <rc/start_stop.h>

#define MAXBUF 128
#define TIMEOUT_S 30
#define START_LOG "/var/log/robotcontrol/startup_log.txt"

static int set_gpio_permissions();
static int check_timeout();
static int check_pwm();
static int check_eqep();
static int check_pru();

static uint64_t start_us;


int main()
{
	char buf[MAXBUF];
	double time;
	rc_model_t model;

	// ensure root privileges
	if(geteuid()!=0){
		fprintf(stderr,"ERROR: rc_startup_routine must be run as root\n");
		return -1;
	}

	// capture start time for check_timeout
	start_us = rc_nanos_since_boot()/1000;

	// log start time
	time = rc_nanos_since_boot()/1000000000.0;
	sprintf(buf, "echo 'start time (s): %6.2f' > %s",time,START_LOG);
	system(buf);

	// whitelist blue, black, and black wireless only when RC device tree is in use
	model = rc_model();
	if(model!=MODEL_BB_BLACK_RC && model!=MODEL_BB_BLACK_W_RC && model!=MODEL_BB_BLUE){
		if(system("grep -q robotcontrol /boot/uEnv.txt")!=0){
			fprintf(stderr,"robotcontrol service can only run on BB Blue, Black, and Black wireless when the robotcontrol device tree is in use.\n");
			return 0;
		}
	}

	// set permissions on gpio, not strictly necessary anymore with udev rules
	// but still here for older systems and since it also checks to make sure
	// gpio chips are loaded
	while(set_gpio_permissions()!=0){
		if(check_timeout()){
			system("echo 'timeout reached while waiting for gpio driver' >> " START_LOG);
			fprintf(stderr,"timeout reached while waiting for gpio driver\n");
			return 0;
		}
		rc_usleep(500000);
	}
	time = rc_nanos_since_boot()/1000000000.0;
	sprintf(buf, "echo 'time (s): %4.2f GPIO loaded' >> %s",time,START_LOG);
	system(buf);


	// set up pwm at desired frequnecy
	while(check_pwm()!=0){
		if(check_timeout()){
			system("echo 'timeout reached while waiting for pwm driver' >> " START_LOG);
			fprintf(stderr,"timeout reached while waiting for pwm driver\n");
			return 0;
		}
		rc_usleep(500000);
	}
	time = rc_nanos_since_boot()/1000000000.0;
	sprintf(buf, "echo 'time (s): %4.2f PWM loaded' >> %s",time,START_LOG);
	system(buf);

	// wait for eQEP to load
	while(check_eqep()!=0){
		if(check_timeout()){
			system("echo 'timeout reached while waiting for eQEP driver' >> " START_LOG);
			fprintf(stderr,"timeout reached while waiting for eQEP driver\n");
		 	return 0;
		}
		rc_usleep(500000);
	}
	time = rc_nanos_since_boot()/1000000000.0;
	sprintf(buf, "echo 'time (s): %4.2f eQEP loaded' >> %s",time,START_LOG);
	system(buf);

	// wait for PRU to load
	while(check_pru()!=0){
		if(check_timeout()){
			system("echo 'timeout reached while waiting for PRU remoteproc driver' >> " START_LOG);
			fprintf(stderr,"timeout reached while waiting for PRU remoteproc driver\n");
			return -1;
		}
		rc_usleep(500000);
	}
	time = rc_nanos_since_boot()/1000000000.0;
	sprintf(buf, "echo 'time (s): %4.2f PRU loaded' >> %s",time,START_LOG);
	system(buf);

	printf("robotcontrol startup routine complete\n");
	system("echo 'startup routine complete' >> " START_LOG);
	return 0;
}


int set_gpio_permissions(){
	int ret;
	// check all gpio devices are online
	if(access("/dev/gpiochip3",F_OK)!=0){
		// not there yet, wait a bit more
		return -1;
	}
	if(access("/dev/gpiochip2",F_OK)!=0){
		// not there yet, wait a bit more
		return -1;
	}
	if(access("/dev/gpiochip1",F_OK)!=0){
		// not there yet, wait a bit more
		return -1;
	}
	if(access("/dev/gpiochip0",F_OK)!=0){
		// not there yet, wait a bit more
		return -1;
	}
	ret=system("/bin/chown -R root:gpio /dev/gpiochip* ");
	if(ret){
		fprintf(stderr,"ERROR setting gpiochip owner, returned %d\n",ret);
	}
	ret=system("/bin/chmod -R ug+rw /dev/gpiochip* ");
	if(ret){
		fprintf(stderr,"ERROR setting gpiochip permission, returned %d\n",ret);
	}
	return 0;
}

/**
 * @brief      looks and the current time to decide if the timeout has been reached.
 *
 * @return      returns 1 if timeout has been reached, otherwise 0
 */
int check_timeout()
{
	uint64_t new_us = rc_nanos_since_boot()/1000;
	int seconds = (new_us-start_us)/1000000;
	if(seconds>TIMEOUT_S){
		printf("TIMEOUT REACHED\n");
		system("echo 'TIMEOUT_REACHED' >> " START_LOG);
		return 1;
	}
	return 0;
}


/**
 * @brief      udev rules is unable to set the permissions correctly for pwm
 *             since it creates a directory at runtime/ solution here is to
 *             create the directory early at boot and set permissions.
 *
 * @return     returns 0 on success or -1 on failure
 */
int check_pwm()
{
	// first make sure the driver is loaded, if so the export paths will appear
	if(access("/sys/devices/platform/ocp/48300000.epwmss/48300200.pwm/",F_OK)!=0){
		return -1;
	}
	if(access("/sys/devices/platform/ocp/48302000.epwmss/48302200.pwm/",F_OK)!=0){
		return -1;
	}
	if(access("/sys/devices/platform/ocp/48304000.epwmss/48304200.pwm/",F_OK)!=0){
		return -1;
	}

	return 0;
}


/**
 * @brief      just check if eqep driver is loaded
 *
 * @return     0 if loaded, otherwise -1
 */
int check_eqep()
{
	if(access("/sys/devices/platform/ocp/48300000.epwmss/48300180.eqep/enabled", F_OK)){
		fprintf(stderr,"missing eqep0\n");
		return -1;
	}
	if(access("/sys/devices/platform/ocp/48302000.epwmss/48302180.eqep/enabled", F_OK)){
		fprintf(stderr,"missing eqep0\n");
		return -1;
	}
	if(access("/sys/devices/platform/ocp/48304000.epwmss/48304180.eqep/enabled", F_OK)){
		fprintf(stderr,"missing eqep0\n");
		return -1;
	}
	return 0;
}


/**
 * @brief      just check if pru remoteproc is loaded
 *
 * @return     0 if loaded, otherwise -1
 */
int check_pru()
{
	if(access("/dev/remoteproc/pruss-core0", F_OK)){
		return -1;
	}
	return 0;
}
