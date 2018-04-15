/**
 * @file rc_startup_routine.c
 *
 *
 * This startup routine is called by the roboticscape systemd service and serves
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
#define TIMEOUT_S 15
#define START_LOG "/var/log/roboticscape/startup_log.txt"

static int make_pid_directory();
static int set_gpio_permissions();
static int check_timeout();
static int setup_pwm();
static int check_eqep();

static uint64_t start_us;


int main()
{
	char buf[MAXBUF];
	float time;
	rc_model_t model;

	// ensure root privileges
	if(geteuid()!=0){
		fprintf(stderr,"ERROR: rc_startup_routine must be run as root\n");
		return -1;
	}

	// log start time
	start_us = rc_nanos_since_epoch()/1000 ;
	system("echo start > " START_LOG);

	// make pid directory with correct permissions
	make_pid_directory();

	// whitelist blue, black, and black wireless only when RC device tree is in use
	model = rc_model();
	if(model!=BB_BLACK_RC && model!=BB_BLACK_W_RC && model!=BB_BLUE){
		if(system("grep -q roboticscape /boot/uEnv.txt")!=0){
			fprintf(stderr,"roboticscape service can only run on BB Blue, Black, and Black wireless when the roboticscape device tree is in use.\n");
			return -1;
		}
	}

	// set permissions on gpio
	while(set_gpio_permissions()!=0){
		if(check_timeout()){
			system("echo 'timeout reached while waiting for gpio driver' >> " START_LOG);
			fprintf(stderr,"timeout reached while waiting for gpio driver\n");
			return -1;
		}
		rc_usleep(500000);
	}
	time = (rc_nanos_since_epoch()/1000-start_us)/1000000.0;
	sprintf(buf, "echo 'time (s): %4.2f GPIO loaded' >> %s",time,START_LOG);
	system(buf);


	// set up pwm at desired frequnecy
	while(setup_pwm()!=0){
		if(check_timeout()){
			system("echo 'timeout reached while waiting for pwm driver' >> " START_LOG);
			fprintf(stderr,"timeout reached while waiting for pwm driver\n");
			return -1;
		}
		rc_usleep(500000);
	}
	time = (rc_nanos_since_epoch()/1000-start_us)/1000000.0;
	sprintf(buf, "echo 'time (s): %4.2f PWM loaded' >> %s",time,START_LOG);
	system(buf);

	// wait for eQEP to load
	while(check_eqep()!=0){
		if(check_timeout()){
			system("echo 'timeout reached while waiting for eQEP driver' >> " START_LOG);
			fprintf(stderr,"timeout reached while waiting for eQEP driver\n");
		 	return -1;
		}
		rc_usleep(500000);
	}
	time = (rc_nanos_since_epoch()/1000-start_us)/1000000.0;
	sprintf(buf, "echo 'time (s): %4.2f eQEP loaded' >> %s",time,START_LOG);
	system(buf);

	set_gpio_permissions();

	printf("roboticscape startup routine complete\n");
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
	uint64_t new_us = rc_nanos_since_epoch()/1000;
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
int setup_pwm()
{
	// first make sure the driver is loaded, if so the export paths will appear
	if(access("/sys/class/pwm/pwmchip0/export",F_OK)!=0){
		// export directory not there yet, wait a bit more
		return -1;
	}
	if(access("/sys/class/pwm/pwmchip2/export",F_OK)!=0){
		// export directory not there yet, wait a bit more
		return -1;
	}
	if(access("/sys/class/pwm/pwmchip4/export",F_OK)!=0){
		// export directory not there yet, wait a bit more
		return -1;
	}

	// now export the channels if necessary
	if(access("/sys/class/pwm/pwmchip0/pwm0/enable",F_OK)!=0){
		system("/bin/echo 0 > /sys/class/pwm/pwmchip0/export");
	}
	if(access("/sys/class/pwm/pwmchip0/pwm1/enable",F_OK)!=0){
		system("/bin/echo 1 > /sys/class/pwm/pwmchip0/export");
	}
	if(access("/sys/class/pwm/pwmchip2/pwm0/enable",F_OK)!=0){
		system("/bin/echo 0 > /sys/class/pwm/pwmchip2/export");
	}
	if(access("/sys/class/pwm/pwmchip2/pwm1/enable",F_OK)!=0){
		system("/bin/echo 1 > /sys/class/pwm/pwmchip2/export");
	}
	if(access("/sys/class/pwm/pwmchip4/pwm0/enable",F_OK)!=0){
		system("/bin/echo 0 > /sys/class/pwm/pwmchip4/export");
	}
	if(access("/sys/class/pwm/pwmchip4/pwm1/enable",F_OK)!=0){
		system("/bin/echo 1 > /sys/class/pwm/pwmchip4/export");
	}

	// set permissions
	system("/bin/chown -R root:pwm /sys/class/pwm/pwmchip*/");
	system("/bin/chmod -R ug+rw /sys/class/pwm/pwmchip*/");

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
 * @brief      Makes a directory to put PID file in which has universal write
 *             access so non-root users can still use PID files in /var/run.
 *
 *             /var/run/user doesn't work since user may run some programs as
 *             root and others as debian user.
 *
 * @return     return 0 on success, -1 on failure
 */
int make_pid_directory()
{
	int ret;
	// these permissions are not the final permissions of the directory as
	// mkdir will mask it with umask on creation
	ret = mkdir(RC_PID_DIR, 0777);
	// error check, EEXIST is okay, we want directory to exist!
	if(ret==-1 && errno!=EEXIST){
		perror("ERROR making PID directory");
		return -1;
	}

	// now set the correct permissions
	ret = chmod(RC_PID_DIR, 0777);
	if(ret==-1){
		perror("ERROR setting permissions of PID directory");
		return -1;
	}
	return 0;
}