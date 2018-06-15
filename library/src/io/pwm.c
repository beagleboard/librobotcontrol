/**
 * @file rc_pwm.c
 *
 * C pwm interface for Beaglebone boards
 */


#include <stdio.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <rc/pwm.h>
#include <rc/time.h>

#define MIN_HZ 1
#define MAX_HZ 1000000000
#define MAXBUF 64
#define BASE_DIR "/sys/class/pwm/pwmchip"

// preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)

// variables
static int dutyA_fd[3];			// pointers to duty cycle file descriptor
static int dutyB_fd[3];			// pointers to duty cycle file descriptor
static unsigned int period_ns[3];	// one period per subsystem
static int init_flag[3] = {0,0,0};
static int mode; // 0 for "pwmx", 1 for "pwmx:y" versions of driver

/**
 * @brief      exports A and B pwm channels
 *
 * @param[in]  ss    subsystem, to export
 *
 * @return     0 on succcess, -1 on failure
 */
int __export_channels(int ss)
{
	int export_fd=0;
	char buf[MAXBUF];
	int len;

	// open export file for that subsystem
	len = snprintf(buf, sizeof(buf), BASE_DIR "%d/export", ss*2);
	export_fd = open(buf, O_WRONLY);
	if(unlikely(export_fd<0)){
		perror("ERROR in rc_pwm_init, can't open pwm export file for writing");
		fprintf(stderr,"Probably kernel or BeagleBone image is too old\n");
		return -1;
	}
	len=write(export_fd, "0", 2);
	if(unlikely(len<0 && errno!=EBUSY)){
		perror("ERROR: in rc_pwm_init, failed to write 0 to export file");
		return -1;
	}
	len=write(export_fd, "1", 2);
	if(unlikely(len<0 && errno!=EBUSY)){
		perror("ERROR: in rc_pwm_init, failed to write 1 to export file");
		return -1;
	}
	close(export_fd);

	// determine mode
	// start with channel A and also check both versions of driver
	len = snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm0/enable", ss*2); // mode 0
	// if it exists, mode is 0
	if(access(buf,F_OK)==0) mode=0;
	else{
		len = snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm-%d:0/enable", ss*2, ss*2); // mode 1
		// if it exists, mode is 1
		if(access(buf,F_OK)==0) mode=1;
		else{
			fprintf(stderr, "ERROR in rc_pwm_init, export failed for subsystem %d channel %d\n", ss, 0);
			return -1;
		}
	}

	// check channel B
	if(mode==0)	len = snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm1/enable", ss*2); // mode 0
	else		len = snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm-%d:1/enable", ss*2, ss*2); // mode 1
	// if it exists, mode is 0
	if(access(buf,F_OK)!=0){
		fprintf(stderr, "ERROR in rc_pwm_init, export failed for subsystem %d channel %d\n", ss, 0);
		return -1;
	}
	#ifdef DEBUG
	printf("pwm ss:%d mode:%d\n",ss,mode);
	#endif
	return 0;
}

/**
 * @brief      unexports A and B pwm channels
 *
 * @param[in]  ss    subsystem, to export
 *
 * @return     0 on succcess, -1 on failure
 */
int __unexport_channels(int ss)
{
	int unexport_fd=0;
	char buf[MAXBUF];
	int len;

	// open export file for that subsystem
	len = snprintf(buf, sizeof(buf), BASE_DIR "%d/unexport", ss*2);
	unexport_fd = open(buf, O_WRONLY);
	if(unlikely(unexport_fd<0)){
		perror("ERROR in rc_pwm_init, can't open pwm unexport file for writing");
		fprintf(stderr,"Probably kernel or BeagleBone image is too old\n");
		return -1;
	}
	len=write(unexport_fd, "0", 2);
	if(unlikely(len<0 && errno!=EBUSY && errno!=ENODEV)){
		perror("ERROR: in rc_pwm_init, failed to write 0 to unexport file");
		return -1;
	}
	len=write(unexport_fd, "1", 2);
	if(unlikely(len<0 && errno!=EBUSY  && errno!=ENODEV)){
		perror("ERROR: in rc_pwm_init, failed to write 1 to unexport file");
		return -1;
	}
	close(unexport_fd);
	return 0;
}


int rc_pwm_init(int ss, int frequency)
{
	int periodA_fd; // pointers to frequency file descriptor
	int periodB_fd;
	int enableA_fd;  // run (enable) file pointers
	int enableB_fd;
	int polarityA_fd;
	int polarityB_fd;
	char buf[MAXBUF];
	int len;

	// sanity checks
	if(ss<0 || ss>2){
		fprintf(stderr,"ERROR in rc_pwm_init, PWM subsystem must be 0 1 or 2\n");
		return -1;
	}
	if(frequency<MIN_HZ || frequency>MAX_HZ){
		fprintf(stderr,"ERROR in rc_pwm_init, frequency must be between %dHz and %dHz\n", MIN_HZ, MAX_HZ);
		return -1;
	}

	// unexport then export channels first
	if(__unexport_channels(ss)==-1) return -1;
	if(__export_channels(ss)==-1) return -1;

	// wait for udev to set correct permissions

	//system("ls -la /sys/class/pwm/pwmchip4/pwm-4:0/");
	//system("udevadm trigger");
	//rc_usleep(1000000);
	//system("ls -la /sys/class/pwm/pwmchip4/pwm-4:0/");

	#ifdef DEBUG
	printf("pwm ss:%d mode:%d\n",ss,mode);
	#endif

	// open file descriptors for duty cycles
	if(mode==0)	len = snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm0/duty_cycle", ss*2); // mode 0
	else		len = snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm-%d:0/duty_cycle", ss*2, ss*2); // mode 1
	dutyA_fd[ss] = open(buf,O_WRONLY);

	if(unlikely(dutyA_fd[ss]==-1)){
		// first error is probably from udev being slow, wait a bit and try again
		rc_usleep(600000);
		dutyA_fd[ss] = open(buf,O_WRONLY);
		if(unlikely(dutyA_fd[ss]==-1)){
			perror("ERROR in rc_pwm_init, failed to open duty_cycle channel A FD");
			fprintf(stderr,"tried accessing: %s\n", buf);
			return -1;
		}
	}

	if(mode==0)	len = snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm1/duty_cycle", ss*2); // mode 0
	else		len = snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm-%d:1/duty_cycle", ss*2, ss*2); // mode 1
	dutyB_fd[ss] = open(buf,O_WRONLY);
	if(unlikely(dutyB_fd[ss]==-1)){
		perror("ERROR in rc_pwm_init, failed to open duty_cycle channel B FD");
		fprintf(stderr,"tried accessing: %s\n", buf);
		return -1;
	}

	// now open enable, polarity, and period FDs for setup
	if(mode==0)	len = snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm0/enable", ss*2); // mode 0
	else		len = snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm-%d:0/enable", ss*2, ss*2); // mode 1
	enableA_fd = open(buf,O_WRONLY);
	if(unlikely(enableA_fd==-1)){
		perror("ERROR in rc_pwm_init, failed to open pwm A enable fd");
		fprintf(stderr,"tried accessing: %s\n", buf);
		return -1;
	}
	if(mode==0)	len = snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm1/enable", ss*2); // mode 0
	else		len = snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm-%d:1/enable", ss*2, ss*2); // mode 1
	enableB_fd = open(buf,O_WRONLY);
	if(unlikely(enableB_fd==-1)){
		perror("ERROR in rc_pwm_init, failed to open pwm B enable fd");
		fprintf(stderr,"tried accessing: %s\n", buf);
		return -1;
	}
	if(mode==0)	len = snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm0/period", ss*2); // mode 0
	else		len = snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm-%d:0/period", ss*2, ss*2); // mode 1
	periodA_fd = open(buf,O_WRONLY);
	if(unlikely(periodA_fd==-1)){
		perror("ERROR in rc_pwm_init, failed to open pwm A period fd");
		fprintf(stderr,"tried accessing: %s\n", buf);
		return -1;
	}
	if(mode==0)	len = snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm1/period", ss*2); // mode 0
	else		len = snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm-%d:1/period", ss*2, ss*2); // mode 1
	periodB_fd = open(buf,O_WRONLY);
	if(unlikely(periodB_fd==-1)){
		perror("ERROR in rc_pwm_init, failed to open pwm B period fd");
		fprintf(stderr,"tried accessing: %s\n", buf);
		return -1;
	}
	if(mode==0)	len = snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm0/polarity", ss*2); // mode 0
	else		len = snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm-%d:0/polarity", ss*2, ss*2); // mode 1
	polarityA_fd = open(buf,O_WRONLY);
	if(unlikely(polarityA_fd==-1)){
		perror("ERROR in rc_pwm_init, failed to open pwm A polarity fd");
		fprintf(stderr,"tried accessing: %s\n", buf);
		return -1;
	}
	if(mode==0)	len = snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm1/polarity", ss*2); // mode 0
	else		len = snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm-%d:1/polarity", ss*2, ss*2); // mode 1
	polarityB_fd = open(buf,O_WRONLY);
	if(unlikely(polarityB_fd==-1)){
		perror("ERROR in rc_pwm_init, failed to open pwm B polarity fd");
		fprintf(stderr,"tried accessing: %s\n", buf);
		return -1;
	}

	// set the period in nanoseconds
	period_ns[ss] = 1000000000/frequency;
	len = snprintf(buf, sizeof(buf), "%d", period_ns[ss]);
	if(unlikely(write(periodA_fd, buf, len)==-1)){
		perror("ERROR in rc_pwm_init, failed to write to channel A period fd");
		return -1;
	}
	if(unlikely(write(periodB_fd, buf, len)==-1)){
		perror("ERROR in rc_pwm_init, failed to write to channel B period fd");
		return -1;
	}

	// set polarity to 0 (normal)
	if(unlikely(write(polarityA_fd, "normal", 7)==-1)){
		perror("ERROR in rc_pwm_init, failed to write to channel A polarity fd");
		return -1;
	}
	if(unlikely(write(polarityB_fd, "normal", 7)==-1)){
		perror("ERROR in rc_pwm_init, failed to write to channel B polarity fd");
		return -1;
	}

	// make sure duty is 0
	if(unlikely(write(dutyA_fd[ss], "0", 2)==-1)){
		perror("ERROR in rc_pwm_init, failed to write to channel A duty fd");
		return -1;
	}
	if(unlikely(write(dutyB_fd[ss], "0", 2)==-1)){
		perror("ERROR in rc_pwm_init, failed to write to channel B duty fd");
		return -1;
	}

	// now enable both channels
	if(unlikely(write(enableA_fd, "1", 2)==-1)){
		perror("ERROR in rc_pwm_init, failed to write to channel A enable fd");
		return -1;
	}
	if(unlikely(write(enableB_fd, "1", 2)==-1)){
		perror("ERROR in rc_pwm_init, failed to write to channel B enable fd");
		return -1;
	}

	// close all the files
	close(enableA_fd);
	close(enableB_fd);
	close(periodA_fd);
	close(periodB_fd);
	close(polarityA_fd);
	close(polarityB_fd);

	// everything successful
	init_flag[ss] = 1;
	return 0;
}

int rc_pwm_cleanup(int ss)
{
	int enableA_fd;
	int enableB_fd;
	char buf[MAXBUF];

	// sanity check
	if(unlikely(ss<0 || ss>2)){
		fprintf(stderr,"ERROR in rc_pwm_close, subsystem must be between 0 and 2\n");
		return -1;
	}
	if(init_flag[ss]==0){
		return 0;
	}

	// now open enable FDs
	if(mode==0)	snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm0/enable", ss*2); // mode 0
	else		snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm-%d:0/enable", ss*2, ss*2); // mode 1
	enableA_fd = open(buf,O_WRONLY);
	if(unlikely(enableA_fd==-1)){
		perror("ERROR in rc_pwm_cleanup, failed to open pwm A enable fd");
		return -1;
	}
	if(mode==0)	snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm1/enable", ss*2); // mode 0
	else		snprintf(buf, sizeof(buf), BASE_DIR "%d/pwm-%d:1/enable", ss*2, ss*2); // mode 1
	enableB_fd = open(buf,O_WRONLY);
	if(unlikely(enableB_fd==-1)){
		perror("ERROR in rc_pwm_cleanup, failed to open pwm B enable fd");
		return -1;
	}

	// now make sure duty is 0
	if(unlikely(write(dutyA_fd[ss], "0", 2)==-1)){
		perror("ERROR in rc_pwm_init, failed to write to channel A duty fd");
		return -1;
	}
	if(unlikely(write(dutyB_fd[ss], "0", 2)==-1)){
		perror("ERROR in rc_pwm_init, failed to write to channel B duty fd");
		return -1;
	}

	// disable channels
	if(unlikely(write(enableA_fd, "0", 2)==-1)){
		perror("ERROR in rc_pwm_init, failed to write to channel A enable fd");
		return -1;
	}
	if(unlikely(write(enableB_fd, "0", 2)==-1)){
		perror("ERROR in rc_pwm_init, failed to write to channel A enable fd");
		return -1;
	}

	// close fds
	close(enableA_fd);
	close(enableB_fd);
	close(dutyA_fd[ss]);
	close(dutyB_fd[ss]);

	// unexport channels, not critical if this fails since everything else
	// has been closed
	__unexport_channels(ss);

	init_flag[ss] = 0;
	return 0;

}


int rc_pwm_set_duty(int ss, char ch, double duty)
{
	int len, ret, duty_ns;
	char buf[MAXBUF];

	// sanity checks
	if(unlikely(ss<0 || ss>2)){
		fprintf(stderr,"ERROR in rc_pwm_set_duty, PWM subsystem must be between 0 and 2\n");
		return -1;
	}
	if(unlikely(init_flag[ss]==0)){
		fprintf(stderr, "ERROR in rc_pwm_set_duty, subsystem %d not initialized yet\n", ss);
		return -1;
	}
	if(unlikely(duty > 1.0 || duty<0.0)){
		fprintf(stderr,"ERROR in rc_pwm_set_duty, duty must be between 0.0f & 1.0f\n");
		return -1;
	}

	// set the duty
	duty_ns = duty*period_ns[ss];
	len = snprintf(buf, sizeof(buf), "%d", duty_ns);
	switch(ch){
	case 'A':
		ret=write(dutyA_fd[ss], buf, len);
		break;
	case 'B':
		ret=write(dutyB_fd[ss], buf, len);
		break;
	default:
		fprintf(stderr,"ERROR in rc_pwm_set_duty_ns, pwm channel must be 'A' or 'B'\n");
		return -1;
	}

	// make sure write worked
	if(unlikely(ret==-1)){
		perror("ERROR in rc_pwm_set_duty_ns, failed to write to duty_cycle fd");
		return -1;
	}
	return 0;
}


int rc_pwm_set_duty_ns(int ss, char ch, unsigned int duty_ns)
{
	int len, ret;
	char buf[MAXBUF];

	// sanity checks
	if(unlikely(ss<0 || ss>2)){
		fprintf(stderr,"ERROR in rc_pwm_set_duty_ns, PWM subsystem must be between 0 and 2\n");
		return -1;
	}
	if(unlikely(init_flag[ss]==0)){
		fprintf(stderr, "ERROR in rc_pwm_set_duty_ns, subsystem %d not initialized yet\n", ss);
		return -1;
	}
	if(unlikely(duty_ns>period_ns[ss])){
		fprintf(stderr,"ERROR in rc_pwm_set_duty_ns, duty must be between 0 & %d for current frequency\n", period_ns[ss]);
		return -1;
	}

	// set the duty
	len = snprintf(buf, sizeof(buf), "%d", duty_ns);
	switch(ch){
	case 'A':
		ret=write(dutyA_fd[ss], buf, len);
		break;
	case 'B':
		ret=write(dutyB_fd[ss], buf, len);
		break;
	default:
		fprintf(stderr,"ERROR in rc_pwm_set_duty_ns, pwm channel must be 'A' or 'B'\n");
		return -1;
	}

	// make sure write worked
	if(unlikely(ret==-1)){
		perror("ERROR in rc_pwm_set_duty_ns, failed to write to duty_cycle fd");
		return -1;
	}
	return 0;
}
