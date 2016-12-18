
#include "../roboticscape.h"
#include "ti_pwm_userspace_defs.h"
#include <stdio.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>


#define MAXBUF 64
#define DEFAULT_FREQ 20000 // 40khz pwm freq

// variables
int duty_fd[6]; 	// pointers to duty cycle file descriptor
int period_ns[3]; 	//one period (frequency) per subsystem
char simple_pwm_initialized[3] = {0,0,0};
int ver; // pwm driver version, 0 or 1. automatically detected

/*******************************************************************************
* int simple_init_pwm(int ss, int frequency)
*
* configures subsystem 0, 1, or 2 to operate at a particular frequency.
*******************************************************************************/
int simple_init_pwm(int ss, int frequency){
	int export_fd;
	int periodA_fd; // pointers to frequency file pointer
	int periodB_fd;
	int enableA_fd;  // run (enable) file pointers
	int enableB_fd;
	int polarityA_fd;
	int polarityB_fd;
	char buf[MAXBUF];
	int len;
	
	if(ss<0 || ss>2){
		printf("PWM subsystem must be between 0 and 2\n");
		return -1;
	}
	
	// check driver is loaded
	if(access(pwm_export_path[0][ss], F_OK ) == 0) ver=0;
	else if(access(pwm_export_path[1][ss], F_OK ) == 0) ver=1;
	else{
		printf("ERROR: ti-pwm driver not loaded for pwm subsystem %d\n", ss);
		return -1;
	}
		
	// open export file for that subsystem
	export_fd = open(pwm_export_path[ver][ss], O_WRONLY);
	if (export_fd < 0) {
		printf("error opening pwm export file for writing\n");
		return -1;
	}

	// export just the A channel for that subsystem and check that it worked
	write(export_fd, "0", 1); 
	if(access(pwm_chA_enable_path[ver][ss], F_OK ) != 0){
		printf("ERROR: export failed for hrpwm%d channel A\n", ss);
		return -1;
	}
	
	// set up file descriptors for A channel
	enableA_fd = open(pwm_chA_enable_path[ver][ss], O_WRONLY);
	periodA_fd = open(pwm_chA_period_path[ver][ss], O_WRONLY);
	duty_fd[(2*ss)] = open(pwm_chA_duty_path[ver][ss], O_WRONLY);
	polarityA_fd = open(pwm_chA_polarity_path[ver][ss], O_WRONLY);

	
	// disable A channel and set polarity before period
	write(enableA_fd, "0", 1);
	write(duty_fd[(2*ss)], "0", 1); // set duty cycle to 0
	write(polarityA_fd, "0", 1); // set the polarity

	// set the period in nanoseconds
	period_ns[ss] = 1000000000/frequency;
	len = snprintf(buf, sizeof(buf), "%d", period_ns[ss]);
	write(periodA_fd, buf, len);

	
	// now we can set up the 'B' channel since the period has been set
	// the driver will not let you change the period when both are exported
	
	// export the B channel and check that it worked
	write(export_fd, "1", 1);
	if(access(pwm_chB_enable_path[ver][ss], F_OK ) != 0){
		printf("ERROR: export failed for hrpwm%d channel B\n", ss);
		return -1;
	}

	// set up file descriptors for B channel
	enableB_fd = open(pwm_chB_enable_path[ver][ss], O_WRONLY);
	periodB_fd = open(pwm_chB_period_path[ver][ss], O_WRONLY);
	duty_fd[(2*ss)+1] = open(pwm_chB_duty_path[ver][ss], O_WRONLY);
	polarityB_fd = open(pwm_chB_polarity_path[ver][ss], O_WRONLY);
	
	// disable and set polarity before period
	write(enableB_fd, "0", 1);
	write(polarityB_fd, "0", 1);
	write(duty_fd[(2*ss)+1], "0", 1);
	
	// set the period to match the A channel
	len = snprintf(buf, sizeof(buf), "%d", period_ns[ss]);
	write(periodB_fd, buf, len);
	
	// enable A&B channels
	write(enableA_fd, "1", 1);
	write(enableB_fd, "1", 1);
	
	// close all the files
	close(export_fd);
	close(enableA_fd);
	close(enableB_fd);
	close(periodA_fd);
	close(periodB_fd);
	close(polarityA_fd);
	close(polarityB_fd);
	
	// everything successful
	simple_pwm_initialized[ss] = 1;
	return 0;
}

/*******************************************************************************
* int simple_uninit_pwm(int ss){
*
* unexports a subsystem to put it into low-power state
*******************************************************************************/
int simple_uninit_pwm(int ss){
	int fd;

	// sanity check
	if(ss<0 || ss>2){
		printf("PWM subsystem must be between 0 and 2\n");
		return -1;
	}

	// attempt both driver versions
	if(access(pwm_unexport_path[0][ss], F_OK ) == 0) ver=0;
	else if(access(pwm_unexport_path[1][ss], F_OK ) == 0) ver=1;
	else{
		printf("ERROR: ti-pwm driver not loaded for pwm subsystem %d\n", ss);
		return -1;
	}

	// open the unexport file for that subsystem
	fd = open(pwm_unexport_path[ver][ss], O_WRONLY);
	if (fd < 0) {
		printf("error opening pwm unexport file for hrpwm%d\n", ss);
		return -1;
	}

	// write 0 and 1 to the file to unexport both channels
	write(fd, "0", 1);
	write(fd, "1", 1);

	close(fd);
	simple_pwm_initialized[ss] = 0;
	return 0;
	
}

/*******************************************************************************
* int simple_set_pwm_duty(int ss, char ch, double duty)
*
* updates the duty cycle through the file system. see mmap_pwmss.h for a faster
* method.
*******************************************************************************/
int simple_set_pwm_duty(int ss, char ch, double duty){
	// start with sanity checks
	if(duty>1.0 || duty<0.0){
		printf("duty must be between 0.0 & 1.0\n");
		return -1;
	}
	
	// set the duty
	int duty_ns = duty*period_ns[ss];
	return simple_set_pwm_duty_ns(ss, ch, duty_ns);
}

/*******************************************************************************
* int simple_set_pwm_duty_ns(int ss, char ch, int duty_ns)
*
* like simple_set_pwm_duty() but takes a width in nanoseconds.
*******************************************************************************/
int simple_set_pwm_duty_ns(int ss, char ch, int duty_ns){
	int len;
	char buf[MAXBUF];
	// start with sanity checks
	if(ss<0 || ss>2){
		printf("PWM subsystem must be between 0 and 2\n");
		return -1;
	}
	// initialize subsystem if not already
	if(simple_pwm_initialized[ss]==0){
		printf("initializing PWMSS%d with default PWM frequency\n", ss);
		simple_init_pwm(ss, DEFAULT_FREQ);
	}
	// boundary check
	if(duty_ns>period_ns[ss] || duty_ns<0){
		printf("duty must be between 0 & period_ns\n");
		return -1;
	}
	
	// set the duty
	len = snprintf(buf, sizeof(buf), "%d", duty_ns);
	switch(ch){
	case 'A':
		write(duty_fd[(2*ss)], buf, len);
		break;
	case 'B':
		write(duty_fd[(2*ss)+1], buf, len);
		break;
	default:
		printf("pwm channel must be 'A' or 'B'\n");
		return -1;
	}
	
	return 0;
	
}

