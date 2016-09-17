/*
Copyright (c) 2015, James Strawson
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*/

#include "simple_pwm.h"
#include <stdio.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

//#define DEBUG
#define MAXBUF 64
#define DEFAULT_FREQ 40000 // 40khz pwm freq
#define SYSFS_PWM_DIR "/sys/class/pwm"

int duty_fd[6]; 	// pointers to duty cycle file descriptor
int period_ns[3]; 	//one period (frequency) per subsystem
char simple_pwm_initialized[3] = {0,0,0};

int simple_init_pwm(int subsystem, int frequency){
	int export_fd, len;
	char buf[MAXBUF];
	DIR* dir;
	int periodA_fd; // pointers to frequency file pointer
	int periodB_fd;
	int runA_fd;  	// run (enable) file pointers
	int runB_fd;
	int polarityA_fd;
	int polarityB_fd;
	
	if(subsystem<0 || subsystem>2){
		printf("PWM subsystem must be between 0 and 2\n");
		return -1;
	}
	
	// // unexport the channels first
	// simple_uninit_pwm(subsystem);
	 
	// open export file for that subsystem
	len = snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwmchip%d/export", 2*subsystem);
	export_fd = open(buf, O_WRONLY);
	if (export_fd < 0) {
		printf("error opening pwm export file\n");
		return -1;
	}

	// export just the A channel for that subsystem
	write(export_fd, "0", 1); 
	
	//check that the right pwm directories were created
	len = snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwmchip%d/pwm0", 2*subsystem);
	dir = opendir(buf);
	if (dir!=NULL) closedir(dir); //success
	else{
		printf("failed to export pwmss%d chA\n",subsystem);
		return -1;
	}
	
	// set up file descriptors for A channel
	len = snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwmchip%d/pwm0/enable", 2*subsystem);
	runA_fd = open(buf, O_WRONLY);
	len = snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwmchip%d/pwm0/period", 2*subsystem);
	periodA_fd = open(buf, O_WRONLY);
	len = snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwmchip%d/pwm0/duty_cycle", 2*subsystem);
	duty_fd[(2*subsystem)] = open(buf, O_WRONLY);
	len = snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwmchip%d/pwm0/polarity", 2*subsystem);
	polarityA_fd = open(buf, O_WRONLY);

	
	// disable A channel and set polarity before setting frequency
	write(runA_fd, "0", 1);
	write(duty_fd[(2*subsystem)], "0", 1); // set duty cycle to 0
	write(polarityA_fd, "0", 1); // set the polarity

	// set the period in nanoseconds
	period_ns[subsystem] = 1000000000/frequency;
	len = snprintf(buf, sizeof(buf), "%d", period_ns[subsystem]);
	write(periodA_fd, buf, len);

	
	// now we can set up the 'B' channel since the period has been set
	// the driver will not let you change the period when both are exported
	
	// export the B channel
	write(export_fd, "1", 1);
	len = snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwmchip%d/pwm1", 2*subsystem);
	dir = opendir(buf);
	if (dir!=NULL) closedir(dir); //success
	else{
		printf("failed to export pwmss%d chB\n",2*subsystem);
		return -1;
	}
	// set up file descriptors for B channel
	len = snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwmchip%d/pwm1/enable", 2*subsystem);
	runB_fd = open(buf, O_WRONLY);
	len = snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwmchip%d/pwm1/period", 2*subsystem);
	periodB_fd = open(buf, O_WRONLY);
	len = snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwmchip%d/pwm1/duty_cycle", 2*subsystem);
	duty_fd[(2*subsystem)+1] = open(buf, O_WRONLY);
	len = snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwmchip%d/pwm1/polarity", 2*subsystem);
	polarityB_fd = open(buf, O_WRONLY);
	
	// disable the run value and set polarity before period
	write(runB_fd, "0", 1);
	write(polarityB_fd, "0", 1);
	write(duty_fd[(2*subsystem)+1], "0", 1);
	
	// set the period to match the A channel
	len = snprintf(buf, sizeof(buf), "%d", period_ns[subsystem]);
	write(periodB_fd, buf, len);
	
	// enable A&B channels
	write(runA_fd, "1", 1);
	write(runB_fd, "1", 1);
	
	// close all the files
	close(export_fd);
	close(runA_fd);
	close(runB_fd);
	close(periodA_fd);
	close(periodB_fd);
	close(polarityA_fd);
	close(polarityB_fd);
	
	// everything successful
	simple_pwm_initialized[subsystem] = 1;
	return 0;
}

int simple_uninit_pwm(int subsystem){
	int fd;
	char buf[MAXBUF];

	// sanity check
	if(subsystem<0 || subsystem>2){
		printf("PWM subsystem must be between 0 and 2\n");
		return -1;
	}

	// open the unexport file for that subsystem
	snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwmchip%d/unexport", 2*subsystem);
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		printf("error opening pwm export file\n");
		return -1;
	}

	// write 0 and 1 to the file to unexport both channels
	write(fd, "0", 1);
	write(fd, "1", 1);

	close(fd);
	simple_pwm_initialized[subsystem] = 0;
	return 0;
	
}


int simple_set_pwm_duty(int subsystem, char ch, float duty){
	// start with sanity checks
	if(duty>1.0 || duty<0.0){
		printf("duty must be between 0.0 & 1.0\n");
		return -1;
	}
	
	// set the duty
	int duty_ns = duty*period_ns[subsystem];
	return simple_set_pwm_duty_ns(subsystem, ch, duty_ns);
}

int simple_set_pwm_duty_ns(int subsystem, char ch, int duty_ns){
	int len;
	char buf[MAXBUF];
	// start with sanity checks
	if(subsystem<0 || subsystem>2){
		printf("PWM subsystem must be between 0 and 2\n");
		return -1;
	}
	// initialize subsystem if not already
	if(simple_pwm_initialized[subsystem]==0){
		printf("initializing PWMSS%d with default PWM frequency\n", subsystem);
		simple_init_pwm(subsystem, DEFAULT_FREQ);
	}
	// boundary check
	if(duty_ns>period_ns[subsystem] || duty_ns<0){
		printf("duty must be between 0 & period_ns\n");
		return -1;
	}
	
	// set the duty
	len = snprintf(buf, sizeof(buf), "%d", duty_ns);
	switch(ch){
	case 'A':
		write(duty_fd[(2*subsystem)], buf, len);
		break;
	case 'B':
		write(duty_fd[(2*subsystem)+1], buf, len);
		break;
	default:
		printf("pwm channel must be 'A' or 'B'\n");
		return -1;
	}
	
	return 0;
	
}

