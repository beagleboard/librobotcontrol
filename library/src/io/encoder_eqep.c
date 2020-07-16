/**
 * @file encoder_eqep.c
 *
 *
 * @author     James Strawson
 * @date       1/29/2018
 */

#include <stdio.h>
#include <stdlib.h> // for atoi
#include <errno.h>
#include <fcntl.h> // for open
#include <unistd.h> // for close

#include <rc/model.h>
#include <rc/encoder_eqep.h>

// preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)

#define MAX_BUF 64

#define EQEP_BASE0 "/sys/devices/platform/ocp/48300000.epwmss/48300180.eqep"
#define EQEP_BASE1 "/sys/devices/platform/ocp/48302000.epwmss/48302180.eqep"
#define EQEP_BASE2 "/sys/devices/platform/ocp/48304000.epwmss/48304180.eqep"


static int fd[3]; //store file descriptors for 3 position files
static int init_flag = 0; // boolean to check if mem mapped



int rc_encoder_eqep_init(void)
{
	int temp_fd;
	if(init_flag) return 0;

	// enable 3 subsystems
	// subsystem 0
	temp_fd = open(EQEP_BASE0 "/enabled", O_WRONLY);
	if(temp_fd<0){
		perror("ERROR in rc_encoder_eqep_init, failed to open device driver");
		fprintf(stderr,"Perhaps kernel or device tree is too old\n");
		return -1;
	}
	if(write(temp_fd,"1",2)==-1){
		perror("ERROR in rc_encoder_eqep_init, failed to enable device driver");
		return -1;
	}
	close(temp_fd);
	temp_fd = open(EQEP_BASE0 "/position", O_RDWR);
	if(temp_fd<0){
		perror("ERROR in rc_encoder_eqep_init, failed to open device driver");
		fprintf(stderr,"Perhaps kernel or device tree is too old\n");
		return -1;
	}
	if(write(temp_fd,"0",2)==-1){
		perror("ERROR in rc_encoder_eqep_init, failed to zero out position");
		return -1;
	}
	fd[0]=temp_fd;

	// subsystem 1
	if(rc_model()!=MODEL_BB_POCKET) { // Skip since not enabled on Pocket
		temp_fd = open(EQEP_BASE1 "/enabled", O_WRONLY);
		if(temp_fd<0){
			perror("ERROR in rc_encoder_eqep_init, failed to open device driver");
			fprintf(stderr,"Perhaps kernel or device tree is too old\n");
			return -1;
		}
		if(write(temp_fd,"1",2)==-1){
			perror("ERROR in rc_encoder_eqep_init, failed to enable device driver");
			return -1;
		}
		close(temp_fd);
		temp_fd = open(EQEP_BASE1 "/position", O_RDWR);
		if(temp_fd<0){
			perror("ERROR in rc_encoder_eqep_init, failed to open device driver");
			fprintf(stderr,"Perhaps kernel or device tree is too old\n");
			return -1;
		}
		if(write(temp_fd,"0",2)==-1){
			perror("ERROR in rc_encoder_eqep_init, failed to zero out position");
			return -1;
		}
	}
	fd[1]=temp_fd;

	// subsystem 0
	temp_fd = open(EQEP_BASE2 "/enabled", O_WRONLY);
	if(temp_fd<0){
		perror("ERROR in rc_encoder_eqep_init, failed to open device driver");
		fprintf(stderr,"Perhaps kernel or device tree is too old\n");
		return -1;
	}
	if(write(temp_fd,"1",2)==-1){
		perror("ERROR in rc_encoder_eqep_init, failed to enable device driver");
		return -1;
	}
	close(temp_fd);
	temp_fd = open(EQEP_BASE2 "/position", O_RDWR);
	if(temp_fd<0){
		perror("ERROR in rc_encoder_eqep_init, failed to open device driver");
		fprintf(stderr,"Perhaps kernel or device tree is too old\n");
		return -1;
	}
	if(write(temp_fd,"0",2)==-1){
		perror("ERROR in rc_encoder_eqep_init, failed to zero out position");
		return -1;
	}
	fd[2]=temp_fd;



	init_flag = 1;
	return 0;
}

int rc_encoder_eqep_cleanup(void)
{
	int i;
	for(i=0;i<3;i++){
		close(fd[i]);
	}
	init_flag = 0;
	return 0;
}


int rc_encoder_eqep_read(int ch)
{
	char buf[12];

	//sanity checks
	if(unlikely(!init_flag)){
		fprintf(stderr,"ERROR in rc_encoder_eqep_read, please initialize with rc_encoder_eqep_init() first\n");
		return -1;
	}
	if(unlikely(ch==4)){
		fprintf(stderr,"ERROR in rc_encoder_eqep_read, channel 4 is read by the PRU, use rc_encoder_pru_read instead\n");
		return -1;
	}
	if(unlikely(ch<1 || ch>3)){
		fprintf(stderr,"ERROR: in rc_encoder_eqep_read, encoder channel must be between 1 & 3 inclusive\n");
		return -1;
	}
	// seek to beginning of file and read
	if(unlikely(lseek(fd[ch-1],0,SEEK_SET)<0)){
		perror("ERROR: in rc_encoder_eqep_read, failed to seek to beginning of fd");
		return -1;
	}
	if(unlikely(read(fd[ch-1], buf, sizeof(buf))==-1)){
		perror("ERROR in rc_encoder_eqep_read, can't read position fd");
		return -1;
	}
	return atoi(buf);
}



int rc_encoder_eqep_write(int ch, int pos)
{
	char buf[12];
	//sanity checks
	if(unlikely(!init_flag)){
		fprintf(stderr,"ERROR in rc_encoder_eqep_write, please initialize with rc_encoder_eqep_init() first\n");
		return -1;
	}
	if(unlikely(ch==4)){
		fprintf(stderr,"ERROR in rc_encoder_eqep_write, channel 4 is read by the PRU, use rc_encoder_pru_write instead\n");
		return -1;
	}
	if(unlikely(ch<1 || ch>3)){
		fprintf(stderr,"ERROR: in rc_encoder_eqep_write, encoder channel must be between 1 & 3 inclusive\n");
		return -1;
	}
	if(unlikely(lseek(fd[ch-1],0,SEEK_SET)<0)){
		perror("ERROR: in rc_encoder_eqep_write, failed to seek to beginning of fd");
		return -1;
	}
	sprintf(buf,"%d",pos);
	if(unlikely(write(fd[ch-1], buf, sizeof(buf))==-1)){
		perror("ERROR in rc_encoder_eqep_write, can't write position fd");
		return -1;
	}
	return 0;
}


