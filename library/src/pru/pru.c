/**
 * @file pru.c
 *
 * @brief      C interface for starting and stopping the PRU from userspace.
 *
 *             This is primarily for the PRU-dependent servo and encoder
 *             functions to use, however the user may elect to use their own PRU
 *             routines separately from those.
 *
 * @author James Strawson
 * @date 3/15/2018
 */


#include <stdio.h>
#include <fcntl.h>	// for open
#include <unistd.h>	// for close
#include <errno.h>
#include <sys/mman.h>	// mmap
#include <string.h>
#include <rc/time.h>
#include <rc/pru.h>

// remoteproc driver
#define PRU0_STATE	"/dev/remoteproc/pruss-core0/state"
#define PRU1_STATE	"/dev/remoteproc/pruss-core1/state"
#define PRU0_FW		"/dev/remoteproc/pruss-core0/firmware"
#define PRU1_FW		"/dev/remoteproc/pruss-core1/firmware"

// share memory pointer location
#define PRU_ADDR	0x4A300000	// Start of PRU memory Page 184 am335x TRM
#define PRU_LEN		0x80000		// Length of PRU memory
#define PRU_SHAREDMEM	0x10000		// Offset to shared memory

static volatile unsigned int* shared_mem_32bit_ptr = NULL;

int rc_pru_start(int ch, const char* fw_name)
{
	int fd, ret;
	char buf[64];

	// sanity checks
	if(ch!=0 && ch!=1){
		fprintf(stderr, "ERROR in rc_pru_start, PRU channel must be 0 or 1\n");
		return -1;
	}
	if(fw_name==NULL){
		fprintf(stderr, "ERROR in rc_pru_start, received NULL pointer\n");
		return -1;
	}
	// check firmware exists
	memset(buf,0,sizeof(buf));
	snprintf(buf, sizeof(buf), "/lib/firmware/%s", fw_name);
	if(access(buf, F_OK)!=0){
		fprintf(stderr, "ERROR in rc_pru_start, requested firmware %s doesn't exist in /lib/firmware\n", fw_name);
		return -1;
	}

	if(rc_pru_stop(ch)) return -1;

	// write firmware title
	if(ch==0)	fd=open(PRU0_FW, O_WRONLY);
	else		fd=open(PRU1_FW, O_WRONLY);
	if(fd==-1){
		perror("ERROR in rc_pru_start opening remoteproc driver");
		fprintf(stderr,"need to be root to use the pru\n");
		return -1;
	}
	ret = write(fd, fw_name, strlen(fw_name));
	close(fd);
	if(ret==-1){
		perror("ERROR in rc_pru_start setting firmware name");
		return -1;
	}


	// open state fd to start pru
	if(ch==0)	fd=open(PRU0_STATE, O_WRONLY);
	else		fd=open(PRU1_STATE, O_WRONLY);
	if(fd==-1){
		perror("ERROR in rc_pru_start opening remoteproc driver");
		fprintf(stderr,"PRU probably not enabled in device tree\n");
		return -1;
	}
	ret=write(fd, "start", 5);
	close(fd);
	if(ret==-1){
		perror("ERROR in rc_pru_start starting remoteproc");
		return -1;
	}

	// wait for it to start and make sure it's running
	rc_usleep(250000);
	if(ch==0)	fd=open(PRU0_STATE, O_RDONLY);
	else		fd=open(PRU1_STATE, O_RDONLY);
	if(fd==-1){
		perror("ERROR in rc_pru_start opening remoteproc driver");
		fprintf(stderr,"PRU probably not enabled in device tree\n");
		return -1;
	}
	memset(buf,0,sizeof(buf));
	ret=read(fd, buf, sizeof(buf));
	close(fd);
	if(ret==-1){
		perror("ERROR in rc_pru_start reading state");
		return -1;
	}
	if(strcmp(buf,"running\n")){
		fprintf(stderr,"ERROR: in rc_pru_init, pru%d failed to start\n", ch);
		fprintf(stderr,"expected state to become 'running', instead is: %s\n",buf);
		return -1;
	}

	return 0;
}


volatile uint32_t* rc_pru_shared_mem_ptr(void)
{
	int fd;
	volatile unsigned int* map;

	// if already set, just return the pointer
	if(shared_mem_32bit_ptr!=NULL){
		return shared_mem_32bit_ptr;
	}

	// map shared memory
	fd=open("/dev/mem", O_RDWR | O_SYNC);
	if(fd==-1){
		perror("ERROR: in rc_pru_shared_mem_ptr could not open /dev/mem");
		fprintf(stderr, "Need to be root to access PRU shared memory\n");
		return NULL;
	}

	map = mmap(0, PRU_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PRU_ADDR);
	if(map==MAP_FAILED){
		perror("ERROR in rc_pru_shared_mem_ptr failed to map memory");
		close(fd);
		return NULL;
	}
	close(fd);

	// set global shared memory pointer
	// Points to start of shared memory
	shared_mem_32bit_ptr = map + PRU_SHAREDMEM/4;

	return shared_mem_32bit_ptr;
}


int rc_pru_stop(int ch)
{
	int fd, ret;
	char buf[64];

	// sanity checks
	if(ch!=0 && ch!=1){
		fprintf(stderr, "ERROR in rc_pru_stop, PRU channel must be 0 or 1\n");
		return -1;
	}

	// check state
	if(ch==0)	fd=open(PRU0_STATE, O_RDONLY);
	else		fd=open(PRU1_STATE, O_RDONLY);
	if(fd==-1){
		perror("ERROR in rc_pru_stop opening remoteproc driver");
		fprintf(stderr,"PRU probably not enabled in device tree\n");
		return -1;
	}
	memset(buf,0,sizeof(buf));
	ret=read(fd, buf, sizeof(buf));
	close(fd);
	if(ret==-1){
		perror("ERROR in rc_pru_stop reading state before stopping");
		close(fd);
		return -1;
	}

	// already stopped, just return
	if(strcmp(buf,"offline\n")==0){
		return 0;
	}
	// if running, stop it
	else if(strcmp(buf,"running\n")==0){
		if(ch==0) fd=open(PRU0_STATE, O_WRONLY);
		else fd=open(PRU1_STATE, O_WRONLY);
		ret=write(fd,"stop",4);
		close(fd);
		if(ret==-1){
			perror("ERROR in rc_pru_stop while writing to remoteproc state");
			return -1;
		}
	}
	// something unexpected
	else{
		fprintf(stderr, "ERROR in rc_pru_stop remoteproc state should be 'offline' or 'running', read:%s\n", buf);
		return -1;
	}

	// wait for PRU to stop and check it stopped
	//rc_usleep(1000000);
	if(ch==0) fd=open(PRU0_STATE, O_RDONLY);
	else fd=open(PRU1_STATE, O_RDONLY);
	memset(buf,0,sizeof(buf));
	ret=read(fd, buf, sizeof(buf));
	close(fd);
	if(ret==-1){
		perror("ERROR in rc_pru_stop reading state after stopping");
		return -1;
	}
	// if we read anything except "offline" there is something weird
	if(strcmp(buf,"offline\n")){
		fprintf(stderr, "ERROR in rc_pru_stop, remoteproc state should now be 'offline', read:%s\n", buf);
		return -1;
	}
	return 0;
}

