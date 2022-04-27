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

#define NUM_PRU_CONFIGS 6
struct prucfg {
	const char * state;
	const char * firmware;
	const char * shared_mem;
} pru_configs[NUM_PRU_CONFIGS] = {
	{ "/dev/remoteproc/pruss-core0/state", "/dev/remoteproc/pruss-core0/firmware", "/dev/uio/pruss/shared" },
	{ "/dev/remoteproc/pruss-core1/state", "/dev/remoteproc/pruss-core1/firmware", "/dev/uio/pruss/shared" },
	{ "/dev/remoteproc/pruss1-core0/state", "/dev/remoteproc/pruss1-core0/firmware", "/dev/uio/pruss1/pruss/shared" },
	{ "/dev/remoteproc/pruss1-core1/state", "/dev/remoteproc/pruss1-core1/firmware", "/dev/uio/pruss1/pruss/shared" },
	{ "/dev/remoteproc/pruss2-core0/state", "/dev/remoteproc/pruss2-core0/firmware", "/dev/uio/pruss2/pruss/shared" },
	{ "/dev/remoteproc/pruss2-core1/state", "/dev/remoteproc/pruss2-core1/firmware", "/dev/uio/pruss2/pruss/shared" },
};

static volatile unsigned int* shared_mem_ptr[6];

static int __open_pru_ch(int ch, int state, int flags)
{
	int fd;

	// sanity checks
	if(unlikely(ch<0 || ch>=NUM_PRU_CONFIGS)) {
		fprintf(stderr, "ERROR in _open_pru_ch, invalid PRU channel %d\n", ch);
		return -1;
	}

	fd=open(state?pru_configs[ch].state:pru_configs[ch].firmware, flags);
	return fd;
}

int rc_pru_start(int ch, const char* fw_name)
{
	int fd, ret;
	char buf[64];

	// sanity checks
	if(unlikely(ch<0 || ch>=NUM_PRU_CONFIGS)) {
		fprintf(stderr, "ERROR in rc_pru_start, invalid PRU channel %d\n", ch);
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
	fd=__open_pru_ch(ch, 0, O_WRONLY);
	if(fd==-1){
		perror("ERROR in rc_pru_start opening remoteproc driver");
		fprintf(stderr,"need permissions to use the PRU\n");
		return -1;
	}
	ret = write(fd, fw_name, strlen(fw_name));
	close(fd);
	if(ret==-1){
		perror("ERROR in rc_pru_start setting firmware name");
		return -1;
	}


	// open state fd to start pru
	fd=__open_pru_ch(ch, 1, O_WRONLY);
	if(fd==-1){
		perror("ERROR in rc_pru_start opening remoteproc driver");
		fprintf(stderr,"PRU probably not enabled properly in your device tree\n");
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
	fd=__open_pru_ch(ch, 1, O_RDONLY);
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


volatile uint32_t* rc_pru_shared_mem_ptr(int ch)
{
	int fd;
	volatile unsigned int* map;

	// sanity checks
	if(unlikely(ch<0 || ch>=NUM_PRU_CONFIGS)) {
		fprintf(stderr, "ERROR in rc_pru_shared_mem_ptr, invalid PRU channel %d\n", ch);
		return NULL;
	}

	// if already set, just return the pointer
	if(shared_mem_ptr[ch]!=NULL){
		return shared_mem_ptr[ch];
	}

	// map shared memory
	fd=open(pru_configs[ch].shared_mem, O_RDWR | O_SYNC);
	if(fd<0){
		fprintf(stderr, "ERROR: in rc_pru_shared_mem_ptr could not open %s\n", pru_configs[ch].shared_mem);
		fprintf(stderr, "Verify device tree and permissions to access PRU shared memory\n");
		return NULL;
	}

	map = mmap(0, PRU_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
	if(map==MAP_FAILED){
		perror("ERROR in rc_pru_shared_mem_ptr failed to map memory");
		close(fd);
		return NULL;
	}
	close(fd);

	// set global shared memory pointer
	// Points to start of shared memory
	shared_mem_ptr[ch] = map;

	return shared_mem_ptr[ch];
}


int rc_pru_stop(int ch)
{
	int fd, ret;
	char buf[64];

	// sanity checks
	if(unlikely(ch<0 || ch>=NUM_PRU_CONFIGS)) {
		fprintf(stderr, "ERROR in rc_pru_stop, invalid PRU channel %d\n", ch);
		return NULL;
	}

	// check state
	fd=__open_pru_ch(ch, 1, O_RDONLY);
	if(fd==-1){
		perror("ERROR in rc_pru_stop opening remoteproc driver");
		fprintf(stderr,"PRU probably not enabled properly in device tree\n");
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
		fd=__open_pru_ch(ch, 1, O_WRONLY);
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
	fd=__open_pru_ch(ch, 1, O_RDONLY);
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

