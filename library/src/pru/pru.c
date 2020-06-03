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
#warning These need to be against aliases as different numbers of remoteprocX can potentially be loaded
#define PRU0_STATE	"/sys/class/remoteproc/remoteproc1/state"
#define PRU1_STATE	"/sys/class/remoteproc/remoteproc2/state"
#define PRU1_0_STATE	"/sys/class/remoteproc/remoteproc0/state"
#define PRU1_1_STATE	"/sys/class/remoteproc/remoteproc1/state"
#define PRU2_0_STATE	"/sys/class/remoteproc/remoteproc2/state"
#define PRU2_1_STATE	"/sys/class/remoteproc/remoteproc3/state"
#define PRU0_FW		"/sys/class/remoteproc/remoteproc1/firmware"
#define PRU1_FW		"/sys/class/remoteproc/remoteproc2/firmware"
#define PRU1_0_FW	"/sys/class/remoteproc/remoteproc0/firmware"
#define PRU1_1_FW	"/sys/class/remoteproc/remoteproc1/firmware"
#define PRU2_0_FW	"/sys/class/remoteproc/remoteproc2/firmware"
#define PRU2_1_FW	"/sys/class/remoteproc/remoteproc3/firmware"

// share memory pointer location
#define AM335X_PRU_ADDR		0x4A300000	// Start of PRU memory Page 184 am335x TRM
#define AM57XX_PRUSS1_ADDR	0x4B200000	// Start of PRUSS1 memory for AM57xx - See spruhz6
#define AM57XX_PRUSS2_ADDR	0x4B280000	// Start of PRUSS2 memory for AM57xx
/*
 * TODO: Use /dev/bone/pru/X aliases in the future in case other uio drivers are loaded
 *
 * TODO: Use /dev/uioX to isolate permissions as well as abstract the offset
 */
#define PRU_LEN		0x20000		// Length of PRU memory (don't reach into registers)
#define PRU_SHAREDMEM	0x10000		// Offset to shared memory

static volatile unsigned int* shared_mem_32bit_ptr[6];

static int __open_pru_ch(int ch, int state, int flags)
{
	int fd;

	switch(ch) {
	case 0:
		fd=open(state?PRU0_STATE:PRU0_FW, flags);
		break;
	case 1:
		fd=open(state?PRU1_STATE:PRU1_FW, flags);
		break;
	case 2:
		fd=open(state?PRU1_0_STATE:PRU1_0_FW, flags);
		break;
	case 3:
		fd=open(state?PRU1_1_STATE:PRU1_1_FW, flags);
		break;
	case 4:
		fd=open(state?PRU2_0_STATE:PRU2_0_FW, flags);
		break;
	case 5:
		fd=open(state?PRU2_1_STATE:PRU2_1_FW, flags);
		break;
	default:
		fd = -1;
		break;
	}
	return fd;
}

int rc_pru_start(int ch, const char* fw_name)
{
	int fd, ret;
	char buf[64];

	// sanity checks
	//if(ch!=0 && ch!=1){
		//fprintf(stderr, "ERROR in rc_pru_start, PRU channel must be 0 or 1\n");
		//return -1;
	//}
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
	fd=__open_pru_ch(ch, 1, O_WRONLY);
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
	off_t offset = 0;

	// if already set, just return the pointer
	if(shared_mem_32bit_ptr[ch]!=NULL){
		return shared_mem_32bit_ptr[ch];
	}

	// map shared memory
	// TODO: use symlinks for repeatability
	switch(ch) {
	case 0:
	case 1:
		//offset=AM335X_PRU_ADDR;
		fd=open("/dev/uio0", O_RDWR | O_SYNC);
		break;
	case 2:
	case 3:
		//offset=AM57XX_PRUSS1_ADDR;
		fd=open("/dev/uio0", O_RDWR | O_SYNC);
		break;
	case 4:
	case 5:
		//offset=AM57XX_PRUSS2_ADDR;
		fd=open("/dev/uio1", O_RDWR | O_SYNC);
		//fd=open("/dev/mem", O_RDWR | O_SYNC);
		break;
	default:
		fd=-1;
		break;
	}
	if(fd==-1){
		perror("ERROR: in rc_pru_shared_mem_ptr could not open /dev/XXX");	// TODO: name interface
		fprintf(stderr, "Need to be root to access PRU shared memory\n");	// TODO: don't need to be root for /dev/uioX, but need permissions
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
	shared_mem_32bit_ptr[ch] = map + PRU_SHAREDMEM/4;
	printf("%p %p\n", map, shared_mem_32bit_ptr[ch]);

	return shared_mem_32bit_ptr[ch];
}


int rc_pru_stop(int ch)
{
	int fd, ret;
	char buf[64];

	// sanity checks
	//if(ch!=0 && ch!=1){
		//fprintf(stderr, "ERROR in rc_pru_stop, PRU channel must be 0 or 1\n");
		//return -1;
	//}

	// check state
	fd=__open_pru_ch(ch, 1, O_RDONLY);
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

