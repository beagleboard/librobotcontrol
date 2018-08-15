/**
 * @file cpu_freq.c
 */

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
//#include <string.h>
#include <rc/cpu.h>

#define GOVERNOR_PATH "/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"
#define CURFREQ_PATH  "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"


int rc_cpu_set_governor(rc_governor_t gov)
{
	int ret, fd;
	fd = open(GOVERNOR_PATH, O_WRONLY);
	if(fd==-1){
		perror("ERROR in rc_cpu_set_governor");
		if(errno==EPERM){
			fprintf(stderr,"try running as root\n");
		}
		return -1;
	}

	switch(gov){
	case RC_GOV_POWERSAVE:
		ret=write(fd, "powersave", 9);
		break;
	case RC_GOV_PERFORMANCE:
		ret=write(fd, "performance", 11);
		break;
	case RC_GOV_ONDEMAND:
		ret=write(fd, "ondemand", 8);
		break;
	case RC_GOV_SCHEDUTIL:
		ret=write(fd, "schedutil", 9);
		break;
	case RC_GOV_CONSERVATIVE:
		ret=write(fd, "conservative", 12);
		break;
	default:
		fprintf(stderr,"ERROR in rc_cpu_set_governor, unknown governor enum\n");
		close(fd);
		return -1;
	}
	if(ret==-1){
		perror("ERROR in rc_cpu_set_governor");
	}
	close(fd);
	return 0;
}


int rc_cpu_get_freq(void)
{
	int freq;
	FILE* fd = fopen(CURFREQ_PATH, "r");
	if(fd == NULL){
		perror("ERROR in rc_cpu_get_freq");
		return -1;
	}
	if(fscanf(fd,"%d",&freq)!=1){
		perror("ERROR in rc_cpu_get_freq");
		fprintf(stderr,"failed to read from CPU current frequency path\n");
		fclose(fd);
		return -1;
	}
	fclose(fd);
	return freq;
}


int rc_cpu_print_freq(void)
{
	int freq= rc_cpu_get_freq();
	if(freq==-1) return -1;
	printf("%dmhz", freq/1000);
	return 0;
}


