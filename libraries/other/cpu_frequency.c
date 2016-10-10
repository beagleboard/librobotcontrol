/*******************************************************************************
* cpu_frequency.c
*
* A collection of functions for adjusting the cpu frequency on the beaglebone.
* This is part of the robotics cape library but is not dependent on any other
* component so these functions could be used independently.
*******************************************************************************/

#include "../roboticscape.h"
#include <stdio.h>
#include <fcntl.h>
#include <string.h>

#define GOVERNOR_PATH  "/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"
#define SETSPEED_PATH  "/sys/devices/system/cpu/cpu0/cpufreq/scaling_setspeed"
#define CURFREQ_PATH   "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"


/*******************************************************************************
* int set_cpu_frequency(cpu_frequency_t freq)
*
* Sets the CPU frequency to either a fixed value or to onedemand automatic
* scaling mode. Returns 0 on success, -1 on failure.
*******************************************************************************/
int set_cpu_frequency(cpu_frequency_t freq){
	FILE* gov_fd  = fopen(GOVERNOR_PATH,  "w");
	if (gov_fd == NULL) {
		printf("error opening CPU governor path\n");
		return -1;
	}
	
	// in automatic frequency mode, use the ondemand (default) governor
	if(freq==FREQ_ONDEMAND){
		fprintf(gov_fd, "ondemand");
		fflush(gov_fd);
		fclose(gov_fd);
		return 0;
	}
	
	// for a specific desired frequency, use userspace governor
	FILE* freq_fd = fopen(SETSPEED_PATH, "w");
	if (freq_fd == NULL) {
		printf("error opening CPU set frequency path\n");
		fclose(gov_fd);
		return -1;
	}
	fprintf(gov_fd, "userspace");
	fflush(gov_fd);
	fclose(gov_fd);
	
	switch(freq){
	case FREQ_300MHZ:
		fprintf(freq_fd, "300000");
		break;
	case FREQ_600MHZ:
		fprintf(freq_fd, "600000");
		break;
	case FREQ_800MHZ:
		fprintf(freq_fd, "800000");
		break;
	case FREQ_1000MHZ:
		fprintf(freq_fd, "1000000");
		break;
	default:
		printf("ERROR: unknown cpu frequency\n");
		fclose(freq_fd);
		return -1;
	}
	fflush(freq_fd);
	fclose(freq_fd);
	return 0;
}

/*******************************************************************************
* cpu_frequency_t get_cpu_frequency()
*
* Returns the current clock speed of the Beaglebone's Sitara processor in the
* form of the provided enumerated type. It will never return the FREQ_ONDEMAND
* value as the intention of this function is to see the clock speed as set by
* either the user or the ondemand governor itself.
*******************************************************************************/
cpu_frequency_t get_cpu_frequency(){
	int freq;
	FILE* freq_fd = fopen(CURFREQ_PATH, "r");
	if (freq_fd < 0) {
		printf("error opening CPU current frequency path\n");
		return -1;
	}
	if(fscanf(freq_fd,"%d",&freq)<0){
		printf("ERROR: failed to read from CPU current frequency path\n");
		return -1;
	}
	fclose(freq_fd);
	switch(freq){
	case 300000:
		return FREQ_300MHZ;
	case 600000:
		return FREQ_600MHZ;
	case 800000:
		return FREQ_800MHZ;
	case 1000000:
		return FREQ_1000MHZ;
	default:
		printf("ERROR: unknown cpu frequency\n");
	}
	return -1;
}

/*******************************************************************************
* int print_cpu_frequency()
*
* Prints the current frequency to the screen. For example "300MHZ".
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int print_cpu_frequency(){
	int freq;
	FILE* freq_fd = fopen(CURFREQ_PATH, "r");
	if (freq_fd == NULL) {
		printf("error opening CPU current frequency path\n");
		return -1;
	}
	if(fscanf(freq_fd,"%d",&freq)<0){
		printf("ERROR: failed to read from CPU current frequency path\n");
		return -1;
	}
	fclose(freq_fd);
	printf("%dmhz", freq/1000);
	return 0;
}



