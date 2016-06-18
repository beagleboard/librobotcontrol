/*******************************************************************************
* test_cpu_freq.c
*
* This cycles the CPU through each available frequency and leaves the frequency
* set to the default "ondemand" governor.
*******************************************************************************/

#include <robotics_cape.h>
#include <useful_includes.h>

int main(){
	// try 300mhz
	if(set_cpu_frequency(FREQ_300MHZ)<0){
		printf("Failed to set CPU frequency to 300mhz\n");
		return -1;
	}
	printf("\nFrequency set to:     ");  
	print_cpu_frequency();
	
	// try 600mhz
	if(set_cpu_frequency(FREQ_600MHZ)<0){
		printf("Failed to set CPU frequency to 600mhz\n");
		return -1;
	}
	printf("\nFrequency set to:     ");  
	print_cpu_frequency();
	
	// try 800mhz
	if(set_cpu_frequency(FREQ_800MHZ)<0){
		printf("Failed to set CPU frequency to 800mhz\n");
		return -1;
	}
	printf("\nFrequency set to:     ");  
	print_cpu_frequency();
	
	// try 1000mhz
	if(set_cpu_frequency(FREQ_1000MHZ)<0){
		printf("Failed to set CPU frequency to 1000mhz\n");
		return -1;
	}
	printf("\nFrequency set to:     ");  
	print_cpu_frequency();
	
	
	// try auto ondemand governor
	if(set_cpu_frequency(FREQ_ONDEMAND)<0){
		printf("Failed to set CPU frequency to ONDEMAND\n");
		return -1;
	}
	printf("\nFrequency set to scale automatically on demand\n\n");  
	
	
	return 0;
}
