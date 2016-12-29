/*******************************************************************************
* test_cpu_freq.c
*
* James Strawson 2016
* This cycles the CPU through each available frequency and leaves the frequency
* set to the default "ondemand" governor.
*******************************************************************************/

#include "../../libraries/roboticscape-usefulincludes.h"
#include "../../libraries/roboticscape.h"


typedef enum test_mode_t{
	NOT_SET,
	SET_FREQ,
	SET_AUTO,
	READ_FREQ,
	TEST_FREQ
}test_mode_t;

// printed if some invalid argument was given
void print_usage(){
	printf("\n");
	printf(" Options\n");
	printf(" -s {mhz}   set the frequency in mhz\n");
	printf(" -a 	    set auto freuqnecy scaling\n");
	printf(" -r         read the frequency in mhz\n");
	printf(" -t         run a test through all frequencies\n");
	printf(" -h         print this help message\n\n");
}


int run_test(){
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



int main(int argc, char *argv[]){
	int c;
	int freq_int = 0;
	test_mode_t test_mode = NOT_SET;
	cpu_frequency_t freq;

	// parse arguments
	opterr = 0;
	while ((c = getopt(argc, argv, "s:arth")) != -1){
		switch (c){
		case 's':
			if(test_mode!=NOT_SET){
				printf("ERROR: too many arguments\n");
				return -1;
			}
			test_mode = SET_FREQ;
			freq_int = atoi(optarg);
			break;

		case 'r':
			if(test_mode!=NOT_SET){
				printf("ERROR: too many arguments\n");
				return -1;
			}
			test_mode = READ_FREQ;
			break;

		case 'a':
			if(test_mode!=NOT_SET){
				printf("ERROR: too many arguments\n");
				return -1;
			}
			test_mode = SET_AUTO;
			break;

		case 't':
			if(test_mode!=NOT_SET){
				printf("ERROR: too many arguments\n");
				return -1;
			}
			test_mode = TEST_FREQ;
			break;

		case 'h':
			 print_usage();
			 return 0;

		default:
			printf("\nInvalid Argument \n");
			print_usage();
			return -1;
		}
	}

	switch(test_mode){
	case NOT_SET:
		print_usage();
		return 0;
	case READ_FREQ:
		printf("Current Frequency: ");
		print_cpu_frequency();
		printf("\n");
		return 0;
	case SET_AUTO:
		freq = FREQ_ONDEMAND;
		break;
	case TEST_FREQ:
		return run_test();
	case SET_FREQ:
		switch(freq_int){
		case 300:
			freq = FREQ_300MHZ;
			break;
		case 600:
			freq = FREQ_600MHZ;
			break;
		case 800:
			freq = FREQ_800MHZ;
			break;
		case 1000:
			freq = FREQ_1000MHZ;
			break;
		default:
			printf("Invalid frequency option\n");
			printf("try: 300, 600, 800, or 1000\n");
			return -1;
		}
		break;
	default:
		printf("ERROR: Invalid mode\n");
		return -1;
	}

	return set_cpu_frequency(freq);
}
