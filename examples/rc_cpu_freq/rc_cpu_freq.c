/*******************************************************************************
* rc_cpu_freq.c
*
* James Strawson 2016
* This cycles the CPU through each available frequency and leaves the frequency
* set to the default "ondemand" governor.
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
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
	// ensure root privaleges until we sort out udev rules
	if(geteuid()!=0){
		fprintf(stderr,"ERROR: must be root to set cpu frequency\n");
		return -1;
	}
	// try 300mhz
	if(rc_set_cpu_freq(FREQ_300MHZ)<0){
		fprintf(stderr,"Failed to set CPU frequency to 300mhz\n");
		return -1;
	}
	printf("\nFrequency set to:     ");  
	rc_print_cpu_freq();
	
	// try 600mhz
	if(rc_set_cpu_freq(FREQ_600MHZ)<0){
		fprintf(stderr,"Failed to set CPU frequency to 600mhz\n");
		return -1;
	}
	printf("\nFrequency set to:     ");  
	rc_print_cpu_freq();
	
	// try 800mhz
	if(rc_set_cpu_freq(FREQ_800MHZ)<0){
		fprintf(stderr,"Failed to set CPU frequency to 800mhz\n");
		return -1;
	}
	printf("\nFrequency set to:     ");  
	rc_print_cpu_freq();
	
	// try 1000mhz
	if(rc_set_cpu_freq(FREQ_1000MHZ)<0){
		fprintf(stderr,"Failed to set CPU frequency to 1000mhz\n");
		return -1;
	}
	printf("\nFrequency set to:     ");  
	rc_print_cpu_freq();
	
	
	// try auto ondemand governor
	if(rc_set_cpu_freq(FREQ_ONDEMAND)<0){
		fprintf(stderr,"Failed to set CPU frequency to ONDEMAND\n");
		return -1;
	}
	printf("\nFrequency set to scale automatically on demand\n\n");  

	return 0;
}



int main(int argc, char *argv[]){
	int c;
	int freq_int = 0;
	test_mode_t test_mode = NOT_SET;
	rc_cpu_freq_t freq;

	// parse arguments
	opterr = 0;
	while ((c = getopt(argc, argv, "s:arth")) != -1){
		switch (c){
		case 's':
			if(test_mode!=NOT_SET){
				fprintf(stderr,"ERROR: too many arguments\n");
				return -1;
			}
			test_mode = SET_FREQ;
			freq_int = atoi(optarg);
			break;

		case 'r':
			if(test_mode!=NOT_SET){
				fprintf(stderr,"ERROR: too many arguments\n");
				return -1;
			}
			test_mode = READ_FREQ;
			break;

		case 'a':
			if(test_mode!=NOT_SET){
				fprintf(stderr,"ERROR: too many arguments\n");
				return -1;
			}
			test_mode = SET_AUTO;
			break;

		case 't':
			if(test_mode!=NOT_SET){
				fprintf(stderr,"ERROR: too many arguments\n");
				return -1;
			}
			test_mode = TEST_FREQ;
			break;

		case 'h':
			 print_usage();
			 return 0;

		default:
			fprintf(stderr,"Invalid Argument\n");
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
		rc_print_cpu_freq();
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
			fprintf(stderr,"Invalid frequency option\n");
			fprintf(stderr,"try: 300, 600, 800, or 1000\n");
			return -1;
		}
		break;
	default:
		fprintf(stderr,"ERROR: Invalid mode\n");
		return -1;
	}

	// ensure root privaleges until we sort out udev rules
	if(geteuid()!=0){
		fprintf(stderr,"ERROR: must be root to set cpu frequency\n");
		return -1;
	}
	return rc_set_cpu_freq(freq);
}
