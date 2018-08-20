/**
 * @file rc_cpu.c
 * @example    rc_cpu
 *
 * This cycles the CPU through each available frequency and leaves the frequency
 * set to the default "ondemand" governor.
 */

#include <stdio.h>
#include <getopt.h>
#include <string.h>
#include <rc/cpu.h>

typedef enum mode_t{
	NONE,
	SET,
	READ,
	TEST
}mode_t;


// printed if some invalid argument was given
static void __print_usage(void)
{
	printf("\n");
	printf(" Options\n");
	printf(" -g {gov}   set the governor\n");
	printf(" -r         read the frequency in mhz\n");
	printf(" -t         run a test through all governors\n");
	printf(" -h         print this help message\n\n");
	printf("available governors:\n");
	printf("conservative ondemand powersave performance schedutil\n\n");
	return;
}


static int __run_test(void)
{
	printf("setting governor to POWERSAVE\n");
	if(rc_cpu_set_governor(RC_GOV_POWERSAVE)<0){
		fprintf(stderr,"Failed to set governor to POWERSAVE\n");
		return -1;
	}
	printf("setting governor to PERFORMANCE\n");
	if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)<0){
		fprintf(stderr,"Failed to set governor to PERFORMANCE\n");
		return -1;
	}
	printf("setting governor to SCHEDUTIL\n");
	if(rc_cpu_set_governor(RC_GOV_SCHEDUTIL)<0){
		fprintf(stderr,"Failed to set governor to SCHEDUTIL\n");
		return -1;
	}
	printf("setting governor to CONSERVATIVE\n");
	if(rc_cpu_set_governor(RC_GOV_CONSERVATIVE)<0){
		fprintf(stderr,"Failed to set governor to CONSERVATIVE\n");
		return -1;
	}
	printf("setting governor to ONDEMAND\n");
	if(rc_cpu_set_governor(RC_GOV_ONDEMAND)<0){
		fprintf(stderr,"Failed to set governor to ONDEMAND\n");
		return -1;
	}
	return 0;
}



int main(int argc, char *argv[])
{
	int c;
	mode_t mode = NONE;
	rc_governor_t gov;

	if(argc>3){
		fprintf(stderr,"ERROR: too many arguments\n");
		__print_usage();
		return -1;
	}
	if(argc<2){
		fprintf(stderr,"ERROR: not enough arguments\n");
		__print_usage();
		return -1;
	}

	// parse arguments
	opterr = 0;
	while ((c = getopt(argc, argv, "g:rth")) != -1){
		switch (c){
		case 'g':
			if(mode!=NONE){
				fprintf(stderr,"Invalid Argument\n");
				__print_usage();
				return -1;
			}
			mode = SET;
			if(!strcmp(optarg, "powersave")) gov=RC_GOV_POWERSAVE;
			else if(!strcmp(optarg, "performance")) gov=RC_GOV_PERFORMANCE;
			else if(!strcmp(optarg, "ondemand")) gov=RC_GOV_ONDEMAND;
			else if(!strcmp(optarg, "schedutil")) gov=RC_GOV_SCHEDUTIL;
			else if(!strcmp(optarg, "conservative")) gov=RC_GOV_CONSERVATIVE;
			else{
				fprintf(stderr,"ERROR invalid governor\n");
				__print_usage();
				return -1;
			}
			break;
		case 'r':
			if(mode!=NONE){
				fprintf(stderr,"Invalid Argument\n");
				__print_usage();
				return -1;
			}
			mode = READ;
			break;

		case 't':
			if(mode!=NONE){
				fprintf(stderr,"Invalid Argument\n");
				__print_usage();
				return -1;
			}
			mode = TEST;
			break;

		case 'h':
			__print_usage();
			return 0;

		default:
			fprintf(stderr,"Invalid Argument\n");
			__print_usage();
			return -1;
		}
	}

	switch(mode){
	case NONE:
		__print_usage();
		return 0;
	case READ:
		printf("Current Frequency: ");
		rc_cpu_print_freq();
		printf("\n");
		return 0;
	case TEST:
		return __run_test();
	case SET:
		return rc_cpu_set_governor(gov);
	default:
		fprintf(stderr,"ERROR: Invalid mode\n");
		return -1;
	}

	return 0;
}
