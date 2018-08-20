/*
 * @file rc_test_encoders.c
 *
 * @example    rc_test_encoders
 *
 * Prints out current quadrature position for channels 1-4. 1-3 are counted
 * using the eQEP modules. Channel 4 is counted by the PRU.
 */

#include <stdio.h>
#include <signal.h>
#include <rc/encoder.h>
#include <rc/time.h>

static int running = 0;

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}

int main()
{
	int i;

	// initialize hardware first
	if(rc_encoder_init()){
		fprintf(stderr,"ERROR: failed to run rc_encoder_init\n");
		return -1;
	}

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running=1;

	printf("\nRaw encoder positions\n");
	printf("      E1   |");
	printf("      E2   |");
	printf("      E3   |");
	printf("      E4   |");
	printf(" \n");

	while(running){
		printf("\r");
		for(i=1;i<=4;i++){
			printf("%10d |", rc_encoder_read(i));
		}
		fflush(stdout);
		rc_usleep(50000);
	}
	printf("\n");

	rc_encoder_cleanup();
	return 0;
}

