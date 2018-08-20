/*
 * @file rc_test_encoders_eqep.c
 *
 * @example    rc_test_encoders_eqep
 *
 * Prints out current quadrature position for channels 1-3 which are counted
 * using the eQEP modules. Channel 4 is counted by the PRU, test that with
 * rc_test_encoders_pru instead.
 */

#include <stdio.h>
#include <signal.h>
#include <rc/encoder_eqep.h>
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
	if(rc_encoder_eqep_init()){
		fprintf(stderr,"ERROR: failed to run rc_encoder_eqep_init\n");
		return -1;
	}

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running=1;

	printf("\nRaw encoder positions\n");
	printf("      E1   |");
	printf("      E2   |");
	printf("      E3   |");
	printf(" \n");

	while(running){
		printf("\r");
		for(i=1;i<=3;i++){
			printf("%10d |", rc_encoder_eqep_read(i));
		}
		fflush(stdout);
		rc_usleep(50000);
	}
	printf("\n");

	rc_encoder_eqep_cleanup();
	return 0;
}

