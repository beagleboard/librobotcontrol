/*
 * @file rc_test_encoders_pru.c
 *
 * @example    rc_test_encoders_pru
 *
 * Prints out current quadrature position for channel 4 which is counted
 * using the PRU. Channels 1-3 are counted by the eQEP modules, test those with
 * rc_test_encoders_eqep instead.
 */

#include <stdio.h>
#include <signal.h>
#include <rc/encoder_pru.h>
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
	// initialize hardware first
	if(rc_encoder_pru_init()){
		fprintf(stderr,"ERROR: failed to run rc_encoder_pru_init\n");
		return -1;
	}

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running=1;

	printf("\nRaw encoder position\n");
	printf("      E4   |");
	printf(" \n");

	while(running){
		printf("\r%10d |", rc_encoder_pru_read());
		fflush(stdout);
		rc_usleep(50000);
	}
	printf("\n");

	rc_encoder_pru_cleanup();
	return 0;
}

