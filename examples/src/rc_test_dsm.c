/**
 * @example    rc_test_dsm.c
 *
 * Prints out the normalized dsm2/dsmx values. Make sure the transmitter and
 * receiver are paired before testing. Use the rc_pair_dsm example if you didn't
 * already bind receiver to transmitter with a bind plug the old fashioned way.
 * The satellite receiver remembers which transmitter it is paired to, not your
 * BeagleBone.
 *
 * If the values you read are not normalized between +-1, then you should run
 * the rc_calibrate_dsm example to save your particular transmitter's min and
 * max channel values.
 */

#include <stdio.h>
#include <signal.h>
#include <rc/dsm.h>
#include <rc/time.h>

int running;

// interrupt handler to catch ctrl-c
void signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}

void new_dsm_data_callback()
{
	int i;
	printf("\r");// keep printing on same line
	int channels = rc_dsm_channels();
	// print framerate
	printf("%d/", rc_dsm_resolution());
	// print num channels in use
	printf("%d-ch ", channels);
	//print all channels
	for(i=0;i<channels;i++){
		printf("%d:% 0.2f ", i+1, rc_dsm_ch_normalized(i+1));
	}
	fflush(stdout);
}

int main()
{
	if(rc_dsm_init()) return -1;

	printf("\n");
	printf("Make sure transmitter and receiver are bound and on.\n");
	printf("If data is received, the normalized values will be printed\n");
	printf("here along with the bit resolution and the number of channels\n");
	printf("\n");
	printf("If connection is lost the number of seconds since last packet\n");
	printf("will be displayed\n");
	printf("\n");

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, signal_handler);
	running =1;

	printf("Waiting for first packet");
	fflush(stdout);
	while(rc_dsm_is_new_data()==0){
		if(running==0){
			rc_dsm_cleanup();
			return 0;
		}
		rc_usleep(50000);
	}

	// first packet arrived, set the callback and run
	rc_dsm_set_callback(new_dsm_data_callback);

	// main loop monitor if connection is active or now
	while(running){
		if(rc_dsm_is_connection_active()==0){
			printf("\rSeconds since last DSM packet: ");
			printf("%0.1f ", rc_dsm_nanos_since_last_packet()/1000000000.0);
			printf("                             ");
			fflush(stdout);
		}
		rc_usleep(25000);
	}

	rc_dsm_cleanup();
	return 0;
}