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
#include <getopt.h>
#include <rc/dsm.h>
#include <rc/time.h>

// possible printing modes, user selected with command line arguments
typedef enum p_mode_t{
	P_MODE_NONE,
	P_MODE_RAW,
	P_MODE_NORM
} p_mode_t;

static int running = 0;
static p_mode_t print_mode;

// printed if some invalid argument was given
static void __print_usage(void)
{
	printf("\n");
	printf("-r	print raw channel values in microseconds\n");
	printf("-n	print normalized channel values/s\n");
	printf("-h	print this help message\n");
	printf("\n");
}

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}

static void __new_dsm_data_callback(void)
{
	int i;
	printf("\r");// keep printing on same line
	int channels = rc_dsm_channels();
	// print framerate
	printf("%d/", rc_dsm_resolution());
	// print num channels in use
	printf("%d-ch ", channels);
	// print all channels
	if(print_mode==P_MODE_NORM){
		for(i=0;i<channels;i++){
			printf("%d:% 0.2f ", i+1, rc_dsm_ch_normalized(i+1));
		}
	}
	else{
		for(i=0;i<channels;i++){
			printf("%d:% 4d ", i+1, rc_dsm_ch_raw(i+1));
		}
	}
	fflush(stdout);
}

int main(int argc, char *argv[])
{
	int c;
	print_mode = P_MODE_NONE;

	// parse arguments
	opterr = 0;
	while ((c = getopt(argc, argv, "rnh")) != -1){
		switch (c){
		case 'r':
			if(print_mode!=P_MODE_NONE){
				printf("\ntoo many arguments given\n");
				__print_usage();
				return -1;
			}
			print_mode = P_MODE_RAW;
			break;
		case 'n':
			if(print_mode!=P_MODE_NONE){
				printf("\ntoo many arguments given\n");
				__print_usage();
				return -1;
			}
			print_mode = P_MODE_NORM;
			break;
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

	if(print_mode==P_MODE_NONE){
		fprintf(stderr, "Please select raw or normalized mode\n");
		__print_usage();
		return -1;
	}


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
	signal(SIGINT, __signal_handler);
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
	rc_dsm_set_callback(__new_dsm_data_callback);

	// main loop monitor if connection is active or now
	while(running){
		if(rc_dsm_is_connection_active()==0){
			printf("\rSeconds since last DSM packet: ");
			printf("%0.1f ", rc_dsm_nanos_since_last_packet()/1000000000.0);
			printf("                             ");
			fflush(stdout);
		}
		rc_usleep(500000);
	}

	rc_dsm_cleanup();
	printf("\n");
	return 0;
}