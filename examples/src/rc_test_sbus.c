/**
 * @example    rc_test_sbus.c
 *
 * Prints out the normalized sbus values. Make sure the transmitter
 * and receiver are paired before testing.  The satellite receiver
 * remembers which transmitter it is paired to, not your BeagleBone.
 *
 * If the values you read are not normalized between +-1, then you
 * should run the rc_calibrate_sbus example to save your particular
 * transmitter's min and max channel values.
 */

#include <stdio.h>
#include <signal.h>
#include <getopt.h>
#include <rc/sbus.h>
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

static void __new_sbus_data_callback(void)
{
	int i,v;
	printf("\r");// keep printing on same line
	// print all channels
	if(print_mode==P_MODE_NORM){
		for(i=0;i<RC_MAX_SBUS_ANALOG_CHANNELS;i++){
			if (rc_sbus_ch_raw(i+1) != 0) {
				printf("%d:% 0.2f|", i+1, rc_sbus_ch_normalized(i+1));
			}
		}
	}
	else{
		for(i=0;i<RC_MAX_SBUS_ANALOG_CHANNELS;i++){
			v=rc_sbus_ch_raw(i+1);
			if (v != 0) {
				printf("%d:%4d|", i+1, rc_sbus_ch_raw(i+1));
			}
		}
	}
	for(i=0;i<RC_MAX_SBUS_BINARY_CHANNELS;i++){
		printf("%d:%1d|", i+RC_MAX_SBUS_ANALOG_CHANNELS+1, rc_sbus_ch_binary(i+1));
	}
	printf("SQ:%d|FL:%d|TE:%d",
	       rc_sbus_signal_quality (), rc_sbus_lost_frames (), rc_sbus_total_errors ());
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


	if(rc_sbus_init()) return -1;

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
	while(rc_sbus_is_new_data()==0){
		if(running==0){
			rc_sbus_cleanup();
			return 0;
		}
		rc_usleep(50000);
	}

	// first packet arrived, set the callback and run
	rc_sbus_set_callback(__new_sbus_data_callback);

	// main loop monitor if connection is active or now
	while(running){
		if(rc_sbus_is_connection_active()==0){
			printf("\rSeconds since last SBUS packet: ");
			printf("%0.1f ", rc_sbus_nanos_since_last_packet()/1000000000.0);
			printf("                             ");
			fflush(stdout);
		}
		rc_usleep(500000);
	}

	rc_sbus_cleanup();
	printf("\n");
	return 0;
}
