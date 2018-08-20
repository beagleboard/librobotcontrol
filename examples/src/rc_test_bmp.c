/**
 * @file rc_test_bmp.c
 * @example    rc_test_bmp
 *
 * This serves as an example of how to read the barometer.
 *
 * @author     James Strawson
 * @date       3/14/2018
 */

#include <stdio.h>
#include <signal.h>
#include <rc/math/filter.h>
#include <rc/time.h>
#include <rc/bmp.h>

// choice of 1,2,4,8,16 oversampling. Here we use 16 and sample at 25hz which
// is close to the update rate specified in rc/bmp.h for that oversample.
#define OVERSAMPLE  BMP_OVERSAMPLE_16

// choice of OFF, 2, 4, 8, 16 filter constants. Here we turn off the filter and
// opt to use our own 2nd order filter instead.
#define INTERNAL_FILTER	BMP_FILTER_OFF

// our own low pass filter
#define ORDER		2
#define CUTOFF_FREQ	2.0f	// 2rad/s, about 0.3hz
#define BMP_CHECK_HZ	25
#define	DT		1.0f/BMP_CHECK_HZ

static int running = 0;

// interrupt handler to catch ctrl-c
static void signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}

int main()
{
	rc_bmp_data_t data;
	double filtered_alt;
	rc_filter_t lowpass = RC_FILTER_INITIALIZER;

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, signal_handler);
	running = 1;

	// create the lowpass filter
	if(rc_filter_butterworth_lowpass(&lowpass,ORDER, DT, CUTOFF_FREQ)) return -1;

	// init barometer and read in first data
	if(rc_bmp_init(OVERSAMPLE, INTERNAL_FILTER))	return -1;
	if(rc_bmp_read(&data)) return -1;

	// prefill low pass filter
	rc_filter_prefill_inputs(&lowpass, data.alt_m);
	rc_filter_prefill_outputs(&lowpass, data.alt_m);

	// print a header
	printf("\n");
	printf("  temp  |");
	printf(" pressure  |");
	printf(" altitude |");
	printf(" filtered |");
	printf("\n");

	//now just wait, print_data will run
	while(running){
		rc_usleep(1000000/BMP_CHECK_HZ);

		// perform the i2c reads to the sensor, on bad read just try later
		if(rc_bmp_read(&data)) continue;

		// if we got here, new data was read and is ready to be accessed.
		// these are very fast function calls and don't actually use i2c
		filtered_alt = rc_filter_march(&lowpass,data.alt_m);

		printf("\r");
		printf("%6.2lfC |", data.temp_c);
		printf("%7.2lfkpa |", data.pressure_pa/1000.0);
		printf("%8.2lfm |", data.alt_m);
		printf("%8.2lfm |", filtered_alt);
		fflush(stdout);
	}
	printf("\n");

	rc_bmp_power_off();
	return 0;
}

