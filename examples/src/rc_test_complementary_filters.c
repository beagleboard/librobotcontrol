/**
 * @example    rc_test_complementary_filters.c
 *
 * @brief      Demonstrates the use of first and third order symmetric
 *             complementary filters.
 *
 *             It sets up four filters and varies a common input u from 0 to 1
 *             through time and show the output of each filter. It also displays
 *             the sums of the two lp/hp pairs to demonstrate that they are in
 *             face complementary.
 *
 * @author     James Strawson
 * @date       4/21/2018
 */

#include <signal.h>
#include <stdio.h>
#include <math.h> // for M_PI
#include <rc/math.h>
#include <rc/time.h>

#define SAMPLE_RATE	50
#define TIME_CONSTANT	2.0
#define DAMP		1.0

static int running = 0;

// interrupt handler to catch ctrl-c
static void signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}

int main()
{
	rc_filter_t lp_first = RC_FILTER_INITIALIZER;
	rc_filter_t hp_first = RC_FILTER_INITIALIZER;
	rc_filter_t lp_third = RC_FILTER_INITIALIZER;
	rc_filter_t hp_third = RC_FILTER_INITIALIZER;

	const double dt = 1.0/SAMPLE_RATE;
	double lpf,hpf,lpt,hpt,u;
	int counter = 0;

	printf("\nSample Rate: %dhz\n", SAMPLE_RATE);
	printf("Time Constant: %5.2f\n", TIME_CONSTANT);

	rc_filter_first_order_lowpass(&lp_first, dt, TIME_CONSTANT);
	rc_filter_first_order_highpass(&hp_first, dt, TIME_CONSTANT);
	rc_filter_third_order_complement(&lp_third,&hp_third, 2.0*M_PI/TIME_CONSTANT, DAMP, dt);

	printf("\nLow Pass:\n");
	rc_filter_print(lp_first);
	printf("\nHigh Pass:\n");
	rc_filter_print(hp_first);
	printf("\nLow Pass Third Order Complement:\n");
	rc_filter_print(lp_third);
	printf("\nHigh Pass Third Order Complement:\n");
	rc_filter_print(hp_third);

	printf("\n\n");

	// print header
	printf("  input u |");
	printf(" lp_first |");
	printf(" hp_first |");
	printf("    sum   |");
	printf(" lp_third |");
	printf(" hp_third |");
	printf("    sum   |");
	printf("\n");

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, signal_handler);
	running = 1;

	// start filter input at 1, the loop will toggle this later
	u=1.0;

	// Keep Running until program state changes to 0
	while(running){
		// march all filters one step forward with u as the common input.
		// new outputs saved as lp,hp,and i. complement is lp+hp
		lpf = rc_filter_march(&lp_first, u);
		hpf = rc_filter_march(&hp_first, u);
		lpt = rc_filter_march(&lp_third, u);
		hpt = rc_filter_march(&hp_third, u);

		printf("\r");
		printf("%8.3lf  |", u);
		printf("%8.3lf  |", lpf);
		printf("%8.3lf  |", hpf);
		printf("%8.3lf  |", lpf+hpf);
		printf("%8.3lf  |", lpt);
		printf("%8.3lf  |", hpt);
		printf("%8.3lf  |", lpt+hpt);
		fflush(stdout);

		// toggle u between 0 and 1 every 10 seconds
		counter++;
		if(counter >= SAMPLE_RATE*10){
			counter = 0.0;
			if(u>0.0) u = 0.0;
			else u = 1.0;
		}

		// sleep enough for a rough timed loop
		rc_usleep(1000000/SAMPLE_RATE);
	}

	printf("\n");
	rc_filter_free(&lp_first);
	rc_filter_free(&hp_first);
	rc_filter_free(&lp_third);
	rc_filter_free(&hp_third);
	return 0;
}
