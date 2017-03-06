/*******************************************************************************
* rc_test_filters.c
*
* This demonstrates the use of the discrete time SISO filters. It sets up three
* filters, a complentary low & high pass filter along with an integrator.
* It varies a common input u from 0 to 1 through time and show the output of 
* each filter. It also displays the sum of the complementary high and low pass
* filters to demonstrate how they sum to 1
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

#define SAMPLE_RATE		50
#define TIME_CONSTANT	2.0

int main(){
	rc_filter_t low_pass = rc_empty_filter();
	rc_filter_t high_pass = rc_empty_filter();
	rc_filter_t integrator = rc_empty_filter();
	rc_filter_t lp_butter = rc_empty_filter();
	rc_filter_t hp_butter = rc_empty_filter();

	const float dt = 1.0/SAMPLE_RATE;
	float lp,hp,i,u,lpb,hpb;
	int counter = 0;

	printf("\nSample Rate: %dhz\n", SAMPLE_RATE);
	printf("Time Constant: %5.2f\n", TIME_CONSTANT);
	
	rc_first_order_lowpass(&low_pass, dt, TIME_CONSTANT);
	rc_first_order_highpass(&high_pass, dt, TIME_CONSTANT);
	rc_integrator(&integrator, dt);
	rc_butterworth_lowpass(&lp_butter, 2, dt, 2.0*M_PI/TIME_CONSTANT);
	rc_butterworth_highpass(&hp_butter, 2, dt, 2.0*M_PI/TIME_CONSTANT);

	printf("\nLow Pass:\n");
	rc_print_filter(low_pass);
	printf("\nHigh Pass:\n");
	rc_print_filter(high_pass);
	printf("\nIntegrator:\n");
	rc_print_filter(integrator);
	printf("\nLow Pass Butterworth:\n");
	rc_print_filter(lp_butter);
	printf("\nHigh Pass Butterworth:\n");
	rc_print_filter(hp_butter);
	printf("\n\n");

	// print header
	printf("  input u |");
	printf("  lowpass |");
	printf(" highpass |");
	printf("complement|");
	printf("integrator|");
	printf(" lp_butter|");
	printf("hp_butter |");
	printf("\n");

	// Keep Running until program state changes to EXITING
	u=1.0;
	while(rc_get_state() != EXITING){
		// march all filters one step forward with u as the common input.
		// new outputs saved as lp,hp,and i. complement is lp+hp
		lp = rc_march_filter(&low_pass, u);
		hp = rc_march_filter(&high_pass, u);
		i  = rc_march_filter(&integrator, u);
		lpb = rc_march_filter(&lp_butter, u);
		hpb = rc_march_filter(&hp_butter, u);

		printf("\r");
		printf("%8.3f  |", u);
		printf("%8.3f  |", lp);
		printf("%8.3f  |", hp);
		printf("%8.3f  |", lp+hp);
		printf("%8.3f  |", i);
		printf("%8.3f  |", lpb);
		printf("%8.3f  |", hpb);
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

	return 0;
}
