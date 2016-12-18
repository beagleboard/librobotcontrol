/*******************************************************************************
* test_filters.c
*
* This demonstrates the use of the discrete time SISO filters. It sets up three
* filters, a complentary low & high pass filter along with an integrator.
* It varies a common input u from 0 to 1 through time and show the output of 
* each filter. It also displays the sum of the complementary high and low pass
* filters to demonstrate how they sum to 1
*******************************************************************************/

#include "../../libraries/roboticscape-usefulincludes.h"
#include "../../libraries/roboticscape.h"

#define SAMPLE_RATE		50
#define TIME_CONSTANT	2.0

int main(){
	d_filter_t low_pass, high_pass, integrator;
	const double dt = 1.0/SAMPLE_RATE;
	double lp,hp,i, u = 0;
	int counter = 0;

	printf("\nInitializing Filters\n\n");
	printf("  Sample Rate: %dhz ", SAMPLE_RATE);
	printf(" Time Constant: %5.2f ", TIME_CONSTANT);
	printf(" dt: %6.2f \n\n", dt);
	
	low_pass   = create_first_order_lowpass(dt, TIME_CONSTANT);
	high_pass  = create_first_order_highpass(dt, TIME_CONSTANT);
	integrator = create_integrator(dt);

	reset_filter(&low_pass);
	reset_filter(&high_pass);
	reset_filter(&integrator);

	// print header
	printf("  input u |");
	printf("  lowpass |");
	printf(" highpass |");
	printf("complement|");
	printf("integrator|");
	printf("\n");

	// Keep Running until program state changes to EXITING
	u=1;
	while(rc_get_state() != EXITING){
		// march all filters one step forward with u as the common input.
		// new outputs saved as lp,hp,and i. complement is lp+hp
		lp = march_filter(&low_pass, u);
		hp = march_filter(&high_pass, u);
		i  = march_filter(&integrator, u);
		
		printf("\r");
		printf("%7.2f   |", u);
		printf("%7.2f   |", lp);
		printf("%7.2f   |", hp);
		printf("%7.2f   |", lp+hp);
		printf("%7.2f   |", i);
		fflush(stdout);
		
		// toggle u between 0 and 1 every 10 seconds
		counter++;
		if(counter >= SAMPLE_RATE*10){
			counter = 0;
			if(u>0) u = 0;
			else u = 1;
		}
		
		// sleep enough for a rough timed loop
		usleep(1000000/SAMPLE_RATE);
	}
	
	return 0;
}
