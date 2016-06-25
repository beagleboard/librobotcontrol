/*******************************************************************************
* test_filters.c
*
* This demonstrates the use of the discrete time SISO filters. It sets up three
* filters, a complentary low & high pass filter along with an integrator.
* It varies a common input u from 0 to 1 through time and show the output of 
* each filter. It also displays the sum of the complementary high and low pass
* filters to demonstrate how they sum to 1
*******************************************************************************/

#include <robotics_cape.h>
#include <useful_includes.h>

#define SAMPLE_RATE 	50
#define TIME_CONSTANT 	2.0

int main(){
	d_filter_t low_pass, high_pass, integrator;
	const float dt = 1.0/(float)SAMPLE_RATE;
	float lp,hp,i, u = 0;
	int counter = 0;

	printf("\nInitializing Filters\n\n");
	printf("  Sample Rate: %dhz ", SAMPLE_RATE);
	printf(" Time Constant: %5.2f ", TIME_CONSTANT);
	printf(" dt: %6.2f \n\n", dt);
	
	low_pass   = generateFirstOrderLowPass(dt, TIME_CONSTANT);
	high_pass  = generateFirstOrderHighPass(dt, TIME_CONSTANT);
	integrator = generateIntegrator(dt);

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
	prefill_filter_outputs(&low_pass,u);
	prefill_filter_inputs(&low_pass, u);
	while(get_state() != EXITING){
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
