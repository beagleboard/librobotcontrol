/**
 * @file rc_test_adc.c
 * @example    rc_test_adc
 *
 * prints voltages read by all adc channels
 *
 * @author     James Strawson
 * @date       1/29/2018
 */


#include <stdio.h>
#include <rc/adc.h>
#include <rc/time.h>


int main()
{
	int i;

	// initialize hardware first
	if(rc_adc_init()){
		fprintf(stderr,"ERROR: failed to run rc_init_adc()\n");
		return -1;
	}

	printf(" adc_0 |");
	printf(" adc_1 |");
	printf(" adc_2 |");
	printf(" adc_3 |");
	printf("DC_Jack|");
	printf("Battery|");
	printf("\n");

	// run untill the signal handler sets state to EXITING
	while(1){
		printf("\r");
		//print all channels
		for(i=0;i<4;i++){
			printf("%6.2f |", rc_adc_read_volt(i));
		}
		printf("%6.2f |", rc_adc_dc_jack());
		printf("%6.2f |", rc_adc_batt());
		fflush(stdout);
		rc_usleep(100000);
	}

	rc_adc_cleanup();
	return 0;
}
