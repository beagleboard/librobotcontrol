/*******************************************************************************
* rc_test_adc.c
*
* James Strawson 2016
* prints voltages read by all adc channels
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

int main(){
	int i;

	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}

	printf(" adc_0 |");
	printf(" adc_1 |");
	printf(" adc_2 |");
	printf(" adc_3 |");
	printf("DC_Jack|");
	printf("Battery|");
	printf("\n");

	while(rc_get_state()!=EXITING){
		printf("\r");
		//print all channels
		for(i=0;i<4;i++){
			printf("%6.2f |", rc_adc_volt(i));
		}
		printf("%6.2f |", rc_dc_jack_voltage());
		printf("%6.2f |", rc_battery_voltage());
		fflush(stdout);
		rc_usleep(100000);
	}
	rc_cleanup();
	return 0;
}
