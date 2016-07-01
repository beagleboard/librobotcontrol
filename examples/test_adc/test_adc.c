/*******************************************************************************
* test_adc.c
*
* James Strawson 2016
* prints voltages read by all adc channels
*******************************************************************************/

#include <useful_includes.h>
#include <robotics_cape.h>

int main(){
	int i;
		
	initialize_cape();
	
	printf(" adc_0 |");
	printf(" adc_1 |");
	printf(" adc_2 |");
	printf(" adc_3 |");
	printf("DC_Jack|");
	printf("Battery|");
	
	printf("\n");
	

	while(get_state()!=EXITING){
		printf("\r");
		//print all channels
		for(i=0;i<4;i++){
			printf("%6.2f |", get_adc_volt(i));
		}
		printf("%6.2f |", get_dc_jack_voltage());
		printf("%6.2f |", get_battery_voltage());
		fflush(stdout);
		usleep(100000);
	}
	cleanup_cape();
	return 0;
}