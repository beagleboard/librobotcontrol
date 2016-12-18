/*******************************************************************************
* test_barometer.c
*
* James Strawson 2016
* This serves as an example of how to read the barometer.
*******************************************************************************/

#include "../../libraries/roboticscape-usefulincludes.h"
#include "../../libraries/roboticscape.h"

// choice of 1,2,4,8,16 oversampling. Here we use 16 and sample at 25hz which
// is close to the update rate specified in robotics_cape.h for that oversample.
#define OVERSAMPLE  BMP_OVERSAMPLE_16
// choice of OFF, 2, 4, 8, 16 filter constants. Here we turn off the filter and 
// opt to use our own 2nd order filter instead.
#define INTERNAL_FILTER	BMP_FILTER_OFF

// our own low pass filter
#define ORDER 			2
#define CUTOFF_FREQ		2	// 2rad/s, about 0.3hz
#define BMP_CHECK_HZ	25
#define	DT 				1.0/BMP_CHECK_HZ

int main(){
	double temp, pressure, altitude, filtered_alt;
	d_filter_t lowpass;

	// set up cape and barometer
	if(initialize_roboticscape()){
		printf("ERROR: failed to initialize_cape\n");
		return -1;
	}
	if(initialize_barometer(OVERSAMPLE, INTERNAL_FILTER)<0){
		printf("initialize_barometer failed\n");
		return -1;
	}
	
	// create the lowpass filter and prefill with current altitude
	lowpass = create_butterworth_lowpass(ORDER, DT, CUTOFF_FREQ);
	altitude = bmp_get_altitude_m();
	prefill_filter_inputs(&lowpass, altitude);
	prefill_filter_outputs(&lowpass, altitude);

	// print a header
	printf("\n");
	printf("  temp  |");
	printf(" pressure  |");
	printf(" altitude |");
	printf(" filtered |");
	printf("\n");
	
	//now just wait, print_data will run
	while (rc_get_state()!=EXITING) {
		usleep(1000000/BMP_CHECK_HZ);
		
		// perform the i2c reads to the sensor, this takes a bit of time
		if(read_barometer()<0){
			printf("\rERROR: Can't read Barometer");
			fflush(stdout);
			continue;
		}

		// if we got here, new data was read and is ready to be accessed.
		// these are very fast function calls and don't actually use i2c
		temp = bmp_get_temperature_c();
		pressure = bmp_get_pressure_pa();
		altitude = bmp_get_altitude_m();
		filtered_alt = march_filter(&lowpass,altitude);

		printf("\r");
		printf("%6.2fC |", temp);
		printf("%7.2fkpa |", pressure/1000.0);
		printf("%8.2fm |", altitude);
		printf("%8.2fm |", filtered_alt);
														
		fflush(stdout);
		
	}

	power_off_barometer();
	cleanup_roboticscape();
	return 0;
}

