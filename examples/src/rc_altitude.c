/**
 * @file rc_altitude.c
 * @example    rc_altitude
 *
 * This serves as an example of how to read the barometer and IMU together to
 * estimate altitude
 *
 * @author     James Strawson
 * @date       3/14/2018
 */

#include <stdio.h>
#include <signal.h>
#include <math.h> // for M_PI
#include <rc/math/filter.h>
#include <rc/math/quaternion.h>
#include <rc/time.h>
#include <rc/bmp.h>
#include <rc/mpu.h>


#define ALT_FITLER_FREQ	0.1	// hz
#define ALT_FILTER_DAMP	1.0	// filter damping ratio
#define SAMPLE_RATE	200	// hz
#define	DT		1.0f/SAMPLE_RATE
#define PRINT_HZ	30
#define BMP_RATE_DIV	4	// only sample bmp every 4th DMP sample

int running;
float altitude, velocity, last_alt;
rc_mpu_data_t mpu_data;
rc_bmp_data_t bmp_data;
rc_filter_t lp_filter, hp_filter;


// interrupt handler to catch ctrl-c
void signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}


void dmp_handler()
{
	int i;
	float accel_vec[3];
	static int bmp_sample_counter = 0;

	// record last altitude for velocity calc
	last_alt = altitude;

	// make copy of acceleration reading before rotating
	for(i=0;i<3;i++) accel_vec[i]=mpu_data.accel[i];
	// rotate accel vector
	rc_quaternion_rotate_vector_array(accel_vec,mpu_data.dmp_quat);
	// run complementary filters, integrator is alredy built into hp_filter
	rc_filter_march(&hp_filter, accel_vec[2]-9.80665);
	rc_filter_march(&lp_filter, bmp_data.alt_m);
	// sum complementary filters
	altitude = hp_filter.newest_output + lp_filter.newest_output;

	// calc velocity
	velocity = (altitude-last_alt)/DT;

	// now check if we need to sample BMP this loop
	bmp_sample_counter++;
	if(bmp_sample_counter>=BMP_RATE_DIV){
		// perform the i2c reads to the sensor, on bad read just try later
		if(rc_bmp_read(&bmp_data)) return;
		bmp_sample_counter=0;
	}
	return;
}



int main()
{
	rc_mpu_config_t mpu_conf;
	rc_filter_t tmp1_filter = rc_filter_empty();
	rc_filter_t tmp2_filter = rc_filter_empty();
	hp_filter = rc_filter_empty();
	lp_filter = rc_filter_empty();

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, signal_handler);
	running = 1;

	// create the filters, accel is the combination of a double integrator
	// (to get from accel to position) and the complementary filter
	rc_filter_third_order_complement(&lp_filter,&tmp1_filter, 2.0*M_PI*SAMPLE_RATE, ALT_FILTER_DAMP, DT);
	rc_filter_double_integrator(&tmp2_filter, DT);
	rc_filter_multiply(tmp1_filter, tmp2_filter,&hp_filter);
	rc_filter_free(&tmp1_filter);
	rc_filter_free(&tmp2_filter);

	// init barometer and read in first data
	if(rc_bmp_init(BMP_OVERSAMPLE_16, BMP_FILTER_OFF)) return -1;
	if(rc_bmp_read(&bmp_data)) return -1;

	// init DMP
	mpu_conf = rc_mpu_default_config();
	mpu_conf.dmp_sample_rate = SAMPLE_RATE;
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_conf)) return -1;


	// do first-run filter setup
	rc_filter_prefill_inputs(&lp_filter, bmp_data.alt_m);
	rc_filter_prefill_outputs(&lp_filter, bmp_data.alt_m);
	altitude = bmp_data.alt_m;
	last_alt = bmp_data.alt_m;

	// wait for dmp to settle then start filter callback
	printf("waiting for sensors to settle");
	rc_usleep(3000000);
	rc_mpu_set_dmp_callback(dmp_handler);

	// print a header
	printf("\r\n");
	printf("  altitude |");
	printf("  velocity |");
	printf(" alt (bmp) |");
	printf(" alt (accel) |");
	printf("\n");

	//now just wait, print_data will run
	while(running){
		rc_usleep(1000000/PRINT_HZ);
		printf("\r");
		printf("%7.3fm |", altitude);
		printf("%5.2fm/s |", velocity);
		printf("%7.3fm |", lp_filter.newest_output);
		printf("%7.3fm |", hp_filter.newest_output);
		fflush(stdout);
	}
	printf("\n");

	rc_mpu_power_off();
	rc_bmp_power_off();
	return 0;
}

