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


#define ALT_FITLER_FREQ	0.15	// hz
#define ALT_FILTER_DAMP	1.0	// filter damping ratio
#define SAMPLE_RATE	200	// hz
#define	DT		(1.0/SAMPLE_RATE)
#define PRINT_HZ	10
#define BMP_RATE_DIV	1	// optionally sample bmp less frequently than mpu

int running;
double altitude, velocity, last_alt;
rc_mpu_data_t mpu_data;
rc_bmp_data_t bmp_data;
rc_filter_t lp_filter, hp_filter;

double accel;


// interrupt handler to catch ctrl-c
void signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}


void dmp_handler()
{
	int i;
	double accel_vec[3];
	static int bmp_sample_counter = 0;

	// record last altitude for velocity calc
	last_alt = altitude;

	// make copy of acceleration reading before rotating
	for(i=0;i<3;i++) accel_vec[i]=mpu_data.accel[i];
	// rotate accel vector
	rc_quaternion_rotate_vector_array(accel_vec,mpu_data.dmp_quat);

	// do first-run filter setup
	if(hp_filter.step==0){
		rc_filter_prefill_inputs(&lp_filter, bmp_data.alt_m);
		rc_filter_prefill_outputs(&lp_filter, bmp_data.alt_m);
		rc_filter_prefill_inputs(&hp_filter, accel_vec[2]-9.80665);
		altitude = bmp_data.alt_m;
		last_alt = bmp_data.alt_m;
	}

	// run complementary filters, integrator is alredy built into hp_filter
	accel=accel_vec[2]-9.80665;
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

	/*
	printf("bmp: %7.3f raw accel %7.3f %7.3f %7.3f rotated %7.3f %7.3f %7.3f\n",
		bmp_data.alt_m, mpu_data.accel[0], mpu_data.accel[1], mpu_data.accel[2],
		accel_vec[0], accel_vec[1], accel_vec[2]);
	*/

	return;
}



int main()
{
	rc_mpu_config_t mpu_conf;
	rc_filter_t tmp1 = rc_filter_empty();
	rc_filter_t tmp2 = rc_filter_empty();
	hp_filter = rc_filter_empty();
	lp_filter = rc_filter_empty();

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, signal_handler);
	running = 1;

	// create the filters, accel is the combination of a double integrator
	// (to get from accel to position) and the complementary filter
	if(rc_filter_third_order_complement(&lp_filter,&tmp1, 2.0*M_PI*ALT_FITLER_FREQ, ALT_FILTER_DAMP, DT)) return -1;
	if(rc_filter_double_integrator(&tmp2, DT)) return -1;
	if(rc_filter_multiply(tmp1, tmp2, &hp_filter)) return -1;
	if(rc_filter_normalize(&hp_filter)) return -1;


	// print filters
	printf("lp_filter:\n");
	rc_filter_print(lp_filter);
	printf("hp_filter:\n");
	rc_filter_print(tmp1);
	printf("hp_filter with integrator:\n");
	rc_filter_print(hp_filter);

	rc_filter_free(&tmp1);
	rc_filter_free(&tmp2);

	// init barometer and read in first data
	if(rc_bmp_init(BMP_OVERSAMPLE_16, BMP_FILTER_16)) return -1;
	if(rc_bmp_read(&bmp_data)) return -1;

	// init DMP
	mpu_conf = rc_mpu_default_config();
	mpu_conf.dmp_sample_rate = SAMPLE_RATE;
	mpu_conf.dmp_fetch_accel_gyro = 1;
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_conf)) return -1;


	// wait for dmp to settle then start filter callback
	printf("waiting for sensors to settle");
	fflush(stdout);
	rc_usleep(3000000);
	rc_mpu_set_dmp_callback(dmp_handler);

	// print a header
	printf("\r\n");
	printf(" altitude |");
	printf("  velocity |");
	printf(" alt (bmp) |");
	printf(" alt (acl) |");
	printf("\n");

	//now just wait, print_data will run
	while(running){
		rc_usleep(1000000/PRINT_HZ);
		printf("\r");
		printf("%8.2fm |", altitude);
		printf("%7.1fm/s |", velocity);
		printf("%9.2fm |", lp_filter.newest_output);
		printf("%9.2fm |", hp_filter.newest_output);
		// printf(" accel %7.3fm |", accel);
		fflush(stdout);
	}
	printf("\n");

	rc_mpu_power_off();
	rc_bmp_power_off();
	return 0;
}

