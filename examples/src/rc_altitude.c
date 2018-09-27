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
#include <rc/math/kalman.h>
#include <rc/math/filter.h>
#include <rc/math/quaternion.h>
#include <rc/time.h>
#include <rc/bmp.h>
#include <rc/mpu.h>


#define Nx 3
#define Ny 1
#define Nu 1
#define SAMPLE_RATE	200	// hz
#define	DT		(1.0/SAMPLE_RATE)
#define ACCEL_LP_TC	20*DT	// fast LP filter for accel
#define PRINT_HZ	10
#define BMP_RATE_DIV	10	// optionally sample bmp less frequently than mpu

static int running = 0;
static rc_mpu_data_t mpu_data;
static rc_bmp_data_t bmp_data;
static rc_kalman_t kf = RC_KALMAN_INITIALIZER;
static rc_vector_t u = RC_VECTOR_INITIALIZER;
static rc_vector_t y = RC_VECTOR_INITIALIZER;
static rc_filter_t acc_lp = RC_FILTER_INITIALIZER;


// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}


static void __dmp_handler(void)
{
	int i;
	double accel_vec[3];
	static int bmp_sample_counter = 0;

	// make copy of acceleration reading before rotating
	for(i=0;i<3;i++) accel_vec[i]=mpu_data.accel[i];
	// rotate accel vector
	rc_quaternion_rotate_vector_array(accel_vec,mpu_data.dmp_quat);

	// do first-run filter setup
	if(kf.step==0){
		kf.x_est.d[0] = bmp_data.alt_m;
		rc_filter_prefill_inputs(&acc_lp, accel_vec[2]-9.80665);
		rc_filter_prefill_outputs(&acc_lp, accel_vec[2]-9.80665);
	}

	// calculate acceleration and smooth it just a tad
	rc_filter_march(&acc_lp, accel_vec[2]-9.80665);
	u.d[0] = acc_lp.newest_output;

	// don't bother filtering Barometer, kalman will deal with that
	y.d[0] = bmp_data.alt_m;
	if(rc_kalman_update_lin(&kf, u, y)) running=0;

	// now check if we need to sample BMP this loop
	bmp_sample_counter++;
	if(bmp_sample_counter>=BMP_RATE_DIV){
		// perform the i2c reads to the sensor, on bad read just try later
		if(rc_bmp_read(&bmp_data)) return;
		bmp_sample_counter=0;
	}

	return;
}



int main(void)
{
	rc_mpu_config_t mpu_conf;
	rc_matrix_t F = RC_MATRIX_INITIALIZER;
	rc_matrix_t G = RC_MATRIX_INITIALIZER;
	rc_matrix_t H = RC_MATRIX_INITIALIZER;
	rc_matrix_t Q = RC_MATRIX_INITIALIZER;
	rc_matrix_t R = RC_MATRIX_INITIALIZER;
	rc_matrix_t Pi = RC_MATRIX_INITIALIZER;

	// allocate appropirate memory for system
	rc_matrix_zeros(&F, Nx, Nx);
	rc_matrix_zeros(&G, Nx, Nu);
	rc_matrix_zeros(&H, Ny, Nx);
	rc_matrix_zeros(&Q, Nx, Nx);
	rc_matrix_zeros(&R, Ny, Ny);
	rc_matrix_zeros(&Pi, Nx, Nx);
	rc_vector_zeros(&u, Nu);
	rc_vector_zeros(&y, Ny);

	// define system -DT; // accel bias
	F.d[0][0] = 1.0;
	F.d[0][1] = DT;
	F.d[0][2] = 0.0;
	F.d[1][0] = 0.0;
	F.d[1][1] = 1.0;
	F.d[1][2] = -DT; // subtract accel bias
	F.d[2][0] = 0.0;
	F.d[2][1] = 0.0;
	F.d[2][2] = 1.0; // accel bias state

	G.d[0][0] = 0.5*DT*DT;
	G.d[0][1] = DT;
	G.d[0][2] = 0.0;

	H.d[0][0] = 1.0;
	H.d[0][1] = 0.0;
	H.d[0][2] = 0.0;

	// covariance matrices
	Q.d[0][0] = 0.000000001;
	Q.d[1][1] = 0.000000001;
	Q.d[2][2] = 0.0001; // don't want bias to change too quickly
	R.d[0][0] = 1000000.0;

	// initial P, cloned from converged P while running
	Pi.d[0][0] = 1258.69;
	Pi.d[0][1] = 158.6114;
	Pi.d[0][2] = -9.9937;
	Pi.d[1][0] = 158.6114;
	Pi.d[1][1] = 29.9870;
	Pi.d[1][2] = -2.5191;
	Pi.d[2][0] = -9.9937;
	Pi.d[2][1] = -2.5191;
	Pi.d[2][2] = 0.3174;

	// initialize the kalman filter
	if(rc_kalman_alloc_lin(&kf,F,G,H,Q,R,Pi)==-1) return -1;
	// initialize the little LP filter to take out accel noise
	if(rc_filter_first_order_lowpass(&acc_lp, DT, ACCEL_LP_TC)) return -1;


	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running = 1;

	// init barometer and read in first data
	printf("initializing barometer\n");
	if(rc_bmp_init(BMP_OVERSAMPLE_16, BMP_FILTER_16)) return -1;
	if(rc_bmp_read(&bmp_data)) return -1;

	// init DMP
	printf("initializing DMP\n");
	mpu_conf = rc_mpu_default_config();
	mpu_conf.dmp_sample_rate = SAMPLE_RATE;
	mpu_conf.dmp_fetch_accel_gyro = 1;
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_conf)) return -1;

	// wait for dmp to settle then start filter callback
	printf("waiting for sensors to settle");
	fflush(stdout);
	rc_usleep(3000000);
	rc_mpu_set_dmp_callback(__dmp_handler);

	// print a header
	printf("\r\n");
	printf(" altitude |");
	printf("  velocity |");
	printf(" accel_bias |");
	printf(" alt (bmp) |");
	printf(" vert_accel |");
	printf("\n");

	//now just wait, print_data will run
	while(running){
		rc_usleep(1000000/PRINT_HZ);

		printf("\r");
		printf("%8.2fm |", kf.x_est.d[0]);
		printf("%7.2fm/s |", kf.x_est.d[1]);
		printf("%7.2fm/s^2|", kf.x_est.d[2]);
		printf("%9.2fm |", bmp_data.alt_m);
		printf("%7.2fm/s^2|", acc_lp.newest_output);

		fflush(stdout);
	}
	printf("\n");

	rc_mpu_power_off();
	rc_bmp_power_off();
	return 0;
}

