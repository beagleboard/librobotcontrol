/**
 * @example    rc_test_kalman.c
 *
 * Tests the linear kalman filter in <rc/math/kalman.h> by simulating pushing a
 * mass of 1kg back and forth with a force of 1N. A noisy position sensor is
 * simulated as well as noisy control input u which just toggles back and forth
 * between +1 and -1.
 *
 * State equations:
 * - x = [position] = [ 1 , dt]x  +  [ 0.5*dt^2 ]u  +  w
 * -     [velocity]   [ 0 , 1 ]      [    dt     ]
 * - y = [pos_est]  = [ 1 ]x   +   w
 * -                  [ 0 ]
 *
 *
 * @author     James Strawson
 * @date       4/26/2018
 */


#include <stdio.h>
#include <signal.h>
#include <rc/math/kalman.h>
#include <rc/time.h>

#define Nx 2
#define Ny 1
#define Nu 1
#define DT 0.05
#define REVERSE_TIME 2.0

static int running = 1;

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}

int main()
{
	// declare variables
	int counter;

	rc_kalman_t kf	= RC_KALMAN_INITIALIZER;
	rc_matrix_t F	= RC_MATRIX_INITIALIZER;
	rc_matrix_t G	= RC_MATRIX_INITIALIZER;
	rc_matrix_t H	= RC_MATRIX_INITIALIZER;
	rc_matrix_t Q	= RC_MATRIX_INITIALIZER;
	rc_matrix_t R	= RC_MATRIX_INITIALIZER;
	rc_matrix_t Pi	= RC_MATRIX_INITIALIZER;
	rc_vector_t u	= RC_VECTOR_INITIALIZER;
	rc_vector_t y	= RC_VECTOR_INITIALIZER;

	// allocate appropriate memory for system
	rc_matrix_zeros(&F, Nx, Nx);
	rc_matrix_zeros(&G, Nx, Nu);
	rc_matrix_zeros(&H, Ny, Nx);
	rc_matrix_zeros(&Q, Nx, Nx);
	rc_matrix_zeros(&R, Ny, Ny);
	rc_matrix_zeros(&Pi, Nx, Nx);
	rc_vector_zeros(&u, Nu);
	rc_vector_zeros(&y, Ny);

	// define system
	F.d[0][0] = 1;
	F.d[0][1] = DT;
	F.d[1][0] = 0;
	F.d[1][1] = 1;
	G.d[0][0] = 0.5*DT*DT;
	G.d[0][1] = DT;
	H.d[0][0] = 1;
	H.d[0][1] = 0;

	// covariance matrices
	Q.d[0][0] = 0.1;
	Q.d[1][1] = 0.0001;
	R.d[0][0] = 1.0;

	// initial P
	Pi.d[0][0] = 0.0001;
	Pi.d[1][1] = 0.0001;


	if(rc_kalman_alloc_lin(&kf,F,G,H,Q,R,Pi)==-1) return -1;

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running = 1;
	counter = REVERSE_TIME/(DT*2.0);
	u.d[0]=1.0;
	while(running){
		// based on time, see if we should reverse u
		if(counter > REVERSE_TIME/DT){
			counter = 0;
			if(u.d[0]<1.0) u.d[0]=1.0;
			else u.d[0]=-1.0;
			// bump up y to see x_est track
			y.d[0]+=0.5;
		}

		// update filter
		if(rc_kalman_update_lin(&kf, u, y)) running=0;

		// print result
		printf("\rpos: %5.2f vel: %5.2f u: %5.2f y: %5.2f    ", kf.x_est.d[0], kf.x_est.d[1], u.d[0], y.d[0]);
		fflush(stdout);


		counter++;
		rc_usleep(DT*1000000);
	}
	printf("\n");

	rc_matrix_free(&F);
	rc_matrix_free(&G);
	rc_matrix_free(&H);
	rc_matrix_free(&Q);
	rc_matrix_free(&R);
	rc_matrix_free(&Pi);
	rc_vector_free(&u);
	rc_vector_free(&y);
	rc_kalman_free(&kf);
	return 0;
}
