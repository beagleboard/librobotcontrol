/*
 * @file kalman.h
 *
 * @brief      Kalman filter implementation
 * @date       April 2018
 * @author     Eric Nauli Sihite & James Strawson
 */

#include <stdio.h>
#include <rc/math/algebra.h>
#include <rc/math/kalman.h>

rc_kalman_t rc_kalman_empty()
{
	rc_kalman_t kf;
	// set struct to zeros
	kf.P = rc_matrix_empty();
	kf.Q = rc_matrix_empty();
	kf.R = rc_matrix_empty();
	kf.F = rc_matrix_empty();
	kf.H = rc_matrix_empty();

	kf.x_est = rc_vector_empty();
	kf.x_pre = rc_vector_empty();

	kf.P_init = 0.0;
	kf.initialized = 0;
	kf.step = 0;
	return kf;
}


int rc_kalman_alloc(rc_kalman_t* kf, rc_matrix_t F, rc_matrix_t G, rc_matrix_t H, rc_matrix_t Q, rc_matrix_t R, double Pi)
{
	int Nx;

	// sanity checks
	if(kf==NULL){
		fprintf(stderr, "ERROR in rc_kalman_alloc, received NULL pointer\n");
		return -1;
	}
	if(F.initialized!=1 || H.initialized!=1){
		fprintf(stderr, "ERROR in rc_kalman_alloc, received uinitialized F or H\n");
		return -1;
	}
	if(Q.initialized!=1 || R.initialized!=1){
		fprintf(stderr, "ERROR in rc_kalman_alloc, received initialized P or Q\n");
		return -1;
	}
	if(F.rows != F.cols){
		fprintf(stderr, "ERROR in rc_kalman_alloc, F must be square\n");
		return -1;
	}
	if(H.cols != F.cols){
		fprintf(stderr, "ERROR in rc_kalman_alloc, F and H must have same number of columns\n");
		return -1;
	}
	if(G.rows != F.rows){
		fprintf(stderr, "ERROR in rc_kalman_alloc, F and G must have same number of rows\n");
		return -1;
	}

	// free existing memory, this also zero's out the struct
	rc_kalman_free(kf);

	// Matrices
	Nx = F.cols;
	rc_matrix_identity(&kf->P, Nx);
	rc_matrix_times_scalar(&kf->P, kf->P_init);
	rc_matrix_duplicate(Q, &kf->Q);
	rc_matrix_duplicate(R, &kf->R);
	rc_matrix_duplicate(F, &kf->F);
	rc_matrix_duplicate(G, &kf->G);
	rc_matrix_duplicate(H, &kf->H);

	// Vectors
	rc_vector_zeros(&kf->x_est, Nx);
	rc_vector_zeros(&kf->x_pre, Nx);

	return 0;
}


int rc_kalman_free(rc_kalman_t* kf)
{
	// sanity checks
	if(kf==NULL){
		fprintf(stderr, "ERROR in rc_kalman_free, received NULL pointer\n");
		return -1;
	}
	rc_matrix_free(&kf->P);
	rc_matrix_free(&kf->Q);
	rc_matrix_free(&kf->R);
	rc_matrix_free(&kf->F);
	rc_matrix_free(&kf->H);

	rc_vector_free(&kf->x_est);
	rc_vector_free(&kf->x_pre);

	*kf = rc_kalman_empty();
	return 0;
}


int rc_kalman_reset(rc_kalman_t* kf)
{
	int i;

	// sanity checks
	if(kf==NULL){
		fprintf(stderr, "ERROR in rc_kalman_reset, received NULL pointer\n");
		return -1;
	}
	if(kf->initialized !=1){
		fprintf(stderr, "ERROR in rc_kalman_reset, kf uninitialized\n");
		return -1;
	}

	// set P to I*P_init
	rc_matrix_zero_out(&kf->P);
	for(i=0; i<kf->P.rows; i++) kf->P.d[i][i]=kf->P_init;

	rc_vector_zero_out(&kf->x_est);
	rc_vector_zero_out(&kf->x_pre);
	kf->step = 0;

	return 0;
}

int rc_kalman_prediction(rc_kalman_t* kf, rc_vector_t u)
{
	rc_matrix_t newP = rc_matrix_empty();
	rc_matrix_t FT = rc_matrix_empty();
	rc_vector_t tmp1 = rc_vector_empty();
	rc_vector_t tmp2 = rc_vector_empty();

	// sanity checks
	if(kf==NULL){
		fprintf(stderr, "ERROR in rc_kalman_prediction, received NULL pointer\n");
		return -1;
	}
	if(kf->initialized !=1){
		fprintf(stderr, "ERROR in rc_kalman_prediction, kf uninitialized\n");
		return -1;
	}

	// x_pre = x[k|k-1] = F*x[k-1|k-1] +  G*u[k]
	rc_matrix_times_col_vec(kf->F, kf->x_est, &tmp1);
	rc_matrix_times_col_vec(kf->G, u, &tmp2);
	rc_vector_sum(tmp1, tmp2, &kf->x_pre);

	// P[k+1|k] = F*P*F^T + Q
	rc_matrix_multiply(kf->F, kf->P, &newP);	// P_new = F*P_old
	rc_matrix_transpose(kf->F, &FT);
	rc_matrix_right_multiply_inplace(&newP, FT);	// P = F*P*F^T
	rc_matrix_add(newP, kf->Q, &kf->P);		// P = F*P*F^T + Q
	rc_matrix_symmetrize(&kf->P);			// Force symmetric P

	// cleanup
	rc_vector_free(&tmp1);
	rc_vector_free(&tmp2);
	rc_matrix_free(&FT);
	rc_matrix_free(&newP);
	return 0;
}


int rc_kalman_update(rc_kalman_t* kf, rc_vector_t y, rc_vector_t h)
{
	// Initialize temporary variables
	rc_matrix_t S = rc_matrix_empty();

	rc_vector_t temp = rc_vector_empty();
	rc_matrix_t K = rc_matrix_empty(); // kalman gain

	// sanity checks
	if(kf==NULL){
		fprintf(stderr, "ERROR in rc_kalman_update, received NULL pointer\n");
		return -1;
	}
	if(kf->initialized !=1){
		fprintf(stderr, "ERROR in rc_kalman_update, kf uninitialized\n");
		return -1;
	}
	if(y.initialized!=1 || h.initialized!=1){
		fprintf(stderr, "ERROR in rc_kalman_update, received uninitialized vector\n");
		return -1;
	}

	// S = H*P*H^T + R
	// Calculate H^T, borrow S for H^T
	rc_matrix_transpose(kf->H, &S);			// S = H^T
	// Calculate a part of K in advance before we modify S = H^T
	rc_matrix_multiply(kf->P, S, &K);		// K = P*(H^T)
	rc_matrix_left_multiply_inplace(kf->P, &S);	// S = P*H^T
	rc_matrix_left_multiply_inplace(kf->H, &S);	// S = H*(P*H^T)
	rc_matrix_add_inplace(&S, kf->R);		// S = H*P*H^T + R

	// K = P*(H^T)*(S^-1)
	rc_algebra_invert_matrix_inplace(&S);		// S2^(-1) = S^(-1)
	rc_matrix_right_multiply_inplace(&K, S);	// K = (P*H^T)*(S^-1)

	// x[k|k] = x[k|k-1] + K[k]*y[k]
	rc_matrix_times_col_vec(K, y, &temp);		// temp = K*y
	rc_vector_sum(kf->x_pre, temp, &kf->x_est);	// x_est = x + K*y

	// P[k|k] = (I - L*H)*P = P - L*H*P, reuse the matrix S.
	rc_matrix_multiply(kf->H, kf->P, &S);		// S = H*P
	rc_matrix_left_multiply_inplace(K, &S);		// S = K*(H*P)
	rc_matrix_subtract_inplace(&kf->P, S);		// P = P - K*H*P
	rc_matrix_symmetrize(&kf->P);			// Force symmetric P

	// Free memory
	rc_matrix_free(&K);
	rc_matrix_free(&S);
	rc_vector_free(&temp);

	kf->step++;
	return 0;
}

