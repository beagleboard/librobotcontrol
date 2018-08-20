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
#include "algebra_common.h"

rc_kalman_t rc_kalman_empty(void)
{
	rc_kalman_t kf = RC_KALMAN_INITIALIZER;
	return kf;
}


int rc_kalman_alloc_lin(rc_kalman_t* kf, rc_matrix_t F, rc_matrix_t G, rc_matrix_t H, rc_matrix_t Q, rc_matrix_t R, rc_matrix_t Pi)
{
	int Nx;

	// sanity checks
	if(kf==NULL){
		fprintf(stderr, "ERROR in rc_kalman_alloc_lin, received NULL pointer\n");
		return -1;
	}
	if(!F.initialized || !H.initialized){
		fprintf(stderr, "ERROR in rc_kalman_alloc, received uninitialized F or H\n");
		return -1;
	}
	if(!Q.initialized || !R.initialized){
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
	if(Q.rows != Q.cols){
		fprintf(stderr, "ERROR in rc_kalman_alloc_ekf, Q must be square\n");
		return -1;
	}
	if(R.rows != R.cols){
		fprintf(stderr, "ERROR in rc_kalman_alloc_ekf, R must be square\n");
		return -1;
	}

	// free existing memory, this also zero's out the struct
	if(rc_kalman_free(kf)==-1) return -1;

	// allocate memory
	Nx = F.cols;
	if(rc_matrix_duplicate(F, &kf->F)==-1) return -1;
	if(rc_matrix_duplicate(G, &kf->G)==-1) return -1;
	if(rc_matrix_duplicate(H, &kf->H)==-1) return -1;


	if(rc_matrix_duplicate(Q, &kf->Q)==-1) return -1;
	if(rc_matrix_duplicate(R, &kf->R)==-1) return -1;
	if(rc_matrix_duplicate(Pi, &kf->P)==-1) return -1;
	if(rc_matrix_duplicate(Pi, &kf->Pi)==-1) return -1;

	if(rc_vector_zeros(&kf->x_est, Nx)==-1) return -1;
	if(rc_vector_zeros(&kf->x_pre, Nx)==-1) return -1;
	kf->initialized = 1;
	return 0;
}

int rc_kalman_alloc_ekf(rc_kalman_t* kf, rc_matrix_t Q, rc_matrix_t R, rc_matrix_t Pi)
{
	// sanity checks
	if(kf==NULL){
		fprintf(stderr, "ERROR in rc_kalman_alloc_ekf, received NULL pointer\n");
		return -1;
	}
	if(!Q.initialized || !R.initialized || !Pi.initialized){
		fprintf(stderr, "ERROR in rc_kalman_alloc_ekf, received uninitialized matrix\n");
		return -1;
	}
	if(Q.rows != Q.cols){
		fprintf(stderr, "ERROR in rc_kalman_alloc_ekf, Q must be square\n");
		return -1;
	}
	if(R.rows != R.cols){
		fprintf(stderr, "ERROR in rc_kalman_alloc_ekf, R must be square\n");
		return -1;
	}

	// free existing memory, this also zero's out the struct
	rc_kalman_free(kf);

	// allocate memory
	rc_matrix_duplicate(Q, &kf->Q);
	rc_matrix_duplicate(R, &kf->R);
	rc_matrix_duplicate(Pi, &kf->Pi);
	rc_matrix_duplicate(Pi, &kf->P);
	rc_vector_zeros(&kf->x_est, Q.rows);
	rc_vector_zeros(&kf->x_pre, Q.rows);
	kf->initialized = 1;
	return 0;
}


int rc_kalman_free(rc_kalman_t* kf)
{
	rc_kalman_t new = RC_KALMAN_INITIALIZER;
	// sanity checks
	if(kf==NULL){
		fprintf(stderr, "ERROR in rc_kalman_free, received NULL pointer\n");
		return -1;
	}
	rc_matrix_free(&kf->F);
	rc_matrix_free(&kf->G);
	rc_matrix_free(&kf->H);

	rc_matrix_free(&kf->Q);
	rc_matrix_free(&kf->R);
	rc_matrix_free(&kf->P);
	rc_matrix_free(&kf->Pi);

	rc_vector_free(&kf->x_est);
	rc_vector_free(&kf->x_pre);

	*kf = new;
	return 0;
}


int rc_kalman_reset(rc_kalman_t* kf)
{
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
	rc_matrix_duplicate(kf->Pi, &kf->P);
	rc_vector_zero_out(&kf->x_est);
	rc_vector_zero_out(&kf->x_pre);
	kf->step = 0;
	return 0;
}

int rc_kalman_update_lin(rc_kalman_t* kf, rc_vector_t u, rc_vector_t y)
{
	rc_matrix_t L = RC_MATRIX_INITIALIZER;
	rc_matrix_t newP = RC_MATRIX_INITIALIZER;
	rc_matrix_t S = RC_MATRIX_INITIALIZER;
	rc_matrix_t FT = RC_MATRIX_INITIALIZER;
	rc_vector_t h = RC_VECTOR_INITIALIZER;
	rc_vector_t z = RC_VECTOR_INITIALIZER;
	rc_vector_t tmp1 = RC_VECTOR_INITIALIZER;
	rc_vector_t tmp2 = RC_VECTOR_INITIALIZER;

	// sanity checks
	if(unlikely(kf==NULL)){
		fprintf(stderr, "ERROR in rc_kalman_lin_update, received NULL pointer\n");
		return -1;
	}
	if(unlikely(kf->initialized !=1)){
		fprintf(stderr, "ERROR in rc_kalman_lin_update, kf uninitialized\n");
		return -1;
	}
	if(unlikely(u.initialized!=1 || y.initialized!=1)){
		fprintf(stderr, "ERROR in rc_kalman_lin_update received uninitialized vector\n");
		return -1;
	}
	if(unlikely(u.len != kf->G.cols)){
		fprintf(stderr, "ERROR in rc_kalman_lin_update u must have same dimension as columns of G\n");
		return -1;
	}
	if(unlikely(y.len != kf->H.rows)){
		fprintf(stderr, "ERROR in rc_kalman_lin_update y must have same dimension as rows of H\n");
		return -1;
	}

	// for linear case only, calculate x_pre from linear system model
	// x_pre = x[k|k-1] = F*x[k-1|k-1] +  G*u[k-1]
	rc_matrix_times_col_vec(kf->F, kf->x_est, &tmp1);
	rc_matrix_times_col_vec(kf->G, u, &tmp2);
	rc_vector_sum(tmp1, tmp2, &kf->x_pre);

	// F is constant in this linear case
	// P[k|k-1] = F*P[k-1|k-1]*F^T + Q
	rc_matrix_multiply(kf->F, kf->P, &newP);	// newP = F*P_old
	rc_matrix_transpose(kf->F, &FT);
	rc_matrix_right_multiply_inplace(&newP, FT);	// P = F*P*F^T
	rc_matrix_add_inplace(&newP, kf->Q);		// P = F*P*F^T + Q
	rc_matrix_symmetrize(&newP);			// Force symmetric P

	// h[k] = H * x_pre[k]
	rc_matrix_times_col_vec(kf->H,kf->x_pre,&h);

	// H is constant in the linear case
	// S = H*P*H^T + R
	// Calculate H^T, borrow S for H^T
	rc_matrix_transpose(kf->H, &S);			// S = H^T
	// Calculate a part of L in advance before we modify S = H^T
	rc_matrix_multiply(newP, S, &L);		// K = P*(H^T)
	rc_matrix_left_multiply_inplace(newP, &S);	// S = P*H^T
	rc_matrix_left_multiply_inplace(kf->H, &S);	// S = H*(P*H^T)
	rc_matrix_add_inplace(&S, kf->R);		// S = H*P*H^T + R

	// L = P*(H^T)*(S^-1)
	rc_algebra_invert_matrix_inplace(&S);		// S2^(-1) = S^(-1)
	rc_matrix_right_multiply_inplace(&L, S);	// L = (P*H^T)*(S^-1)

	// x[k|k] = x[k|k-1] + K[k]*(y[k]-h[k])
	rc_vector_subtract(y,h,&z);			// z = k-h
	rc_matrix_times_col_vec(L, z, &tmp1);		// temp = L*z
	rc_vector_sum(kf->x_pre, tmp1, &kf->x_est);	// x_est = x + K*y

	// P[k|k] = (I - L*H)*P = P[k|k-1] - L*H*P[k|k-1], reuse the matrix S.
	rc_matrix_multiply(kf->H, newP, &S);		// S = H*P
	rc_matrix_left_multiply_inplace(L, &S);		// S = L*(H*P)
	rc_matrix_subtract_inplace(&newP, S);		// P = P - K*H*P
	rc_matrix_symmetrize(&newP);			// Force symmetric P
	rc_matrix_duplicate(newP,&kf->P);

	// cleanup
	rc_matrix_free(&L);
	rc_matrix_free(&newP);
	rc_matrix_free(&S);
	rc_matrix_free(&FT);
	rc_vector_free(&h);
	rc_vector_free(&z);
	rc_vector_free(&tmp1);
	rc_vector_free(&tmp2);

	kf->step++;
	return 0;
}


int rc_kalman_update_ekf(rc_kalman_t* kf, rc_matrix_t F, rc_matrix_t H, rc_vector_t x_pre, rc_vector_t y, rc_vector_t h)
{
	rc_matrix_t L = RC_MATRIX_INITIALIZER;
	rc_matrix_t newP = RC_MATRIX_INITIALIZER;
	rc_matrix_t S = RC_MATRIX_INITIALIZER;
	rc_matrix_t FT = RC_MATRIX_INITIALIZER;
	rc_vector_t z = RC_VECTOR_INITIALIZER;
	rc_vector_t tmp1 = RC_VECTOR_INITIALIZER;
	rc_vector_t tmp2 = RC_VECTOR_INITIALIZER;

	// sanity checks
	if(unlikely(kf==NULL)){
		fprintf(stderr, "ERROR in rc_kalman_ekf_update, received NULL pointer\n");
		return -1;
	}
	if(unlikely(kf->initialized !=1)){
		fprintf(stderr, "ERROR in rc_kalman_ekf_update, kf uninitialized\n");
		return -1;
	}
	if(unlikely(F.initialized!=1 || H.initialized!=1)){
		fprintf(stderr, "ERROR in rc_kalman_ekf_update received uninitialized matrix\n");
		return -1;
	}
	if(unlikely(x_pre.initialized!=1 || y.initialized!=1 || h.initialized!=1)){
		fprintf(stderr, "ERROR in rc_kalman_ekf_update received uninitialized vector\n");
		return -1;
	}
	if(unlikely(F.rows != F.cols)){
		fprintf(stderr, "ERROR in rc_kalman_ekf_update F must be square\n");
		return -1;
	}
	if(unlikely(x_pre.len != F.rows)){
		fprintf(stderr, "ERROR in rc_kalman_ekf_update x_pre must have same dimension as rows of F\n");
		return -1;
	}
	if(unlikely(x_pre.len != H.cols)){
		fprintf(stderr, "ERROR in rc_kalman_ekf_update x_pre must have same dimension as columns of H\n");
		return -1;
	}
	if(unlikely(y.len != H.rows)){
		fprintf(stderr, "ERROR in rc_kalman_ekf_update y must have same dimension as rows of H\n");
		return -1;
	}
	if(unlikely(y.len != h.len)){
		fprintf(stderr, "ERROR in rc_kalman_ekf_update y must have same dimension h\n");
		return -1;
	}

	// copy in new jacobians and x prediction
	rc_matrix_duplicate(F, &kf->F);
	rc_vector_duplicate(x_pre, &kf->x_pre);
	rc_matrix_duplicate(H, &kf->H);

	// F is new now in non-linear case
	// P[k|k-1] = F*P[k-1|k-1]*F^T + Q
	rc_matrix_multiply(kf->F, kf->P, &newP);	// P_new = F*P_old
	rc_matrix_transpose(kf->F, &FT);
	rc_matrix_right_multiply_inplace(&newP, FT);	// P = F*P*F^T
	rc_matrix_add(newP, kf->Q, &kf->P);		// P = F*P*F^T + Q
	rc_matrix_symmetrize(&kf->P);			// Force symmetric P

	// H is constant in the linear case
	// S = H*P*H^T + R
	// Calculate H^T, borrow S for H^T
	rc_matrix_transpose(kf->H, &S);			// S = H^T
	// Calculate a part of L in advance before we modify S = H^T
	rc_matrix_multiply(kf->P, S, &L);		// K = P*(H^T)
	rc_matrix_left_multiply_inplace(kf->P, &S);	// S = P*H^T
	rc_matrix_left_multiply_inplace(kf->H, &S);	// S = H*(P*H^T)
	rc_matrix_add_inplace(&S, kf->R);		// S = H*P*H^T + R

	// L = P*(H^T)*(S^-1)
	rc_algebra_invert_matrix_inplace(&S);		// S2^(-1) = S^(-1)
	rc_matrix_right_multiply_inplace(&L, S);	// L = (P*H^T)*(S^-1)

	// x[k|k] = x[k|k-1] + L[k]*(y[k]-h[k])
	rc_vector_subtract(y,h,&z);			// z = k-h
	rc_matrix_times_col_vec(L, z, &tmp1);		// temp = L*z
	rc_vector_sum(kf->x_pre, tmp1, &kf->x_est);	// x_est = x + L*y

	// P[k|k] = (I - L*H)*P = P - L*H*P, reuse the matrix S.
	rc_matrix_multiply(kf->H, newP, &S);		// S = H*P
	rc_matrix_left_multiply_inplace(L, &S);		// S = L*(H*P)
	rc_matrix_subtract_inplace(&newP, S);		// P = P - K*H*P
	rc_matrix_symmetrize(&newP);			// Force symmetric P
	rc_matrix_duplicate(newP,&kf->P);

	// cleanup
	rc_matrix_free(&L);
	rc_matrix_free(&newP);
	rc_matrix_free(&S);
	rc_matrix_free(&FT);
	rc_vector_free(&z);
	rc_vector_free(&tmp1);
	rc_vector_free(&tmp2);

	kf->step++;
	return 0;
}

