/**
 * @example    rc_test_algebra.c
 *
 * @brief      Tests the functions in rc_algebra.h
 *
 * @author     James Strawson
 * @date       1/29/2018
 */

#include <stdio.h>
#include <rc/math.h>

#define DIM 3

int main()
{
	rc_matrix_t A = rc_matrix_empty();
	rc_matrix_t Ainv = rc_matrix_empty();
	rc_matrix_t AA = rc_matrix_empty();
	rc_matrix_t L = rc_matrix_empty();
	rc_matrix_t U = rc_matrix_empty();
	rc_matrix_t P = rc_matrix_empty();
	rc_matrix_t Q = rc_matrix_empty();
	rc_matrix_t R = rc_matrix_empty();
	rc_vector_t b = rc_vector_empty();
	rc_vector_t x = rc_vector_empty();
	rc_vector_t y = rc_vector_empty();

	printf("Let's test some linear algebra functions....\n\n");


	// random matrix
	printf("\nNew Random Matrix A:\n");
	rc_matrix_random(&A,DIM,DIM);
	rc_matrix_print(A);

	// get an inverse for A
	printf("\nAinverse:\n");
	rc_algebra_invert_matrix(A,&Ainv);
	rc_matrix_print(Ainv);

	// multiply A times A inverse
	printf("\nA times A_inverse:\n");
	rc_matrix_multiply(A,Ainv,&AA);
	rc_matrix_print(AA);

	// invert A back again
	printf("\ninvert A again inplace\n");
	rc_algebra_invert_matrix_inplace(&Ainv);
	rc_matrix_print(Ainv);

	// do an LUP decomposition on A
	printf("\nLUP decomposition of A\n");
	rc_algebra_lup_decomp(A,&L,&U,&P);
	printf("L:\n");
	rc_matrix_print(L);
	printf("U:\n");
	rc_matrix_print(U);
	printf("P:\n");
	rc_matrix_print(P);

	// do a QR decomposition on A
	printf("\nQR Decomposition of A\n");
	rc_algebra_qr_decomp(A,&Q,&R);
	printf("Q:\n");
	rc_matrix_print(Q);
	printf("R:\n");
	rc_matrix_print(R);

	// solve a square linear system
	printf("\nGaussian Elimination solution x to the equation Ax=b:\n");
	printf("equivalent to A\\b in MATLAB\n");
	rc_vector_random(&b,DIM);
	rc_algebra_lin_system_solve(A,b,&x);
	rc_vector_print(x);

	// now do again but with qr decomposition method
	printf("\nQR solution x to the equation Ax=b:\n");
	printf("equivalent to A\\b in MATLAB\n");
	rc_algebra_lin_system_solve_qr(A,b,&y);
	rc_vector_print(y);


	printf("\nDONE\n");
	return 0;
}
