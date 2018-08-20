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
	rc_matrix_t A	= RC_MATRIX_INITIALIZER;
	rc_matrix_t Ainv= RC_MATRIX_INITIALIZER;
	rc_matrix_t AA	= RC_MATRIX_INITIALIZER;
	rc_matrix_t L	= RC_MATRIX_INITIALIZER;
	rc_matrix_t U	= RC_MATRIX_INITIALIZER;
	rc_matrix_t P	= RC_MATRIX_INITIALIZER;
	rc_matrix_t Q	= RC_MATRIX_INITIALIZER;
	rc_matrix_t R	= RC_MATRIX_INITIALIZER;
	rc_vector_t b	= RC_VECTOR_INITIALIZER;
	rc_vector_t x	= RC_VECTOR_INITIALIZER;
	rc_vector_t y	= RC_VECTOR_INITIALIZER;

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

	// free memory
	rc_matrix_free(&A);
	rc_matrix_free(&Ainv);
	rc_matrix_free(&AA);
	rc_matrix_free(&L);
	rc_matrix_free(&U);
	rc_matrix_free(&P);
	rc_matrix_free(&Q);
	rc_matrix_free(&R);
	rc_vector_free(&b);
	rc_vector_free(&x);
	rc_vector_free(&y);
	printf("\nDONE\n");
	return 0;
}
