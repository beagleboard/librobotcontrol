/*******************************************************************************
* rc_test_algebra.c
*
* This tests some of the more common functions in rc_linear_algebra.c
* it is not a complete test of all available linear algebra functions but
* should get you started as an example.
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

#define DIM 3

int main(){
	float det;
	rc_matrix_t A = rc_empty_matrix();
	rc_matrix_t Ainv = rc_empty_matrix();
	rc_matrix_t AA = rc_empty_matrix();
	rc_matrix_t L = rc_empty_matrix();
	rc_matrix_t U = rc_empty_matrix();
	rc_matrix_t P = rc_empty_matrix();
	rc_matrix_t Q = rc_empty_matrix();
	rc_matrix_t R = rc_empty_matrix();
	rc_vector_t b = rc_empty_vector();
	rc_vector_t x = rc_empty_vector();
	rc_vector_t y = rc_empty_vector();
	
	printf("Let's test some linear algebra functions....\n\n");

	// identity matrix
	printf("Identity Matrix:\n");
	rc_identity_matrix(&A,DIM);
	rc_print_matrix(A);
	
	// zeros matrix
	printf("\nzeros Matrix:\n");
	rc_matrix_zeros(&A,DIM,DIM);
	rc_print_matrix(A);
	
	// random vector
	printf("\nNew Random Vector b:\n");
	rc_random_vector(&b,DIM);
	rc_print_vector(b);
	
	// diagonal matrix
	printf("\nDiagonal Matrix:\n");
	rc_diag_matrix(&A,b);
	rc_print_matrix(A);
	
	// random matrix
	printf("\nNew Random Matrix A:\n");
	rc_random_matrix(&A,DIM,DIM);
	rc_print_matrix(A);
	
	// duplicate matrix
	printf("\nDuplicate of Matrix A:\n");
	rc_duplicate_matrix(A,&AA);
	rc_print_matrix(AA);
	
	// get determinant of A
	printf("\nDeterminant of A :");
	det = rc_matrix_determinant(A);
	printf("%8.4f\n", det);
	
	// get an inverse for A
	printf("\nAinverse:\n");
	rc_invert_matrix(A,&Ainv);
	rc_print_matrix(Ainv);

	// multiply A times A inverse
	printf("\nA times Ainverse:\n");
	rc_multiply_matrices(A,Ainv,&AA);
	rc_print_matrix(AA);
	
	// invert A back again
	printf("\ninvert A again inplace\n");
	rc_invert_matrix_inplace(&Ainv);
	rc_print_matrix(Ainv);
	
	// multiply b times A
	printf("\nrow vector b times A:\n");
	rc_row_vec_times_matrix(b,A,&x);
	rc_print_vector(x);
	
	// multiply A times b
	printf("\nA times column vector b\n");
	rc_matrix_times_col_vec(A,b,&y);
	rc_print_vector(y);

	// do an LUP decomposition on A
	printf("\nLUP decomposition of A\n");
	rc_lup_decomp(A,&L,&U,&P);
	printf("L:\n");
	rc_print_matrix(L);
	printf("U:\n");
	rc_print_matrix(U);
	printf("P:\n");
	rc_print_matrix(P);

	// do a QR decomposition on A
	printf("\nQR Decomposition of A\n");
	rc_qr_decomp(A,&Q,&R);
	printf("Q:\n");
	rc_print_matrix(Q);
	printf("R:\n");
	rc_print_matrix(R);

	// solve a square linear system
	printf("\nGaussian Elimination solution x to the equation Ax=b:\n");
	printf("equivalent to A\\b in MATLAB\n");
	rc_lin_system_solve(A,b,&x);
	rc_print_vector(x);

	// now do again but with qr decomposition method
	printf("\nQR solution x to the equation Ax=b:\n");
	printf("equivalent to A\\b in MATLAB\n");
	rc_lin_system_solve_qr(A,b,&y);
	rc_print_vector(y);


	printf("\nDONE\n");
	return 0;
}
