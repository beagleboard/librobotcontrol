/*******************************************************************************
* test_algebra.c
*
* James Strawson 2016
* This tests some of the more common functions in linear_algebra.c, it is not a
* complete test of all available linear algebra functions but should get you
* started.
*******************************************************************************/

#include "../../libraries/roboticscape-usefulincludes.h"
#include "../../libraries/roboticscape.h"

#define DIM 3

int main(){
	printf("Let's test some linear algebra functions....\n\n");

	//create a random nxn matrix for later use
	printf("New Random Matrix A:\n");
	matrix_t A = create_random_matrix(DIM,DIM);

	print_matrix(A);

	// also create random vector
	printf("\nNew Random Vector b:\n");
	vector_t b = create_random_vector(DIM);
	print_vector(b);

	// get determinant of A
	double det = matrix_determinant(A);
	printf("\nDeterminant of A : %8.4f\n", det);

	// get an inverse for A
	printf("\nAinverse:\n");
	matrix_t Ainv = matrix_inverse(A);
	if(A.initialized != 1) return -1;
	print_matrix(Ainv);

	// multiply A times A inverse
	printf("\nA * Ainverse:\n");
	matrix_t AA = multiply_matrices(A,Ainv);
	if(AA.initialized!=1) return -1;
	print_matrix(AA);

	// do an LUP decomposition on A
	matrix_t L,U,P;
	printf("\nLUP decomposition of A\n");
	LUP_decomposition(A,&L,&U,&P);
	printf("\nL:\n");
	print_matrix(L);
	printf("U:\n");
	print_matrix(U);
	printf("P:\n");
	print_matrix(P);

	// do a QR decomposition on A
	matrix_t Q,R;
	printf("\nQR Decomposition of A\n");
	QR_decomposition(A,&Q,&R);
	printf("Q:\n");
	print_matrix(Q);
	printf("R:\n");
	print_matrix(R);

	// solve a square linear system
	printf("\nGaussian Elimination solution x to the equation Ax=b:\n");
	printf("equivalent to A\\b in MATLAB\n");
	vector_t x = lin_system_solve(A, b);
	print_vector(x);

	// now do again but with qr decomposition method
	printf("\nQR solution x to the equation Ax=b:\n");
	printf("equivalent to A\\b in MATLAB\n");
	vector_t xqr = lin_system_solve_qr(A, b);
	print_vector(xqr);

	// clean up all the allocated memory. This isn't strictly necessary since
	// we are already at the end of the program, but good practice to do.
	destroy_matrix(&A);
	destroy_vector(&b);
	destroy_matrix(&Ainv);
	destroy_matrix(&AA);
	destroy_matrix(&L);
	destroy_matrix(&U);
	destroy_matrix(&P);
	destroy_vector(&xqr);
	destroy_matrix(&Q);
	destroy_matrix(&R);
	destroy_vector(&x);

	printf("\nDONE\n");
	return 0;
}
