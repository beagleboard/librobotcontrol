/*******************************************************************************
* test_algebra.c
*
* This tests some of the more common functions in linear_algebra.c, it is not a
* complete test of all available linear algebra functions but should get you
* started.
*******************************************************************************/

#include <robotics_cape.h>
#include <useful_includes.h>

#define DIM 5
	
int main(){
	printf("Let's test some linear algebra functions....\n\n");
	
	// create a random nxn matrix for later use
	matrix_t A = create_random_matrix(DIM,DIM);
	printf("New Random Matrix A:\n");
	print_matrix(A);
	
	// also create random vector
	vector_t b = create_random_vector(DIM);
	printf("\nNew Random Vector b:\n");
	print_vector(b);
	
	// do an LUP decomposition on A
	matrix_t L,U,P;
	LUP_decomposition(A,&L,&U,&P);
	printf("\nL:\n");
	print_matrix(L);
	printf("U:\n");
	print_matrix(U);
	printf("P:\n");
	print_matrix(P);
	
	// do a QR decomposition on A
	matrix_t Q,R;
	QR_decomposition(A,&Q,&R);
	printf("\nQR Decomposition of A\n");
	printf("Q:\n");
	print_matrix(Q);
	printf("R:\n");
	print_matrix(R);
	
	// get determinant of A
	float det = matrix_determinant(A);
	printf("\nDeterminant of A : %8.4f\n", det);
	
	// get an inverse for A
	matrix_t Ainv;
	if(invert_matrix(A,&Ainv)<0) return -1;
	printf("A inv\n");
	print_matrix(Ainv);
	
	// multiply A times A inverse
	matrix_t AA;
	if(multiply_matrices(A,Ainv,&AA)){
		printf("ERROR: can't multiply matrices\n");
		return -1;
	}
	printf("\nA * A^(-1):\n");
	print_matrix(AA);
	
	// solve a square linear system
	vector_t x = lin_system_solve(A, b);
	printf("\nGaussian Elimination solution x to the equation Ax=b:\n");
	print_vector(x);
	
	// now do again but with qr decomposition method
	vector_t xqr = lin_system_solve_qr(A, b);
	printf("\nQR solution x to the equation Ax=b:\n");
	print_vector(xqr);
	
	// If b are the coefficients of a polynomial, get the coefficients of the
	// new polynomial b^2
	vector_t bb;
	polynomial_power(b,2,&bb);
	printf("Coefficients of polynomial b times itself\n");
	print_vector(bb);
	
	// clean up all the allocated memory. This isn't strictly necessary since
	// we are already at the end of the program, but good practice to do.
	destroy_matrix(&A);
	destroy_matrix(&AA);
	destroy_vector(&b);
	destroy_vector(&bb);
	destroy_vector(&x);
	destroy_vector(&xqr);
	destroy_matrix(&Q);
	destroy_matrix(&R);

	printf("DONE\n");
	return 0;
}
