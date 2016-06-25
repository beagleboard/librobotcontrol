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
	matrix_t A = createRandomMatrix(DIM,DIM);
	printf("New Random Matrix A:\n");
	printMatrix(A);
	
	// also create random vector
	vector_t b = createRandomVector(DIM);
	printf("\nNew Random Vector b:\n");
	printVector(b);
	
	// do an LUP decomposition on A
	matrix_t L,U,P;
	LUPdecomposition(A,&L,&U,&P);
	printf("\nL:\n");
	printMatrix(L);
	printf("U:\n");
	printMatrix(U);
	printf("P:\n");
	printMatrix(P);
	
	// do a QR decomposition on A
	matrix_t Q,R;
	QRdecomposition(A,&Q,&R);
	printf("\nQR Decomposition of A\n");
	printf("Q:\n");
	printMatrix(Q);
	printf("R:\n");
	printMatrix(R);
	
	// get determinant of A
	float det = matrixDeterminant(A);
	printf("\nDeterminant of A : %8.4f\n", det);
	
	// get an inverse for A
	matrix_t Ainv = duplicateMatrix(A);
	if(invertMatrix(&Ainv)<0) return -1;
	printf("A inv\n");
	printMatrix(Ainv);
	
	// multiply A times A inverse
	matrix_t AA = matrixMultiply(A,Ainv);
	printf("\nA * A^(-1):\n");
	printMatrix(AA);
	
	// solve a square linear system
	vector_t x = linSolve(A, b);
	printf("\nGaussian Elimination solution x to the equation Ax=b:\n");
	printVector(x);
	
	// now do again but with qr decomposition method
	vector_t xqr = linSolveQR(A, b);
	printf("\nQR solution x to the equation Ax=b:\n");
	printVector(xqr);
	
	// If b are the coefficients of a polynomial, get the coefficients of the
	// new polynomial b^2
	vector_t bb;
	polyPower(b,2,&bb);
	printf("Coefficients of polynomial b times itself\n");
	printVector(bb);
	
	// clean up all the allocated memory. This isn't strictly necessary since
	// we are already at the end of the program, but good practice to do.
	destroyMatrix(&A);
	destroyMatrix(&AA);
	destroyVector(&b);
	destroyVector(&bb);
	destroyVector(&x);
	destroyVector(&xqr);
	destroyMatrix(&Q);
	destroyMatrix(&R);

	printf("DONE\n");
	return 0;
}
