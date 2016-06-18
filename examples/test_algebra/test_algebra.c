/*******************************************************************************
* bare_minimum.c
*
* This is meant to be a skeleton program for future projects. 
*******************************************************************************/

#include <robotics_cape.h>
#include <useful_includes.h>

#define DIM 3

int main(){
	//int i;
	printf("Let's test some linear algebra functions....\n");
	
	matrix_t A = createRandomMatrix(DIM,DIM);
	printf("Matrix A:\n");
	printMatrix(A);
	
	vector_t b = createRandomVector(DIM);
	printf("\nvector b:\n");
	printVector(b);
	
	vector_t x = linSolve(A, b);
	printf("\nsolution x to the equation Ax=b:\n");
	printVector(x);
	
	float det = matrixDeterminant(A);
	printf("\nDeterminant of A : %8.4f\n", det);
	
	matrix_t AA = matrixMultiply(A,A);
	printf("\nMatrix A^2:\n");
	printMatrix(AA);
	
	printf("Random tall matrix B:\n");
	matrix_t B = createRandomMatrix(2*DIM,DIM);
	printMatrix(B);
	
	printf("QR Decomposition of B\n");
	matrix_t Q,R;
	QRdecomposition(B,&Q,&R);
	printf("Q:\n");
	printMatrixSciNotation(Q);
	printf("R:\n");
	printMatrixSciNotation(R);
	
	destroyMatrix(&A);
	destroyMatrix(&AA);
	destroyVector(&b);
	destroyVector(&x);
	destroyMatrix(&B);
	destroyMatrix(&Q);
	destroyMatrix(&R);
	return 0;
}
