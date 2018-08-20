/**
 * @example    rc_test_matrix.c
 *
 * @brief      Tests the functions in rc_matrix.h
 *
 * @author     James Strawson
 * @date       1/29/2018
 */

#include <stdio.h>
#include <rc/math.h>

#define DIM 3 // dimension of matrix to test

int main()
{
	double det;
	rc_matrix_t A		= RC_MATRIX_INITIALIZER;
	rc_matrix_t A_dup	= RC_MATRIX_INITIALIZER;
	rc_matrix_t B		= RC_MATRIX_INITIALIZER;
	rc_matrix_t B_dup	= RC_MATRIX_INITIALIZER;
	rc_matrix_t C		= RC_MATRIX_INITIALIZER;
	rc_vector_t b		= RC_VECTOR_INITIALIZER;
	rc_vector_t y		= RC_VECTOR_INITIALIZER;

	printf("Let's test some matrix functions....\n\n");

	// zeros matrix
	printf("\nzeros Matrix:\n");
	rc_matrix_zeros(&A,DIM,DIM);
	rc_matrix_print(A);

	// identity matrix
	printf("\nIdentity Matrix:\n");
	rc_matrix_identity(&A,DIM);
	rc_matrix_print(A);

	// random vector
	printf("\nNew Random Vector b:\n");
	rc_vector_random(&b,DIM);
	rc_vector_print(b);

	// diagonal matrix
	printf("\nDiagonal Matrix from b:\n");
	rc_matrix_diagonal(&A,b);
	rc_matrix_print(A);

	// random matrix
	printf("\nNew Random Matrix A:\n");
	rc_matrix_random(&A,DIM,DIM);
	rc_matrix_print(A);

	// duplicate matrix
	printf("\nDuplicate A into B:\n");
	rc_matrix_duplicate(A,&B);
	rc_matrix_print(B);

	// print scientific notation
	printf("\nMatrix B in sci notation:\n");
	rc_matrix_print_sci(B);

	// times scalar
	printf("\nMatrix B times 2.0:\n");
	rc_matrix_times_scalar(&B,2.0);
	rc_matrix_print(B);

	// get determinant of A & B
	printf("\nDeterminant of A:");
	det = rc_matrix_determinant(A);
	printf("%8.6lf\n", det);
	printf("\nDeterminant of B:");
	det = rc_matrix_determinant(B);
	printf("%8.6lf\n", det);

	// multiply A*B=C
	printf("\nThree ways to multiply:");
	printf("\nA*B=C:\n");
	rc_matrix_multiply(A,B,&C);
	rc_matrix_print(C);

	// left multiply in place
	printf("\nleft multiply inplace B=A*B\n");
	rc_matrix_duplicate(B,&B_dup);
	rc_matrix_left_multiply_inplace(A,&B_dup);
	rc_matrix_print(B_dup);

	// right multiply in place
	printf("\nright multiply inplace A=A*B\n");
	rc_matrix_duplicate(A,&A_dup);
	rc_matrix_right_multiply_inplace(&A_dup,B);
	rc_matrix_print(A_dup);

	// add
	printf("\ntwo ways to add:");
	printf("\nA+B=C:\n");
	rc_matrix_add(A,B,&C);
	rc_matrix_print(C);
	printf("\ninplace: A=A+B:\n");
	rc_matrix_duplicate(A,&A_dup);
	rc_matrix_add_inplace(&A_dup,B);
	rc_matrix_print(A_dup);

	// transpose
	printf("\ntwo ways to transpose:");
	printf("\nC=A':\n");
	rc_matrix_transpose(A,&C);
	rc_matrix_print(C);
	printf("\ninplace: A=A'\n");
	rc_matrix_duplicate(A,&A_dup);
	rc_matrix_transpose_inplace(&A_dup);
	rc_matrix_print(A_dup);


	// multiply b times A
	printf("\nrow vector b times A:\n");
	rc_matrix_row_vec_times_matrix(b,A,&y);
	rc_vector_print(y);

	// multiply A times b
	printf("\nA times column vector b\n");
	rc_matrix_times_col_vec(A,b,&y);
	rc_vector_print(y);

	// outer product
	printf("\nouter product C=b*y\n");
	rc_matrix_outer_product(b,y,&C);
	rc_matrix_print(C);


	printf("\nDONE\n");
	return 0;
}
