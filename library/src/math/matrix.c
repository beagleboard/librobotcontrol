/**
 * @file math/matrix.c
 *
 * @brief      This is a collection of functions for generating and implementing
 *             discrete SISO filters for arbitrary transfer functions.
 *
 * @author     James Strawson
 * @date       2016
 *
 */

#include <stdio.h>	// for fprintf
#include <stdlib.h>	// for malloc,calloc,free
#include <string.h>	// for memcpy

#include <rc/math/other.h>
#include <rc/math/matrix.h>
#include "algebra_common.h"


rc_matrix_t rc_matrix_empty(void)
{
	rc_matrix_t out = RC_MATRIX_INITIALIZER;
	return out;
}


int rc_matrix_alloc(rc_matrix_t* A, int rows, int cols)
{
	int i;
	// sanity checks
	if(unlikely(rows<1 || cols<1)){
		fprintf(stderr,"ERROR in rc_matrix_alloc, rows and cols must be >=1\n");
		return -1;
	}
	if(unlikely(A==NULL)){
		fprintf(stderr,"ERROR in rc_matrix_alloc, received NULL pointer\n");
		return -1;
	}
	// if A is already allocated and of the right size, nothing to do!
	if(A->initialized==1 && rows==A->rows && cols==A->cols) return 0;
	// free any old memory
	rc_matrix_free(A);
	// allocate contiguous memory for the major(row) pointers
	A->d = (double**)malloc(rows*sizeof(double*));
	if(unlikely(A->d==NULL)){
		perror("ERROR in rc_matrix_alloc");
		fprintf(stderr, "tried allocating a %dx%d matrix\n", rows,cols);
		return -1;
	}
	// allocate contiguous memory for the actual data
	void* ptr = malloc(rows*cols*sizeof(double));
	if(unlikely(ptr==NULL)){
		perror("ERROR in rc_matrix_alloc");
		fprintf(stderr, "tried allocating a %dx%d matrix\n", rows,cols);
		free(A->d);
		return -1;
	}
	// manually fill in the pointer to each row
	for(i=0;i<rows;i++) A->d[i]=(double*)(((char*)ptr) + (i*cols*sizeof(double)));
	A->rows = rows;
	A->cols = cols;
	A->initialized = 1;
	return 0;
}


int rc_matrix_free(rc_matrix_t* A)
{
	rc_matrix_t new = RC_MATRIX_INITIALIZER;
	if(unlikely(A==NULL)){
		fprintf(stderr,"ERROR in rc_matrix_free, received NULL pointer\n");
		return -1;
	}
	// free memory allocated for the data then the major array
	if(A->d!=NULL && A->initialized==1) free(A->d[0]);
	free(A->d);
	// zero out the struct
	*A = new;
	return 0;
}


int rc_matrix_zeros(rc_matrix_t* A, int rows, int cols)
{
	int i;
	// sanity checks
	if(unlikely(rows<1 || cols<1)){
		fprintf(stderr,"ERROR in rc_create_matrix_zeros, rows and cols must be >=1\n");
		return -1;
	}
	if(unlikely(A==NULL)){
		fprintf(stderr,"ERROR in rc_create_matrix_zeros, received NULL pointer\n");
		return -1;
	}
	// make sure A is freed before allocating new memory
	rc_matrix_free(A);
	// allocate contiguous memory for the major(row) pointers
	A->d = (double**)malloc(rows*sizeof(double*));
	if(unlikely(A->d==NULL)){
		fprintf(stderr,"ERROR in rc_create_matrix_zeros, not enough memory\n");
		return -1;
	}
	// allocate contiguous memory for the actual data
	void* ptr = calloc(rows*cols,sizeof(double));
	if(unlikely(ptr==NULL)){
		fprintf(stderr,"ERROR in rc_create_matrix_zeros, not enough memory\n");
		free(A->d);
		return -1;
	}
	// manually fill in the pointer to each row
	for(i=0;i<rows;i++) A->d[i]=(double*)(((char*)ptr) + (i*cols*sizeof(double)));
	A->rows = rows;
	A->cols = cols;
	A->initialized = 1;
	return 0;
}


int rc_matrix_identity(rc_matrix_t* A, int dim)
{
	int i;
	if(unlikely(rc_matrix_zeros(A,dim,dim))){
		fprintf(stderr,"ERROR in rc_matrix_identity, failed to allocate matrix\n");
		return -1;
	}
	// fill in diagonal of ones
	for(i=0;i<dim;i++) A->d[i][i]=1.0;
	return 0;
}


int rc_matrix_random(rc_matrix_t* A, int rows, int cols)
{
	int i;
	if(unlikely(rc_matrix_alloc(A,rows,cols))){
		fprintf(stderr,"ERROR in rc_matrix_random, failed to allocate matrix\n");
		return -1;
	}
	for(i=0;i<(A->rows*A->cols);i++) A->d[0][i]=rc_get_random_double();
	return 0;
}


int rc_matrix_diagonal(rc_matrix_t* A, rc_vector_t v)
{
	int i;
	// sanity check
	if(unlikely(v.initialized!=1)){
		fprintf(stderr,"ERROR in rc_matrix_diagonal, vector not initialized\n");
		return -1;
	}
	// allocate fresh zero-initialized memory for A
	if(unlikely(rc_matrix_zeros(A,v.len,v.len))){
		fprintf(stderr,"ERROR in rc_matrix_diagonal, failed to allocate matrix\n");
		return -1;
	}
	for(i=0;i<v.len;i++) A->d[i][i]=v.d[i];
	return 0;
}


int rc_matrix_duplicate(rc_matrix_t A, rc_matrix_t* B)
{
	// sanity check
	if(unlikely(A.initialized!=1)){
		fprintf(stderr,"ERROR in rc_matrix_duplicate not initialized yet\n");
		return -1;
	}
	// make sure there is enough space in B
	if(unlikely(rc_matrix_alloc(B,A.rows,A.cols))){
		fprintf(stderr,"ERROR in rc_matrix_duplicate, failed to allocate memory\n");
		return -1;
	}
	// all matrix data is stored contiguously so one memcpy is sufficient
	memcpy(B->d[0],A.d[0],A.rows*A.cols*sizeof(double));
	return 0;
}


int rc_matrix_print(rc_matrix_t A)
{
	int i,j;
	if(unlikely(A.initialized!=1)){
		fprintf(stderr,"ERROR in rc_matrix_print, matrix not initialized yet\n");
		return -1;
	}
	for(i=0;i<A.rows;i++){
		for(j=0;j<A.cols;j++){
			printf("%7.4f  ",A.d[i][j]);
		}
		printf("\n");
	}
	return 0;
}


int rc_matrix_print_sci(rc_matrix_t A)
{
	int i,j;
	if(unlikely(A.initialized!=1)){
		fprintf(stderr,"ERROR in rc_matrix_print_sci, matrix not initialized yet\n");
		return -1;
	}
	for(i=0;i<A.rows;i++){
		for(j=0;j<A.cols;j++){
			printf("%11.4e  ",A.d[i][j]);
		}
		printf("\n");
	}
	return 0;
}


int rc_matrix_zero_out(rc_matrix_t* A)
{
	int i,j;
	if(unlikely(A->initialized!=1)){
		fprintf(stderr,"ERROR in rc_matrix_zero_out, matrix not initialized yet\n");
		return -1;
	}
	for(i=0;i<A->rows;i++){
		for(j=0;j<A->cols;j++){
			A->d[i][j]=0.0;
		}
	}
	return 0;
}


int rc_matrix_times_scalar(rc_matrix_t* A, double s)
{
	int i;
	if(unlikely(A->initialized!=1)){
		fprintf(stderr,"ERROR in rc_matrix_times_scalar. matrix uninitialized\n");
		return -1;
	}
	// since A contains contiguous memory, gcc should vectorize this loop
	for(i=0;i<(A->rows*A->cols);i++) A->d[0][i] *= s;
	return 0;
}


int rc_matrix_multiply(rc_matrix_t A, rc_matrix_t B, rc_matrix_t* C)
{
	int i,j;
	double* tmp;
	if(unlikely(A.initialized!=1 || B.initialized!=1)){
		fprintf(stderr,"ERROR in rc_matrix_multiply, matrix not initialized\n");
		return -1;
	}
	if(unlikely(A.cols!=B.rows)){
		fprintf(stderr,"ERROR in rc_matrix_multiply, dimension mismatch\n");
		return -1;
	}
	// if C is not initialized, allocate memory for it
	if(unlikely(rc_matrix_alloc(C,A.rows,B.cols))){
		fprintf(stderr,"ERROR in rc_matrix_multiply, can't allocate memory for C\n");
		return -1;
	}
	// allocate memory for a column of B from the stack, this is faster than
	// malloc and the memory is freed automatically when this function returns
	// it is faster to put a column in contiguous memory before multiplying
	tmp = alloca(B.rows*sizeof(double));
	if(unlikely(tmp==NULL)){
		fprintf(stderr,"ERROR in rc_matrix_multiply, alloca failed, stack overflow\n");
		return -1;
	}
	// go through columns of B calculating columns of C left to right
	for(i=0;i<(B.cols);i++){
		// put column of B in sequential memory slot
		for(j=0;j<B.rows;j++) tmp[j]=B.d[j][i];
		// calculate each row in column i
		for(j=0;j<(A.rows);j++){
			C->d[j][i]=__vectorized_mult_accumulate(A.d[j],tmp,B.rows);
		}
	}
	return 0;
}


int rc_matrix_left_multiply_inplace(rc_matrix_t A, rc_matrix_t* B)
{
	rc_matrix_t tmp = RC_MATRIX_INITIALIZER;
	// Sanity Checks
	if(unlikely(A.initialized!=1 || B->initialized!=1)){
		fprintf(stderr,"ERROR in rc_matrix_left_multiply_inplace, matrix not initialized\n");
		return -1;
	}
	if(unlikely(A.cols!=B->rows)){
		fprintf(stderr,"ERROR in rc_matrix_left_multiply_inplace, dimension mismatch\n");
		return -1;
	}
	// use the normal multiply function which will allocate memory for tmp
	if(rc_matrix_multiply(A, *B, &tmp)){
		fprintf(stderr,"ERROR in rc_matrix_left_multiply_inplace, failed to multiply\n");
		rc_matrix_free(&tmp);
		return -1;
	}
	rc_matrix_free(B);
	*B=tmp;
	return 0;
}


int rc_matrix_right_multiply_inplace(rc_matrix_t* A, rc_matrix_t B)
{
	rc_matrix_t tmp = RC_MATRIX_INITIALIZER;
	// Sanity Checks
	if(unlikely(A->initialized!=1 || B.initialized!=1)){
		fprintf(stderr,"ERROR in rc_matrix_right_multiply_inplace, matrix not initialized\n");
		return -1;
	}
	if(unlikely(A->cols!=B.rows)){
		fprintf(stderr,"ERROR in rc_matrix_right_multiply_inplace, dimension mismatch\n");
		return -1;
	}
	if(rc_matrix_multiply(*A, B, &tmp)){
		fprintf(stderr,"ERROR in rc_matrix_right_multiply_inplace, failed to multiply\n");
		rc_matrix_free(&tmp);
		return -1;
	}
	rc_matrix_free(A);
	*A=tmp;
	return 0;
}


int rc_matrix_add(rc_matrix_t A, rc_matrix_t B, rc_matrix_t* C)
{
	int i;
	if(unlikely(A.initialized!=1 || B.initialized!=1)){
		fprintf(stderr,"ERROR in rc_matrix_add, matrix not initialized\n");
		return -1;
	}
	if(unlikely(A.rows!=B.rows || A.cols!=B.cols)){
		fprintf(stderr,"ERROR in rc_matrix_add, dimension mismatch\n");
		return -1;
	}
	// make sure C is allocated
	if(unlikely(rc_matrix_alloc(C,A.rows,A.cols))){
		fprintf(stderr,"ERROR in rc_matrix_add, can't allocate memory for C\n");
		return -1;
	}
	for(i=0;i<(A.rows*A.cols);i++) C->d[0][i]=A.d[0][i]+B.d[0][i];
	return 0;
}


int rc_matrix_add_inplace(rc_matrix_t* A, rc_matrix_t B)
{
	int i;
	if(unlikely(A->initialized!=1 || B.initialized!=1)){
		fprintf(stderr,"ERROR in rc_matrix_add_inplace, matrix not initialized\n");
		return -1;
	}
	if(unlikely(A->rows!=B.rows || A->cols!=B.cols)){
		fprintf(stderr,"ERROR in rc_matrix_add_inplace, dimension mismatch\n");
		return -1;
	}
	for(i=0;i<(A->rows*A->cols);i++) A->d[0][i]+=B.d[0][i];
	return 0;
}

int rc_matrix_subtract_inplace(rc_matrix_t* A, rc_matrix_t B)
{
	int i;
	if(unlikely(A->initialized!=1 || B.initialized!=1)){
		fprintf(stderr,"ERROR in rc_matrix_subtract_inplace, matrix not initialized\n");
		return -1;
	}
	if(unlikely(A->rows!=B.rows || A->cols!=B.cols)){
		fprintf(stderr,"ERROR in rc_matrix_subtract_inplace, dimension mismatch\n");
		return -1;
	}
	for(i=0;i<(A->rows*A->cols);i++) A->d[0][i]-=B.d[0][i];
	return 0;
}


int rc_matrix_transpose(rc_matrix_t A, rc_matrix_t* T)
{
	int i,j;
	if(unlikely(A.initialized!=1)){
		fprintf(stderr,"ERROR in rc_matrix_transpose, received uninitialized matrix\n");
		return -1;
	}
	// make sure T is allocated
	if(unlikely(rc_matrix_alloc(T,A.cols,A.rows))){
		fprintf(stderr,"ERROR in rc_matrix_transpose, can't allocate memory for T\n");
		return -1;
	}
	// fill in new memory
	for(i=0;i<(A.rows);i++){
		for(j=0;j<(A.cols);j++){
			T->d[j][i] = A.d[i][j];
		}
	}
	return 0;
}


int rc_matrix_transpose_inplace(rc_matrix_t* A)
{
	rc_matrix_t tmp = RC_MATRIX_INITIALIZER;
	if(unlikely(A==NULL)){
		fprintf(stderr,"ERROR in rc_transpose_matrix_inplace, received NULL pointer\n");
		return -1;
	}
	if(unlikely(!A->initialized)){
		fprintf(stderr,"ERROR in rc_transpose_matrix_inplace, matrix uninitialized\n");
		return -1;
	}
	// shortcut for 1x1 matrix
	if(A->rows==1 && A->cols==1) return 0;
	// allocate memory for new A, easier than doing it in place since A will
	// change size if non-square
	if(unlikely(rc_matrix_transpose(*A, &tmp))){
		fprintf(stderr,"ERROR in rc_transpose_matrix_inplace, can't transpose\n");
		rc_matrix_free(&tmp);
		return -1;
	}
	// free the original matrix A and set it's struct to point to the new memory
	rc_matrix_free(A);
	*A=tmp;
	return 0;
}

int rc_matrix_times_col_vec(rc_matrix_t A, rc_vector_t v, rc_vector_t* c)
{
	int i;
	// sanity checks
	if(unlikely(A.initialized!=1 || v.initialized!=1)){
		fprintf(stderr,"ERROR in rc_matrix_times_col_vec, matrix or vector uninitialized\n");
		return -1;
	}
	if(unlikely(A.cols!=v.len)){
		fprintf(stderr,"ERROR in rc_matrix_times_col_vec, dimension mismatch\n");
		return -1;
	}
	if(unlikely(rc_vector_alloc(c,A.rows))){
		fprintf(stderr,"ERROR in rc_matrix_times_col_vec, failed to allocate c\n");
		return -1;
	}
	// run the sum
	for(i=0;i<A.rows;i++) c->d[i]=__vectorized_mult_accumulate(A.d[i],v.d,v.len);
	return 0;
}


int rc_matrix_row_vec_times_matrix(rc_vector_t v, rc_matrix_t A, rc_vector_t* c)
{
	int i,j;
	double* tmp;
	// sanity checks
	if(unlikely(A.initialized!=1 || v.initialized!=1)){
		fprintf(stderr,"ERROR in rc_matrix_row_vec_times_matrix, matrix or vector uninitialized\n");
		return -1;
	}
	if(unlikely(A.rows!=v.len)){
		fprintf(stderr,"ERROR in rc_matrix_row_vec_times_matrix, dimension mismatch\n");
		return -1;
	}
	// allocate memory for a column of A from the stack, this is faster than
	// malloc and the memory is freed automatically when this function returns
	// it is faster to put a column of A in contiguous memory then multiply
	tmp = alloca(A.rows*sizeof(double));
	if(unlikely(tmp==NULL)){
		fprintf(stderr,"ERROR in rc_matrix_row_vec_times_matrix, alloca failed, stack overflow\n");
		return -1;
	}
	// make sure c is allocated correctly
	if(unlikely(rc_vector_alloc(c,A.cols))){
		fprintf(stderr,"ERROR in rc_matrix_row_vec_times_matrix, failed to allocate c\n");
		return -1;
	}
	// go through columns of A calculating c left to right
	for(i=0;i<A.cols;i++){
		// put column of A in sequential memory slot
		for(j=0;j<A.rows;j++) tmp[j]=A.d[j][i];
		// calculate each entry in c
		c->d[i]=__vectorized_mult_accumulate(v.d,tmp,v.len);
	}
	return 0;
}

int rc_matrix_outer_product(rc_vector_t v1, rc_vector_t v2, rc_matrix_t* A)
{
	int i, j;
	if(unlikely(v1.initialized!=1 || v2.initialized!=1)){
		fprintf(stderr,"ERROR in rc_matrix_outer_product, vector uninitialized\n");
		return -1;
	}
	if(unlikely(rc_matrix_alloc(A,v1.len,v2.len))){
		fprintf(stderr,"ERROR in rc_matrix_outer_product, failed to allocate A\n");
		return -1;
	}
	for(i=0;i<v1.len;i++){
		for(j=0;j<v2.len;j++){
			A->d[i][j] = v1.d[i]*v2.d[j];
		}
	}
	return 0;
}

double rc_matrix_determinant(rc_matrix_t A)
{
	int i,j,k;
	double ratio, det;
	rc_matrix_t tmp = RC_MATRIX_INITIALIZER;
	// sanity checks
	if(unlikely(A.initialized!=1)){
		fprintf(stderr,"ERROR in rc_matrix_determinant, received uninitialized matrix\n");
		return -1.0;
	}
	if(unlikely(A.rows!=A.cols)){
		fprintf(stderr,"ERROR in rc_matrix_determinant, expected square matrix\n");
		return -1.0;
	}
	// shortcut for 1x1 matrix
	if(A.rows==1) return A.d[0][0];
	// shortcut for 2x2 matrix
	if(A.rows==2) return A.d[0][0]*A.d[1][1] - A.d[0][1]*A.d[1][0];
	// allocate a duplicate to shuffle around
	if(unlikely(rc_matrix_duplicate(A,&tmp))){
		fprintf(stderr,"ERROR in rc_matrix_determinant, failed to allocate duplicate\n");
		return -1.0;
	}
	for(i=0;i<(A.rows-1);i++){
		for(j=i+1;j<A.rows;j++){
			ratio = tmp.d[j][i]/tmp.d[i][i];
			for(k=0;k<A.rows;k++) tmp.d[j][k] -= ratio*tmp.d[i][k];
		}
	}
	// multiply along the main diagonal
	det = 1.0;
	for(i=0;i<A.rows;i++) det *= tmp.d[i][i];
	// free memory and return
	rc_matrix_free(&tmp);
	return det;
}

int rc_matrix_symmetrize(rc_matrix_t* P)
{
	int i,j;
	double val;
	// sanity checks
	if(P==NULL){
		fprintf(stderr, "ERROR in rc_matrix_symmetrize, matrix pointer is NULL\n");
		return -1;
	}
	if(P->initialized!=1){
		fprintf(stderr, "ERROR in rc_matrix_symmetrize, matrix uninitialized\n");
		return -1;
	}
	if(P->rows != P->cols){
		fprintf(stderr, "ERROR in rc_matrix_symmetrize, matrix must be square\n");
		return -1;
	}
	// itterate top to bottom, skipping last row
	for(i=0; i<(P->rows-1); i++){
		// itterate left to right, skipping diagonal
		for(j=i+1; j<P->cols; j++){
			val = (P->d[i][j] + P->d[j][i])/2.0;
			P->d[i][j] = val;
			P->d[j][i] = val;
		}
	}
	return 0;
}