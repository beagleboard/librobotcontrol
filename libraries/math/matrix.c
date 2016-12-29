/*******************************************************************************
* matrix.c
*
* James Strawson & Matt Atlas 2016
* Numerical Renaissance codebase used as reference for many algorithms
*******************************************************************************/

#include "../roboticscape.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h> // for memset


/*******************************************************************************
* matrix_t create_matrix(int rows, int cols)
*
* Allocates memory for a matrix full of zeros. Returns the matrix_t struct
* containing rows, columns, and the memory pointer
*******************************************************************************/
matrix_t create_matrix(int rows, int cols){
	int i;
	matrix_t A;
	if(rows<1 || cols<1){
		printf("error creating matrix, row or col must be >=1");
		return A;
	}
	A.rows = rows;
	A.cols = cols;
	// allocate contiguous memory
	A.data = (double**)malloc(rows*sizeof(double*));
	void* ptr = calloc(rows*cols, sizeof(double));
	// manually fill in the pointer to each row
	for (i=0; i<rows; i++){
		A.data[i] = (double*)(ptr + i*cols*sizeof(double));
	}
	
	A.initialized = 1;
	return A;
}

/*******************************************************************************
* matrix_t create_matrix_fast(int rows, int cols)
*
* Like create_matrix but contents are not guaranteed to be 0
*******************************************************************************/
matrix_t create_matrix_fast(int rows, int cols){
	int i;
	matrix_t A;
	if(rows<1 || cols<1){
		printf("error creating matrix, row or col must be >=1");
		return A;
	}
	A.rows = rows;
	A.cols = cols;
	// allocate contiguous memory
	A.data = (double**)malloc(rows*sizeof(double*));
	void* ptr = malloc(rows*cols*sizeof(double));
	// manually fill in the pointer to each row
	for (i=0; i<rows; i++){
		A.data[i] = (double*)(ptr + i*cols*sizeof(double));
	}
	
	A.initialized = 1;
	return A;
}

/*******************************************************************************
* void destroy_matrix(matrix_t* A)
*
* 
*******************************************************************************/
void destroy_matrix(matrix_t* A){
	if(A==NULL){
		printf("ERROR: Can't destroy matrix, NULL pointer detected\n");
		return;
	}
	if(A->initialized==1 && A->rows>0 && A->cols>0){
		free(A->data[0]);
		free(A->data);
	}
	memset(A,0,sizeof(matrix_t));
	return;
}

/*******************************************************************************
* matrix_t empty_matrix()
*
* 
*******************************************************************************/
matrix_t empty_matrix(){
	matrix_t out;
	memset(&out,0,sizeof(matrix_t));
	return out;
}

/*******************************************************************************
* matrix_t duplicate_matrix(matrix_t A)
*
* copy information of one matrix to a new memory location 
*******************************************************************************/
matrix_t duplicate_matrix(matrix_t A){
	int i,j;
	matrix_t out;
	if(!A.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return out;
	}
	out = create_matrix_fast(A.rows,A.cols);
	for(i=0;i<A.rows;i++){
		for(j=0;j<A.cols;j++){
			out.data[i][j] = A.data[i][j];
		}
	}
	return out;
}


/*******************************************************************************
* matrix_t create_square_matrix(int n)
*
* 
*******************************************************************************/
matrix_t create_square_matrix(int n){
	matrix_t A = create_matrix(n,n);
	return A;
}

/*******************************************************************************
* matrix_t create_random_matrix(int rows, int cols)
*
* 
*******************************************************************************/
matrix_t create_random_matrix(int rows, int cols){
	int i,j;
	matrix_t A;
	if(rows<1 || cols<1){
		printf("error creating matrix, row & col must be >=1");
		return A;
	}
	A = create_matrix_fast(rows, cols);
	for(i=0;i<rows;i++){
		for(j=0;j<cols;j++){
			// use random 32-bit floats because it's faster to generate
			A.data[i][j]=get_random_float();
		}
	}
	return A;
}

/*******************************************************************************
* matrix_t create_identity_matrix(int dim)
*
* 
*******************************************************************************/
matrix_t create_identity_matrix(int dim){
	int i;
	matrix_t A;
	if(dim<1){
		printf("error creating matrix, dim must be >=1");
		return A;
	}
	A = create_square_matrix(dim);
	for(i=0;i<dim;i++){
		A.data[i][i]=1.0;
	}
	return A;
}

/*******************************************************************************
* matrix_t create_diagonal_matrix(vector_t v)
*
* 
*******************************************************************************/
matrix_t create_diagonal_matrix(vector_t v){
	int i;
	matrix_t A;
	if(!v.initialized){
		printf("error creating matrix, vector_t v not initialized");
		return A;
	}
	A = create_square_matrix(v.len);
	for(i=0;i<v.len;i++){
		A.data[i][i]=v.data[i];
	}
	return A;
}

/*******************************************************************************
* matrix_t create_matrix_of_ones(int dim)
*
* 
*******************************************************************************/
matrix_t create_matrix_of_ones(int rows, int cols){
	int i,j;
	matrix_t A;
	if(rows<1){
		printf("error creating matrix, dim must be >=1");
		return empty_matrix();
	}
	if(cols<1){
		printf("error creating matrix, dim must be >=1");
		return empty_matrix();
	}
	A = create_matrix_fast(rows, cols);
	for(i=0;i<rows;i++){
		for(j=0;j<cols;j++){
			A.data[i][j]=1.0;
		}
	}
	return A;
}

/*******************************************************************************
* int set_matrix_entry(matrix_t* A, int row, int col, double val)
*
* 
*******************************************************************************/
int set_matrix_entry(matrix_t* A, int row, int col, double val){
	if(A==NULL){
		printf("ERROR: matrix is null pointer\n");
		return -1;
	}
	if(!A->initialized){
		printf("ERROR: A not initialized yet\n");
		return -1;
	}
	if(row<0 || row>=A->rows){
		printf("ERROR: row out of bounds\n");
		return -1;
	}
	if(col<0 || col>=A->cols){
		printf("ERROR: col out of bounds\n");
		return -1;
	}
	A->data[row][col] = val;
	return 0;
}

/*******************************************************************************
* double get_matrix_entry(matrix_t A, int row, int col)
*
* 
*******************************************************************************/
double get_matrix_entry(matrix_t A, int row, int col){
	if(!A.initialized){
		printf("ERROR: A not initialized yet\n");
		return -1;
	}
	if(row<0 || row>=A.rows){
		printf("ERROR: row out of bounds\n");
		return -1;
	}
	if(col<0 || col>=A.cols){
		printf("ERROR: col out of bounds\n");
		return -1;
	}
	return  A.data[row][col];
}

/*******************************************************************************
* void print_matrix(matrix_t A)
*
* 
*******************************************************************************/
void print_matrix(matrix_t A){
	int i,j;
	if(A.initialized!=1){
		printf("ERROR: matrix not initialized yet\n");
		return;
	}
	if(A.rows<1 || A.cols<1){
		printf("ERROR: rows and cols must be >=1\n");
		return;
	}
	for(i=0;i<A.rows;i++){
		for(j=0;j<A.cols;j++){
			printf("%7.4f  ",A.data[i][j]);
		}	
		printf("\n");
		fflush(stdout);
	}
	return;
}

/*******************************************************************************
* void print_matrix_sci_notation(matrix_t A)
*
* 
*******************************************************************************/
void print_matrix_sci_notation(matrix_t A){
	int i,j;
	if(!A.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return;
	}
	for(i=0;i<A.rows;i++){
		for(j=0;j<A.cols;j++){
			printf("%11.4e  ",A.data[i][j]);
		}	
		printf("\n");
	}
	return;
}

/*******************************************************************************
* matrix_t multiply_matrices(matrix_t A, matrix_t B)
*
* 
*******************************************************************************/
matrix_t multiply_matrices(matrix_t A, matrix_t B){
	int i,j,k;
	matrix_t out;
	if(!A.initialized||!B.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return empty_matrix();
	}
	if (A.cols != B.rows){
		printf("ERROR: Invalid matrix sizes");
		return empty_matrix();
	}
	out = create_matrix(A.rows, B.cols);
	for(i=0;i<(A.rows);i++){
		for(j=0;j<(B.cols);j++){
			for(k=0;k<(A.cols);k++){
				// do the matrix multiplication
				out.data[i][j] += A.data[i][k]*B.data[k][j];
			}
		}
	}
	return out;
}

/*******************************************************************************
* int matrix_times_scalar(matrix_t* A, double s)
*
* 
*******************************************************************************/
int matrix_times_scalar(matrix_t* A, double s){
	int i,j;
	if(!A->initialized){
		printf("ERROR: matrix not initialized yet\n");
		return -1;
	}
	for(i=0;i<(A->rows);i++){
		for(j=0;j<(A->cols);j++){
			A->data[i][j] = s*A->data[i][j];
		}
	}
	return 0;
}

/*******************************************************************************
* matrix_t add_matrices(matrix_t A, matrix_t B)
*
* 
*******************************************************************************/
matrix_t add_matrices(matrix_t A, matrix_t B){
	int i,j;
	matrix_t out;
	if(!A.initialized||!B.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return empty_matrix();
	}
	if ((A.rows != B.rows)||(A.cols != B.cols)){
		printf("ERROR: trying to add matrices with mismatched dimensions\n");
		return empty_matrix();
	}
	out = create_matrix_fast(A.rows, A.cols);
	for(i=0;i<(A.rows);i++){
		for(j=0;j<(A.cols);j++){
			out.data[i][j] = A.data[i][j] + B.data[i][j];
		}
	}
	return out;
}

/*******************************************************************************
* int transpose_matrix(matrix_t* A)
*
* 
*******************************************************************************/
int transpose_matrix(matrix_t* A){
	int i,j;
	if(!A->initialized){
		printf("ERROR: matrix not initialized yet\n");
		return -1;
	}
	// shortcut for 1x1 matrix
	if(A->rows==1 && A->cols==1) return 0;
	// allocate memory for new A, easier than doing it in place
	// since A will change size if non-square
	matrix_t temp = create_matrix(A->cols, A->rows);
	// fill in A
	for(i=0;i<(A->rows);i++){
		for(j=0;j<(A->cols);j++){
			temp.data[i][j] = A->data[j][i];
		}
	}
	// unallocate the original matrix A and set its data pointer to point
	// to the newly allocated memory
	destroy_matrix(A);
	*A=temp;
	return  0;
}