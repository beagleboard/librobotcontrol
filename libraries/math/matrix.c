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

#define PI (double)M_PI

/*******************************************************************************
* matrix_t create_matrix(int rows, int cols)
*
* 
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
	A.data[0] = (double*)ptr;
	// manually fill in the pointer to each row
	for (i=1; i<rows; i++){
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
	out = create_matrix(A.rows,A.cols);
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
		printf("error creating matrix, row or col must be >=1");
		return A;
	}
	A = create_matrix(rows, cols);
	for(i=0;i<rows;i++){
		for(j=0;j<cols;j++){
			A.data[i][j]=get_random_double();
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
		A.data[i][i]=1;
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
matrix_t create_matrix_of_ones(int dim){
	int i,j;
	matrix_t A;
	if(dim<1){
		printf("error creating matrix, dim must be >=1");
		return A;
	}
	A = create_square_matrix(dim);
	for(i=0;i<dim;i++){
		for(j=0;j<dim;j++){
			A.data[i][j]=1;
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
			printf("%7.3f  ",A.data[i][j]);
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
* int multiply_matrices(matrix_t A, matrix_t B, matrix_t* out)
*
* 
*******************************************************************************/
matrix_t multiply_matrices(matrix_t A, matrix_t B){
	int i,j,k;
	double sum = 0;
	matrix_t out = empty_matrix();
	if(!A.initialized||!B.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return out;
	}
	if (A.cols != B.rows){
		printf("ERROR: Invalid matrix sizes");
		return out;
	}
	out = create_matrix(A.rows, B.cols);	
	for(i=0;i<(A.rows);i++){
		for(j=0;j<(B.cols);j++){	
			for(k=0;k<(A.cols);k++){
				// do the matrix multiplication
				sum = sum + A.data[i][k]*B.data[k][j];
			}
			// save mult sum to new location
			out.data[i][j] = sum;
			sum = 0; 	// re-initialize sum for next loop
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
	matrix_t out = empty_matrix();
	if(!A.initialized||!B.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return out;
	}
	if ((A.rows != B.rows)||(A.cols != B.cols)){
		printf("Invalid matrix sizes");
		return out;
	}
	out = create_matrix(A.rows, A.cols);
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
	// swap rows and cols
	matrix_t temp = create_matrix(A->cols, A->rows);
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