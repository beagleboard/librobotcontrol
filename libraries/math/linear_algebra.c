/*******************************************************************************
* linear_algebra.c
*
* James Strawson & Matt Atlas 2016
*******************************************************************************/

#include "../robotics_cape.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

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
	A.data = (float**)malloc(rows*sizeof(float*));
	void* ptr = calloc(rows*cols, sizeof(float));
	A.data[0] = (float*)ptr;
	// manually fill in the pointer to each row
	for (i=1; i<rows; i++){
		A.data[i] = (float*)(ptr + i*cols*sizeof(float));
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
	if(A->initialized==1 && A->rows>0 && A->cols>0){
		free(A->data[0]);
		free(A->data);
	}
	A->data = 0;
	A->rows = 0;
	A->cols = 0;
	A->initialized = 0;
	return;
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
* int set_matrix_entry(matrix_t* A, int row, int col, float val)
*
* 
*******************************************************************************/
int set_matrix_entry(matrix_t* A, int row, int col, float val){
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
* float get_matrix_entry(matrix_t A, int row, int col)
*
* 
*******************************************************************************/
float get_matrix_entry(matrix_t A, int row, int col){
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
* vector_t create_vector(int n)
*
* 
*******************************************************************************/
vector_t create_vector(int n){
	vector_t v;
	if(n<1){
		printf("error creating vector, n must be >=1");
		return v;
	}
	v.len = n;
	v.data = (float*)calloc(n, sizeof(float));
	v.initialized = 1;
	return v;
}

/*******************************************************************************
* void destroy_vector(vector_t* v)
*
* 
*******************************************************************************/
void destroy_vector(vector_t* v){
	if(v->initialized==1){
		free(v->data);
	}
	v->len = 0;
	v->initialized = 0;
	return;
}

/*******************************************************************************
* vector_t duplicate_vector(vector_t v)
*
* 
*******************************************************************************/
vector_t duplicate_vector(vector_t v){
	int i;
	vector_t out;
	if(!v.initialized){
		printf("ERROR: vector not initialized yet\n");
		return out;
	}
	out = create_vector(v.len);
	for(i=0;i<v.len;i++){
		out.data[i] = v.data[i];
	}
	return out;
}

/*******************************************************************************
* vector_t create_random_vector(int len)
*
* 
*******************************************************************************/
vector_t create_random_vector(int len){
	int i;
	vector_t v;
	if(len<1){
		printf("error creating vector, len must be >=1");
		return v;
	}
	v = create_vector(len);
	for(i=0;i<len;i++){
		v.data[i]=get_random_float();
	}
	return v;
}

/*******************************************************************************
* vector_t create_vector_of_ones(int len)
*
* 
*******************************************************************************/
vector_t create_vector_of_ones(int len){
	int i;
	vector_t v;
	if(len<1){
		printf("error creating vector, len must be >=1");
		return v;
	}
	v = create_vector(len);
	for(i=0;i<len;i++){
		v.data[i]=1;
	}
	return v;
}

/*******************************************************************************
* vector_t create_vector_from_array(int len, float* array)
*
* 
*******************************************************************************/
vector_t create_vector_from_array(int len, float* array){
	vector_t v;
	if(len<1){
		printf("ERROR: len must be greater than 0\n");
		return v;
	}
	v = create_vector(len);
	int i;
	for(i=0;i<len;i++){
		v.data[i] = array[i];
	}
	return v;
}

/*******************************************************************************
* int set_vector_entry(vector_t* v, int pos, float val)
*
* 
*******************************************************************************/
int set_vector_entry(vector_t* v, int pos, float val){
	if(v==NULL){
		printf("ERROR: v is null pointer\n");
		return -1;
	}
	if(!v->initialized){
		printf("ERROR: v not initialized yet\n");
		return -1;
	}
	if(pos<0 || pos>=v->len){
		printf("ERROR: pos out of bounds\n");
		return -1;
	}
	v->data[pos] = val;
	return 0;
}

/*******************************************************************************
* float get_vector_entry(vector_t v, int pos)
*
* 
*******************************************************************************/
float get_vector_entry(vector_t v, int pos){
	if(!v.initialized){
		printf("ERROR: v not initialized yet\n");
		return -1;
	}
	if(pos<0 || pos>=v.len){
		printf("ERROR: pos out of bounds\n");
		return -1;
	}
	return v.data[pos];
}

/*******************************************************************************
* void print_vector(vector_t v)
*
* 
*******************************************************************************/
void print_vector(vector_t v){
	int i;
	if(!v.initialized){
		printf("ERROR: vector not initialized yet\n");
		return;
	}
	for(i=0;i<v.len;i++){
		printf("%7.3f  ",v.data[i]);
	}
	printf("\n");
	return;
}	

/*******************************************************************************
* void print_vector_sci_notation(vector_t v)
*
* 
*******************************************************************************/
void print_vector_sci_notation(vector_t v){
	int i;
	if(!v.initialized){
		printf("ERROR: vector not initialized yet\n");
		return;
	}
	for(i=0;i<v.len;i++){
		printf("%.4e  ",v.data[i]);
	}
	printf("\n");
	return;
}	

/*******************************************************************************
* int multiply_matrices(matrix_t A, matrix_t B, matrix_t* out)
*
* 
*******************************************************************************/
int multiply_matrices(matrix_t A, matrix_t B, matrix_t* out){
	int i,j,k;
	float sum = 0;
	if(!A.initialized||!B.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return -1;
	}
	if (A.cols != B.rows){
		printf("ERROR: Invalid matrix sizes");
		return -1;
	}
	*out = create_matrix(A.rows, B.cols);	
	for(i=0;i<(A.rows);i++){
		for(j=0;j<(B.cols);j++){	
			for(k=0;k<(A.cols);k++){
				// do the matrix multiplication
				sum = sum + A.data[i][k]*B.data[k][j];
			}
			// save mult sum to new location
			out->data[i][j] = sum;
			sum = 0; 	// re-initialize sum for next loop
		}
	}
	return 0;
}

/*******************************************************************************
* int matrix_times_scalar(matrix_t* A, float s)
*
* 
*******************************************************************************/
int matrix_times_scalar(matrix_t* A, float s){
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
* int vector_times_scalar(vector_t* v, float s)
*
* 
*******************************************************************************/
int vector_times_scalar(vector_t* v, float s){
	int i;
	if(!v->initialized){
		printf("ERROR: vector not initialized yet\n");
		return -1;
	}
	for(i=0;i<(v->len);i++){	
		v->data[i] = s*v->data[i];
	}
	return 0;
}

/*******************************************************************************
* vector_t matrix_times_col_vec(matrix_t A, vector_t v)
*
* 
*******************************************************************************/
vector_t matrix_times_col_vec(matrix_t A, vector_t v){
	int i,j;
	vector_t out;
	if(!A.initialized || !v.initialized){
		printf("ERROR: matrix or vector not initialized yet\n");
		return out;
	}
	if(A.cols != v.len){
		printf("ERROR: dimensions do not match\n");
		return out;
	}
	out = create_vector(A.rows);
	
	for(i=0;i<A.rows;i++){
		for(j=0;j<A.cols;j++){	
			out.data[i] += v.data[j]*A.data[i][j];
		}
	}
	return out;
}

/*******************************************************************************
* vector_t row_vec_times_matrix(vector_t v, matrix_t A)
*
* 
*******************************************************************************/
vector_t row_vec_times_matrix(vector_t v, matrix_t A){
	int i,j;
	vector_t out;
	if(!A.initialized || !v.initialized){
		printf("ERROR: matrix or vector not initialized yet\n");
		return out;
	}
	if(A.rows != v.len){
		printf("ERROR: dimensions do not match\n");
		return out;
	}
	
	out = create_vector(A.cols);
	for(i=0;i<A.cols;i++){
		for(j=0;j<A.rows;j++){	
			out.data[i] += v.data[j]*A.data[j][i];
		}
	}
	return out;
}



/*******************************************************************************
* int add_matrices(matrix_t A, matrix_t B, matrix_t* out)
*
* 
*******************************************************************************/
int add_matrices(matrix_t A, matrix_t B, matrix_t* out){
	int i,j;
	if(!A.initialized||!B.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return -1;
	}
	if ((A.rows != B.rows)||(A.cols != B.cols)){
		printf("Invalid matrix sizes");
		return -1;
	}
	*out = create_matrix(A.rows, A.cols);
	for(i=0;i<(A.rows);i++){
		for(j=0;j<(A.cols);j++){	
			out->data[i][j] = A.data[i][j] + B.data[i][j];
		}
	}
	return 0;
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


/*******************************************************************************
* float vector_norm(vector_t v)
*
* Returns the L2-Norm of a vector. This is also commonly known as the vector
* magnitude or length.
*******************************************************************************/
float vector_norm(vector_t v){
	float out = 0;
	int i;
	if(!v.initialized){
		printf("ERROR: vector not initialized yet\n");
		return -1;
	}
	for(i=0;i<v.len;i++){
		out = out + v.data[i]*v.data[i];
	}
	return sqrt(out);
}

/*******************************************************************************
* vector_t vector_projection(vector_t v, vector_t e)
*
* Projects vector v onto e
*******************************************************************************/
vector_t vector_projection(vector_t v, vector_t e){
	int i;
	float factor;
	vector_t out;
	
	if(!v.initialized || !e.initialized){
		printf("ERROR: vectors not initialized yet\n");
		return out;
	}
	if(v.len != e.len){
		printf("ERROR: vectors not of same dimension\n");
		return out;
	}
	out = create_vector(v.len);
	factor = vector_dot_product(v,e)/vector_dot_product(e,e);
	for(i=0;i<v.len;i++){
		out.data[i] = factor * e.data[i];
	}
	return out;
}


/*******************************************************************************
* matrix_t vector_outer_product(vector_t v1, vector_t v2)
* 
* Computes v1 times v2 where v1 is a column vector and v2 is a row vector.
* Output is a matrix with same rows as v1 and same columns as v2.
*******************************************************************************/
matrix_t vector_outer_product(vector_t v1, vector_t v2){
	int i, j;
	int m = v1.len;
	int n = v2.len;
	matrix_t out;
	if(!v1.initialized || !v2.initialized){
		printf("ERROR: vectors not initialized yet\n");
		return out;
	}
	out = create_matrix(m,n);
	for(i=0;i<m;i++){
		for(j=0;j<n;j++){
			out.data[j][i] = v1.data[i]*v2.data[j];
		}
	}
	return out;
}

/*******************************************************************************
* float vector_dot_product(vector_t v1, vector_t v2)
*
* 
*******************************************************************************/
float vector_dot_product(vector_t v1, vector_t v2){
	float out;
	int i;
	if(!v1.initialized || !v2.initialized){
		printf("ERROR: vector not initialized yet\n");
		return -1;
	}
	if(v1.len != v2.len){
		printf("ERROR: vector dimensions do not match\n");
		return -1;
	}
	for(i=0; i<v1.len; i++){
		out = out + (v1.data[i] * v2.data[i]);
	}
	return out;
}

/*******************************************************************************
* vector_t cross_product_3d(vector_t v1, vector_t v2)
*
* 
*******************************************************************************/
vector_t cross_product_3d(vector_t v1, vector_t v2){
	vector_t out;
	if(!v1.initialized || !v2.initialized){
		printf("ERROR: vector not initialized yet\n");
		return out;
	}
	if((v1.len != 3) || (v2.len != 3)){
		printf("ERROR: vectors not of dimension 3\n");
		return out;
	}
	
	out = create_vector(v1.len);
	out.data[0] = (v1.data[1]*v2.data[2]) - (v1.data[2]*v2.data[1]);
	out.data[1] = (v1.data[2]*v2.data[0]) - (v1.data[0]*v2.data[2]);
	out.data[2] = (v1.data[0]*v2.data[1]) - (v1.data[1]*v2.data[0]);
	return out;	
}

/*******************************************************************************
* int polynomial_convolution(vector_t v1, vector_t v2, vector_t* out)
*
* 
*******************************************************************************/
int polynomial_convolution(vector_t v1, vector_t v2, vector_t* out){
	int m,n,i,j,k;
	if(v1.initialized!=1 || v2.initialized!=1){
		printf("ERROR: vector not initialized yet\n");
		return -1;
	}
	m = v1.len;
	n = v2.len;
	k = m+n-1;
	*out = create_vector(k);
	for(i=0;i<m;i++){
		for(j=0;j<n;j++){
			out->data[i+j] += v1.data[i] * v2.data[j];
		}
	}
	return 0;	
}

/*******************************************************************************
* int polynomial_power(vector_t v, int order, vector_t* out)
*
* 
*******************************************************************************/
int polynomial_power(vector_t v, int order, vector_t* out){
	int i;
	vector_t temp, current;
	if(order<2){
		printf("Error: polynomial power needs an order >=2\n");
		return -1;
	}
	if(v.initialized!=1){
		printf("ERROR: vector not initialized yet\n");
		return -1;
	}
	current = duplicate_vector(v);
	for(i=2;i<=order;i++){
		polynomial_convolution(current, v, &temp);
		destroy_vector(&current);
		current = duplicate_vector(temp);
		destroy_vector(&temp);
	}
	*out = current;
	return 0;
}


/*******************************************************************************
* float matrix_determinant(matrix_t A)
*
* 
*******************************************************************************/
float matrix_determinant(matrix_t A){
	int i,j,k;
	float ratio, det;
	if(!A.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return -1;
	}
	if (A.rows != A.cols){
		printf("Error: Matrix is not square\n");
		return -1;
	}
	matrix_t temp = duplicate_matrix(A);
	for(i=0;i<A.rows;i++){
        for(j=0;j<A.rows;j++){
            if(j>i){
				ratio = temp.data[j][i]/temp.data[i][i];
                for(k=0;k<A.rows;k++){
                    temp.data[j][k] = temp.data[j][k] - ratio * temp.data[i][k];
                }
            }
        }
    }
	det = 1; //storage for determinant
    for(i=0;i<A.rows;i++) det = det*temp.data[i][i];

	destroy_matrix(&temp);
    return det;  
}

/*******************************************************************************
* int LUP_decomposition(matrix_t A, matrix_t* L, matrix_t* U, matrix_t* P)
*
* LUP decomposition with partial pivoting 
*******************************************************************************/
int LUP_decomposition(matrix_t A, matrix_t* L, matrix_t* U, matrix_t* P){
	int i, j, k, m, index;
	float s1, s2, temp;
	m = A.cols;
	destroy_matrix(L);
	destroy_matrix(U);
	destroy_matrix(P);
	matrix_t Lt, Ut, Pt;
	if(!A.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return -1;
	}
	if(A.cols != A.rows){
		printf("ERROR: matrix is not square\n");
		return -1;
	}
	Lt = create_identity_matrix(m);
	Ut = create_square_matrix(m);
	Pt = create_identity_matrix(m);
	for(i=0;i<m-1;i++){
		index = i;
		for(j=i;j<m;j++){
			if(fabs(A.data[j][i]) >= fabs(A.data[index][i])){
				index = j;
			}
				
		}
		if(index != i){
			for(j=0;j<m;j++){
				temp 				= A.data[index][j];
				A.data[index][j] 	= A.data[i][j];
				A.data[i][j]		= temp;
				temp				= Pt.data[index][j];
				Pt.data[index][j]	= Pt.data[i][j];
				Pt.data[i][j]		= temp;	
			}
		}	
	}
	for(i=0;i<m;i++){
		for(j=0;j<m;j++){
			s1 = 0;
			s2 = 0;
			for(k=0;k<i;k++){
				s1 += Ut.data[k][j] * Lt.data[i][k];
			}
			for(k=0;k<j;k++){
				s2 += Ut.data[k][j] * Lt.data[i][k];
			}
			
			if(j>=i)	Ut.data[i][j] = A.data[i][j] - s1;
			
			if(i>=j)	Lt.data[i][j] = (A.data[i][j] - s2)/Ut.data[j][j];	
		}
	}
	*L = Lt;
	*U = Ut;
	*P = Pt;
	return 0;
}

/*******************************************************************************
* int QR_decomposition(matrix_t A, matrix_t* Q, matrix_t* R)
*
* 
*******************************************************************************/
int QR_decomposition(matrix_t A, matrix_t* Q, matrix_t* R){
	int i, j, k, s;
	int m = A.rows;
	int n = A.cols;
	vector_t xtemp;
	matrix_t Qt, Rt, Qi, F, temp;
	
	if(!A.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return -1;
	}
	
	destroy_matrix(Q);
	destroy_matrix(R);
	
	Qt = create_matrix(m,m);
	for(i=0;i<m;i++){					// initialize Qt as I
		Qt.data[i][i] = 1;
	}
	
	Rt = duplicate_matrix(A);			// duplicate A to Rt

	for(i=0;i<n;i++){					// iterate through columns of A
		xtemp = create_vector(m-i);		// allocate length, decreases with i
		
		for(j=i;j<m;j++){						// take col of -R from diag down
			xtemp.data[j-i] = -Rt.data[j][i]; 	
		}
		if(Rt.data[i][i] > 0)	s = -1;			// check the sign
		else					s = 1;
		xtemp.data[0] += s*vector_norm(xtemp);	// add norm to 1st element
		
		Qi = create_square_matrix(m);			// initialize Qi
		F  = create_square_matrix(m-i);			// initialize shrinking householder_matrix
		F  = householder_matrix(xtemp);			// fill in Househodor
		
		for(j=0;j<i;j++){
			Qi.data[j][j] = 1;				// fill in partial I matrix
		}
		for(j=i;j<m;j++){					// fill in remainder (householder_matrix)
			for(k=i;k<m;k++){
				Qi.data[j][k] = F.data[j-i][k-i];
			}
		}
		// multiply new Qi to old Qtemp
		temp = duplicate_matrix(Qt);
		destroy_matrix(&Qt);
		multiply_matrices(Qi,temp,&Qt);
		destroy_matrix(&temp);
		
		// same with Rtemp
		temp = duplicate_matrix(Rt);
		destroy_matrix(&Rt);
		multiply_matrices(Qi,temp,&Rt);
		destroy_matrix(&temp);
		
		// free other allocation used in this step
		destroy_matrix(&Qi);					
		destroy_matrix(&F);
		destroy_vector(&xtemp);
	}
	transpose_matrix(&Qt);
	*Q = Qt;
	*R = Rt;
	return 0;
}

/*******************************************************************************
*  int invert_matrix(matrix_t A, matrix_t* out)
*
* Invert Matrix function based on LUP decomposition and then forward and
*  backward substitution.
* 
*******************************************************************************/
int invert_matrix(matrix_t A, matrix_t* out){
	int i,j,k,m;
	matrix_t L,U,P,D,temp;
	if(!A.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return -1;
	}
	if(A.cols != A.rows){
		printf("ERROR: matrix is not square\n");
		return -1;
	}
	if(matrix_determinant(A) == 0){
		printf("ERROR: matrix is singular, not invertible\n");
		return -1;
	}
	m = A.cols;
	LUP_decomposition(A,&L,&U,&P);
	D    = create_identity_matrix(m);
	temp = create_square_matrix(m);

	for(j=0;j<m;j++){
		for(i=0;i<m;i++){
			for(k=0;k<i;k++){
				D.data[i][j] -= L.data[i][k] * D.data[k][j];
			}
		}
		for(i=m-1;i>=0;i--){				// backwards.. last to first
			temp.data[i][j] = D.data[i][j];
			for(k=i+1;k<m;k++){	
				temp.data[i][j] -= U.data[i][k] * temp.data[k][j];
			}
			temp.data[i][j] = temp.data[i][j] / U.data[i][i];
		}
	}
	// multiply by permutation matrix
	multiply_matrices(temp, P, out);		
	// free allocation	
	destroy_matrix(&temp);	
	destroy_matrix(&L);		
	destroy_matrix(&U);
	destroy_matrix(&P);
	destroy_matrix(&D);
	return 0;
}

/*******************************************************************************
* matrix_t householder_matrix(vector_t v)
*
* returns the householder reflection matrix for a given vector
*******************************************************************************/
matrix_t householder_matrix(vector_t v){
	int i, j;
	float tau;
	matrix_t out;
	if(!v.initialized){
		printf("ERROR: vector not initialized yet\n");
		return out;
	}
	out = create_square_matrix(v.len);
	for(i=0;i<v.len;i++){
		out.data[i][i] = 1;
	}
	tau = 2.0/vector_dot_product(v,v);
	for(i=0;i<v.len;i++){
		for(j=0;j<v.len;j++){
			out.data[i][j] -= tau * v.data[i]*v.data[j];
		}
	}
	return out;
}

/*******************************************************************************
* vector_t lin_system_solve(matrix_t A, vector_t b)
*
* Returns the vector x that solves Ax=b
* Thank you to  Henry Guennadi Levkin for open sourcing this routine.
*******************************************************************************/
vector_t lin_system_solve(matrix_t A, vector_t b){
	float fMaxElem, fAcc;
	int nDim,i,j,k,m;
	vector_t xout;
	if(!A.initialized || !b.initialized){
		printf("ERROR: matrix or vector not initialized yet\n");
		return xout;
	}
	if(A.cols != b.len){
		printf("ERROR: matrix dimensions do not match\n");
		return xout;
	}
	
	nDim = A.cols;
	xout = create_vector(nDim);
	matrix_t Atemp = duplicate_matrix(A); // duplicate the given matrix 
	vector_t btemp = duplicate_vector(b); // duplicate the given vector
	
	for(k=0; k<(nDim-1); k++){ // base row of matrix
		// search of line with max element
		fMaxElem = fabs( Atemp.data[k][k]);
		m = k;
		for(i=k+1; i<nDim; i++){
			if(fMaxElem < fabs(Atemp.data[i][k])){
				fMaxElem = Atemp.data[i][k];
				m = i;
			}
		}
		// permutation of base line (index k) and max element line(index m)
		if(m != k){
			for(i=k; i<nDim; i++){
				fAcc = Atemp.data[k][i];
				Atemp.data[k][i] = Atemp.data[m][i];
				Atemp.data[m][i]  = fAcc;
			}
			fAcc = btemp.data[k];
			btemp.data[k] = btemp.data[m];
			btemp.data[m] = fAcc;
		}
		if(Atemp.data[k][k]  == 0.0) return xout; // needs improvement !!!
		// triangulation of matrix with coefficients
		for(j=(k+1); j<nDim; j++){ // current row of matrix
			fAcc = - Atemp.data[j][k]  / Atemp.data[k][k];
			for(i=k; i<nDim; i++){
				Atemp.data[j][i] = Atemp.data[j][i] + fAcc*Atemp.data[k][i] ;
			}
			// free member recalculation
			btemp.data[j] = btemp.data[j] + fAcc*btemp.data[k]; 
		}
	}

	for(k=(nDim-1); k>=0; k--){
		xout.data[k] = btemp.data[k];
		for(i=(k+1); i<nDim; i++){
			xout.data[k] -= (Atemp.data[k][i]*xout.data[i]);
		}
		xout.data[k] = xout.data[k] / Atemp.data[k][k];
	}

	destroy_matrix(&Atemp);
	destroy_vector(&btemp);
	return xout;
}

/*******************************************************************************
* vector_t lin_system_solve_qr(matrix_t A, vector_t b)
*
* Gives a least-squares solution to the system AX=b for non-square A using QR.
*
*  Ax=b
* QRx=b
*  Rx=Q'b  (because Q'Q=I)
*  then solve for x with gaussian elimination
*******************************************************************************/
vector_t lin_system_solve_qr(matrix_t A, vector_t b){
	vector_t xout, temp;
	matrix_t Q,R;
	int i,k;
	if(!A.initialized || !b.initialized){
		printf("ERROR: matrix or vector not initialized yet\n");
		return xout;
	}
	// do QR decomposition
	if(QR_decomposition(A,&Q,&R)<0){
		printf("failed to perform QR decomposition on A\n");
		return xout;
	}
	// transpose Q matrix
	if(transpose_matrix(&Q)<0){
		printf("ERROR: failed to transpose Q\n");
		return xout;
	}
	// multiply through
	temp = matrix_times_col_vec(Q,b);
	destroy_matrix(&Q);
	
	// solve for x knowing R is upper triangular
	int nDim = R.cols;
	xout = create_vector(nDim);
	for(k=(nDim-1); k>=0; k--){
		xout.data[k] = temp.data[k];
		for(i=(k+1); i<nDim; i++){
			xout.data[k] -= (R.data[k][i]*xout.data[i]);
		}
		xout.data[k] = xout.data[k] / R.data[k][k];
	}
	destroy_matrix(&R);
	destroy_vector(&temp);
	
	return xout;
}

/*******************************************************************************
* int fit_ellipsoid(matrix_t points, vector_t* center, vector_t* lengths)
*
* Fits an ellipsoid to a set of points in 3D space. The principle axes of the
* fitted ellipsoid align with the global coordinate system. Therefore there are
* 6 degrees of freedom defining the ellipsoid: the x,y,z coordinates of the
* centroid and the lengths from the centroid to the surfance in each of the 3
* directions. 
*
* matrix_t points is a tall matrix with 3 columns and at least 6 rows. Each row
* must contain the xy&z components of each individual point to be fit. If only 
* 6 rows are provided, the resulting ellipsoid will be an exact fit. Otherwise
* the result is a least-squares fit to the overdefined dataset.
*
* vector_t* center is a pointer to a user-created vector which will contain the
* x,y,z position of the centroid of the fit ellipsoid.
*
* vector_t* lengths is a pointer to a user-created vector which will be 
* populated with the 3 distances from the surface to the centroid in each of the 
* 3 directions.
*******************************************************************************/
int fit_ellipsoid(matrix_t points, vector_t* center, vector_t* lengths){
	int i,p;
	matrix_t A;
	vector_t b;
	if(!points.initialized){
		printf("ERROR: matrix_t points not initialized\n");
		return -1;
	}
	if(points.cols!=3){
		printf("ERROR: matrix_t points must have 3 columns\n");
		return -1;
	}
	p = points.rows;
	if(p<6){
		printf("ERROR: matrix_t points must have at least 6 rows\n");
		return -1;
	}
	
	b = create_vector_of_ones(p);
	A = create_matrix(p,6);
	for(i=0;i<p;i++){
		A.data[i][0] = points.data[i][0] * points.data[i][0];
		A.data[i][1] = points.data[i][0];
		A.data[i][2] = points.data[i][1] * points.data[i][1];
		A.data[i][3] = points.data[i][1];
		A.data[i][4] = points.data[i][2] * points.data[i][2];
		A.data[i][5] = points.data[i][2];
	}
	
	vector_t f = lin_system_solve_qr(A,b);
	destroy_matrix(&A);
	destroy_vector(&b);
	
	// compute center 
	*center = create_vector(3);
	center->data[0] = -f.data[1]/(2*f.data[0]);
	center->data[1] = -f.data[3]/(2*f.data[2]);
	center->data[2] = -f.data[5]/(2*f.data[4]);
	
	// Solve for lengths
	A = create_square_matrix(3);
	b = create_vector(3);
	
	// fill in A
	A.data[0][0] = (f.data[0] * center->data[0] * center->data[0]) + 1.0;
	A.data[0][1] = (f.data[0] * center->data[1] * center->data[1]);
	A.data[0][2] = (f.data[0] * center->data[2] * center->data[2]);
	
	A.data[1][0] = (f.data[2] * center->data[0] * center->data[0]);
	A.data[1][1] = (f.data[2] * center->data[1] * center->data[1]) + 1.0;
	A.data[1][2] = (f.data[2] * center->data[2] * center->data[2]);
	
	A.data[2][0] = (f.data[4] * center->data[0] * center->data[0]);
	A.data[2][1] = (f.data[4] * center->data[1] * center->data[1]);
	A.data[2][2] = (f.data[4] * center->data[2] * center->data[2]) + 1.0;
	
	// fill in b
	b.data[0] = f.data[0];
	b.data[1] = f.data[2];
	b.data[2] = f.data[4];

	// solve for lengths
	vector_t scales = lin_system_solve(A, b);
	
	*lengths = create_vector(3);
	lengths->data[0] = 1.0/sqrt(scales.data[0]);
	lengths->data[1] = 1.0/sqrt(scales.data[1]);
	lengths->data[2] = 1.0/sqrt(scales.data[2]);
	// cleanup
	destroy_vector(&scales);
	destroy_matrix(&A);
	destroy_vector(&b);
	return 0;
}

/*******************************************************************************
* float standard_deviation(vector_t v)
*
* 
*******************************************************************************/
float standard_deviation(vector_t v){
	int i;
	float mean, mean_sqr;
	if(v.initialized != 1){
		printf("ERROR: vector not initialied\n");
		return -1;
	}
	if(v.len == 1) return 0;

	// calculate mean
	mean = 0;
	for(i=0;i<v.len;i++){
		mean += v.data[i];
	}
	mean = mean / v.len;
	// calculate mean square
	mean_sqr = 0;
	for(i=0;i<v.len;i++){
		mean_sqr += (v.data[i]-mean)*(v.data[i]-mean);
	}
	return sqrt(mean_sqr/v.len);
}
