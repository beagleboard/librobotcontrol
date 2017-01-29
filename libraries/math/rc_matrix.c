/*******************************************************************************
* rc_matrix.c
*
* James Strawson & Matt Atlas 2016
*******************************************************************************/

#include "rc_algebra_common.h"

/*******************************************************************************
* int rc_alloc_matrix(rc_matrix_t* A, int rows, int cols)
*
* Allocates memory for matrix A to have new dimensions given by arguments rows 
* and cols. If A is initially the right size, nothing is done and the data in A
* is preserved. If A is uninitialized or of the wrong size then any existing
* memory is freed and new memory is allocated, helping to prevent accidental
* memory leaks. The contents of the new matrix is not guaranteed to be anything
* in particular.
* Returns 0 on success, otherwise -1. Will only be unsuccessful if 
* rows&cols are invalid or there is insufficient memory available.
*******************************************************************************/
int rc_alloc_matrix(rc_matrix_t* A, int rows, int cols){
	int i;
	// sanity checks
	if(unlikely(rows<1 || cols<1)){
		fprintf(stderr,"ERROR in rc_alloc_matrix, rows and cols must be >=1\n");
		return -1;
	}
	if(unlikely(A==NULL)){
		fprintf(stderr,"ERROR in rc_alloc_matrix, received NULL pointer\n");
		return -1;
	}
	// if A is already allocated and of the right size, nothing to do!
	if(A->initialized && rows==A->rows && cols==A->cols) return 0;
	// free any old memory 
	rc_free_matrix(A);
	// allocate contiguous memory for the major(row) pointers
	A->d = (float**)malloc(rows*sizeof(float*));
	if(unlikely(A->d==NULL)){
		fprintf(stderr,"ERROR in rc_alloc_matrix, not enough memory\n");
		return -1;
	}
	// allocate contiguous memory for the actual data
	void* ptr = malloc(rows*cols*sizeof(float));
	if(unlikely(ptr==NULL)){
		fprintf(stderr,"ERROR in rc_alloc_matrix, not enough memory\n");
		free(A->d);
		return -1;
	}
	// manually fill in the pointer to each row
	for(i=0;i<rows;i++) A->d[i]=(float*)(ptr+i*cols*sizeof(float));
	A->rows = rows;
	A->cols = cols;
	A->initialized = 1;
	return 0;
}

/*******************************************************************************
* int rc_free_matrix(rc_matrix_t* A)
*
* Frees the memory allocated for a matrix A and importantly sets the dimensions
* and initialized flag of the rc_matrix_t struct to 0 to indicate to other
* functions that A no longer points to allocated memory and cannot be used until
* more memory is allocated such as with rc_alloc_matrix or rc_matrix_zeros.
* Returns 0 on success. Will only fail and return -1 if it is passed a NULL
* pointer.
*******************************************************************************/
int rc_free_matrix(rc_matrix_t* A){
	if(unlikely(A==NULL)){
		fprintf(stderr,"ERROR in rc_free_matrix, received NULL pointer\n");
		return -1;
	}
	// free memory allocated for the data then the major array
	if(A->d!=NULL && A->initialized) free(A->d[0]);
	free(A->d);
	// zero out the struct
	*A = rc_empty_matrix();
	return 0;
}

/*******************************************************************************
* rc_matrix_t rc_empty_matrix()
*
* Returns an rc_matrix_t with no allocated memory and the initialized flag set
* to 0. This is useful for initializing rc_matrix_t structs when they are
* declared since local variables declared in a function without global variable
* scope in C are not guaranteed to be zeroed out which can lead to bad memory 
* pointers and segfaults if not handled carefully. We recommend initializing all
* matrices with this before using rc_alloc_matrix or any other function.
*******************************************************************************/
rc_matrix_t rc_empty_matrix(){
	rc_matrix_t out;
	// zero out the contents of the struct piecemeal instead of with memset
	// in case the struct changes in the future or if compiled with different
	// padding on other architectures.
	out.d = NULL;
	out.rows = 0;
	out.cols = 0;
	out.initialized = 0;
	return out;
}

/*******************************************************************************
* int rc_matrix_zeros(rc_matrix_t* A, int rows, int cols)
*
* Resizes matrix A and allocates memory for a matrix with specified rows &
* columns. The new memory is pre-filled with zeros. Any existing memory 
* allocated for A is freed if necessary to avoid memory leaks.
* Returns 0 on success or -1 on error.
*******************************************************************************/
int rc_matrix_zeros(rc_matrix_t* A, int rows, int cols){
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
	rc_free_matrix(A);
	// allocate contiguous memory for the major(row) pointers
	A->d = (float**)malloc(rows*sizeof(float*));
	if(unlikely(A->d==NULL)){
		fprintf(stderr,"ERROR in rc_create_matrix_zeros, not enough memory\n");
		return -1;
	}
	// allocate contiguous memory for the actual data
	void* ptr = calloc(rows*cols,sizeof(float));
	if(unlikely(ptr==NULL)){
		fprintf(stderr,"ERROR in rc_create_matrix_zeros, not enough memory\n");
		free(A->d);
		return -1;
	}
	// manually fill in the pointer to each row
	for(i=0;i<rows;i++) A->d[i]=(float*)(ptr+i*cols*sizeof(float));
	A->rows = rows;
	A->cols = cols;
	A->initialized = 1;
	return 0;
}

/*******************************************************************************
* int rc_identity_matrix(rc_matrix_t* A, int dim)
*
* Resizes A to be a square identity matrix with dimensions dim-by-dim. Any
* existing memory allocated for A is freed if necessary to avoid memory leaks.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_identity_matrix(rc_matrix_t* A, int dim){
	int i;
	if(unlikely(rc_matrix_zeros(A,dim,dim))){
		fprintf(stderr,"ERROR in rc_identity_matrix, failed to allocate matrix\n");
		return -1;
	}
	// fill in diagonal of ones
	for(i=0;i<dim;i++) A->d[i][i]=1.0f;
	return 0;
}

/*******************************************************************************
* int rc_random_matrix(rc_matrix_t* A, int rows, int cols)
*
* Resizes A to be a matrix with the specified number of rows and columns and
* populates the new memory with random numbers evenly distributed between -1.0
* and 1.0. Any existing memory allocated for A is freed if necessary to avoid
* memory leaks. Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_random_matrix(rc_matrix_t* A, int rows, int cols){
	int i;
	if(unlikely(rc_alloc_matrix(A,rows,cols))){
		fprintf(stderr,"ERROR in rc_random_matrix, failed to allocate matrix\n");
		return -1;
	}
	for(i=0;i<(A->rows*A->cols);i++) A->d[0][i]=rc_get_random_float();
	return 0;
}

/*******************************************************************************
* int rc_diag_matrix(rc_matrix_t* A, rc_vector_t v)
*
* Resizes A to be a square matrix with the same number of rows and columns as 
* vector v's length. The diagonal entries of A are then populated with the
* contents of v and the off-diagonal entries are set to 0. The original contents
* of A are freed to avoid memory leaks.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_diag_matrix(rc_matrix_t* A, rc_vector_t v){
	int i;
	// sanity check
	if(unlikely(!v.initialized)){
		fprintf(stderr,"ERROR in rc_diag_matrix, vector not initialized\n");
		return -1;
	}
	// allocate fresh zero-initialized memory for A
	if(unlikely(rc_matrix_zeros(A,v.len,v.len))){
		fprintf(stderr,"ERROR in rc_diag_matrix, failed to allocate matrix\n");
		return -1;
	}
	for(i=0;i<v.len;i++) A->d[i][i]=v.d[i];
	return 0;
}

/*******************************************************************************
* int rc_duplicate_matrix(rc_matrix_t A, rc_matrix_t* B)
*
* Makes a duplicate of the data from matrix A and places into matrix B. If B is
* already the right size then its contents are overwritten. If B is unallocated
* or is of the wrong size then the memory is freed if necessary and new memory
* is allocated to hold the duplicate of A.
* Returns 0 on success or -1 on error.
*******************************************************************************/
int rc_duplicate_matrix(rc_matrix_t A, rc_matrix_t* B){
	// sanity check
	if(unlikely(!A.initialized)){
		fprintf(stderr,"ERROR in rc_duplicate_matrix not initialized yet\n");
		return -1;
	}
	// make sure there is enough space in B
	if(unlikely(rc_alloc_matrix(B,A.rows,A.cols))){
		fprintf(stderr,"ERROR in rc_duplicate_matrix, failed to allocate memory\n");
		return -1;
	}
	// all matrix data is stored contiguously so one memcpy is sufficient
	memcpy(B->d[0],A.d[0],A.rows*A.cols*sizeof(float));
	return 0;
}

/*******************************************************************************
* int rc_set_matrix_entry(rc_matrix_t* A, int row, int col, float val)
*
* Sets the specified single entry of matrix A to 'val' where the position is
* zero-indexed at the top-left corner. In practice this is never used as it is
* much easier for the user to set values directly with this code:
*
* A.d[row][col]=val;
*
* However, we provide this function for completeness. It is not strictly
* necessary for A to be provided as a pointer since a copy of the struct A
* would also contain the correct pointer to the original matrix's allocated 
* memory. However, in this library we use the convention of passing an 
* rc_vector_t struct or rc_matrix_struct as a pointer when its data is to be 
* modified by the function, and as a normal argument when it is only to be read 
* by the function. Returns 0 on success or -1 on error.
*******************************************************************************/
int rc_set_matrix_entry(rc_matrix_t* A, int row, int col, float val){
	if(unlikely(A==NULL)){
		fprintf(stderr,"ERROR in rc_set_matrix_entry, received null pointer\n");
		return -1;
	}
	if(unlikely(!A->initialized)){
		fprintf(stderr,"ERROR in rc_set_matrix_entry, ,matrix not initialized yet\n");
		return -1;
	}
	if(unlikely(row<0 || row>=A->rows)){
		fprintf(stderr,"ERROR in rc_set_matrix_entry, row out of bounds\n");
		return -1;
	}
	if(unlikely(col<0 || col>=A->cols)){
		fprintf(stderr,"ERROR in rc_set_matrix_entry, column out of bounds\n");
		return -1;
	}
	A->d[row][col] = val;
	return 0;
}

/*******************************************************************************
* float rc_get_matrix_entry(rc_matrix_t A, int row, int col)
*
* Returns the specified single entry of matrix 'A' in position 'pos' where the
* position is zero-indexed. Returns -1.0f on failure and prints an error message
* to stderr. In practice this is never used as it is much easier for the user to
* read values directly with this code:
*
* val = A.d[row][col];
*
* However, we provide this function for completeness. It also provides sanity
* checks to avoid possible segfaults.
*******************************************************************************/
float rc_get_matrix_entry(rc_matrix_t A, int row, int col){
	if(unlikely(!A.initialized)){
		fprintf(stderr,"ERROR in rc_get_matrix_entry, ,matrix not initialized yet\n");
		return -1.0f;
	}
	if(unlikely(row<0 || row>=A.rows)){
		fprintf(stderr,"ERROR in rc_get_matrix_entry, row out of bounds\n");
		return -1.0f;
	}
	if(unlikely(col<0 || col>=A.cols)){
		fprintf(stderr,"ERROR in rc_get_matrix_entry, column out of bounds\n");
		return -1.0f;
	}
	return A.d[row][col];
}

/*******************************************************************************
* int rc_print_matrix(rc_matrix_t A)
*
* Prints the contents of matrix A to stdout in decimal notation with 4 decimal
* places. Not recommended for very large matrices as rows will typically
* linewrap if the terminal window is not wide enough.
*******************************************************************************/
int rc_print_matrix(rc_matrix_t A){
	int i,j;
	if(unlikely(!A.initialized)){
		fprintf(stderr,"ERROR in rc_print_matrix, matrix not initialized yet\n");
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

/*******************************************************************************
* void rc_print_matrix_sci(rc_matrix_t A)
*
* Prints the contents of matrix A to stdout in scientific notation with 4
* significant figures. Not recommended for very large matrices as rows will 
* typically linewrap if the terminal window is not wide enough.
*******************************************************************************/
void rc_print_matrix_sci(rc_matrix_t A){
	int i,j;
	if(unlikely(!A.initialized)){
		fprintf(stderr,"ERROR in rc_print_matrix_sci, matrix not initialized yet\n");
		return;
	}
	for(i=0;i<A.rows;i++){
		for(j=0;j<A.cols;j++){
			printf("%11.4e  ",A.d[i][j]);
		}	
		printf("\n");
	}
	return;
}

/*******************************************************************************
* int rc_matrix_times_scalar(rc_matrix_t* A, float s)
*
* Multiplies every entry in A by scalar value s. It is not strictly
* necessary for A to be provided as a pointer since a copy of the struct A
* would also contain the correct pointer to the original matrix's allocated 
* memory. However, in this library we use the convention of passing an 
* rc_vector_t struct or rc_matrix_struct as a pointer when its data is to be 
* modified by the function, and as a normal argument when it is only to be read 
* by the function. Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_matrix_times_scalar(rc_matrix_t* A, float s){
	int i;
	if(unlikely(!A->initialized)){
		fprintf(stderr,"ERROR in rc_matrix_times_scalar. matrix uninitialized\n");
		return -1;
	}
	// since A contains contiguous memory, gcc should vectorize this loop
	for(i=0;i<(A->rows*A->cols);i++) A->d[0][i] *= s;
	return 0;
}

/*******************************************************************************
* int rc_multiply_matrices(rc_matrix_t A, rc_matrix_t B, rc_matrix_t* C)
*
* Multiplies A*B=C. C is resized and its original contents are freed if 
* necessary to avoid memory leaks. Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_multiply_matrices(rc_matrix_t A, rc_matrix_t B, rc_matrix_t* C){
	int i,j;
	float* tmp;
	if(unlikely(!A.initialized||!B.initialized)){
		fprintf(stderr,"ERROR in rc_multiply_matrices, matrix not initialized\n");
		return -1;
	}
	if(unlikely(A.cols!=B.rows)) {
		fprintf(stderr,"ERROR in rc_multiply_matrices, dimension mismatch\n");
		return -1;
	}
	// if C is not initialized, allocate memory for it
	if(unlikely(rc_alloc_matrix(C,A.rows,B.cols))){
		fprintf(stderr,"ERROR in rc_multiply_matrices, can't allocate memory for C\n");
		return -1;
	}
	// allocate memory for a column of B from the stack, this is faster than 
	// malloc and the memory is freed automatically when this function returns
	// it is faster to put a column in contiguous memory before multiplying
	tmp = alloca(B.rows*sizeof(float));
	if(unlikely(tmp==NULL)){
		fprintf(stderr,"ERROR in rc_multiply_matrices, alloca failed, stack overflow\n");
		return -1;
	}
	// go through columns of B calculating columns of C left to right
	for(i=0;i<(B.cols);i++){
		// put column of B in sequential memory slot
		for(j=0;j<B.rows;j++) tmp[j]=B.d[j][i];
		// calculate each row in column i
		for(j=0;j<(A.rows);j++){
			C->d[j][i]=rc_mult_accumulate(A.d[j],tmp,B.rows);
		}
	}
	return 0;
}

/*******************************************************************************
* int rc_left_multiply_matrix_inplace(rc_matrix_t A, rc_matrix_t* B)
*
* Multiplies A*B and puts the result back in the place of B. B is resized and
* its original contents are freed if necessary to avoid memory leaks.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_left_multiply_matrix_inplace(rc_matrix_t A, rc_matrix_t* B){
	rc_matrix_t tmp = rc_empty_matrix();
	// use the normal multiply function which will allocate memory for tmp
	if(rc_multiply_matrices(A, *B, &tmp)){
		fprintf(stderr,"ERROR in rc_left_multiply_matrix_inplace, failed to multiply\n");
		rc_free_matrix(&tmp);
		return -1;
	}
	rc_free_matrix(B);
	*B=tmp;
	return 0;
}

/*******************************************************************************
* int rc_right_multiply_matrix_inplace(rc_matrix_t* A, rc_matrix_t B)
*
* Multiplies A*B and puts the result back in the place of A. A is resized and
* its original contents are freed if necessary to avoid memory leaks.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_right_multiply_matrix_inplace(rc_matrix_t* A, rc_matrix_t B){
	rc_matrix_t tmp = rc_empty_matrix();
	if(rc_multiply_matrices(*A, B, &tmp)){
		fprintf(stderr,"ERROR in rc_right_multiply_matrix_inplace, failed to multiply\n");
		rc_free_matrix(&tmp);
		return -1;
	}
	rc_free_matrix(A);
	*A=tmp;
	return 0;
}

/*******************************************************************************
* int rc_add_matrices(rc_matrix_t A, rc_matrix_t B, rc_matrix_t* C)
*
* Resizes matrix C and places the sum A+B in C. The original contents of C are
* safely freed if necessary to avoid memory leaks.
* Returns 0 on success or -1 on failure. 
*******************************************************************************/
int rc_add_matrices(rc_matrix_t A, rc_matrix_t B, rc_matrix_t* C){
	int i;
	if(unlikely(!A.initialized||!B.initialized)){
		fprintf(stderr,"ERROR in rc_add_matrices, matrix not initialized\n");
		return -1;
	}
	if(unlikely(A.rows!=B.rows || A.cols!=B.cols)){
		fprintf(stderr,"ERROR in rc_add_matrices, dimension mismatch\n");
		return -1;
	}
	// make sure C is allocated
	if(unlikely(rc_alloc_matrix(C,A.rows,A.cols))){
		fprintf(stderr,"ERROR in rc_add_matrices, can't allocate memory for C\n");
		return -1;
	}
	// since A contains contiguous memory, gcc should vectorize this loop
	for(i=0;i<(A.rows*A.cols);i++) C->d[0][i]=A.d[0][i]+B.d[0][i];
	return 0;
}

/*******************************************************************************
* int rc_add_matrices_inplace(rc_matrix_t* A, rc_matrix_t B)
*
* Adds matrix B to A and places the result in A so the original contents of A
* are lost. Use rc_add_matrices if you wish to keep the contents of both matrix
* A and B. Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_add_matrices_inplace(rc_matrix_t* A, rc_matrix_t B){
	int i;
	if(unlikely(!A->initialized||!B.initialized)){
		fprintf(stderr,"ERROR in rc_add_matrices_inplace, matrix not initialized\n");
		return -1;
	}
	if(unlikely(A->rows!=B.rows || A->cols!=B.cols)){
		fprintf(stderr,"ERROR in rc_add_matrices_inplace, dimension mismatch\n");
		return -1;
	}
	// since A contains contiguous memory, gcc should vectorize this loop
	for(i=0;i<(A->rows*A->cols);i++) A->d[0][i]+=B.d[0][i];
	return 0;
}

/*******************************************************************************
* int rc_matrix_transpose(rc_matrix_t A, rc_matrix_t* T)
*
* Resizes matrix T to hold the transposed contents of A and leaves A untouched.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_matrix_transpose(rc_matrix_t A, rc_matrix_t* T){
	int i,j;
	if(unlikely(!A.initialized)){
		fprintf(stderr,"ERROR in rc_matrix_transpose, received uninitialized matrix\n");
		return -1;
	}
	// make sure T is allocated
	if(unlikely(rc_alloc_matrix(T,A.rows,A.cols))){
		fprintf(stderr,"ERROR in rc_matrix_transpose, can't allocate memory for T\n");
		return -1;
	}
	// fill in new memory
	for(i=0;i<(A.rows);i++){
		for(j=0;j<(A.cols);j++){
			T->d[i][j] = A.d[j][i];
		}
	}
	return 0;
}

/*******************************************************************************
* int rc_matrix_transpose_inplace(rc_matrix_t* A)
*
* Transposes matrix A in place. Use as an alternative to rc_matrix_transpose
* if you no longer have need for the original contents of matrix A.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_matrix_transpose_inplace(rc_matrix_t* A){
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
	rc_matrix_t tmp = rc_empty_matrix();
	if(unlikely(rc_matrix_transpose(*A, &tmp))){
		fprintf(stderr,"ERROR in rc_transpose_matrix_inplace, can't transpose\n");
		rc_free_matrix(&tmp);
		return -1;
	}
	// free the original matrix A and set it's struct to point to the new memory
	rc_free_matrix(A);
	*A=tmp;
	return 0;
}
