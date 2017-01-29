/*******************************************************************************
* rc_vector.c
*
* James Strawson & Matt Atlas 2016
*******************************************************************************/

#include "rc_algebra_common.h"

/*******************************************************************************
* int rc_alloc_vector(rc_vector_t* v, int length)
*
* Allocates memory for vector v to have specified length. If v is initially the
* right length then nothing is done and the data in v is preserved. If v is
* uninitialized or of the wrong length then any existing memory is freed and new
* memory is allocated, helping to prevent accidental memory leaks. The contents 
* of the new vector is not guaranteed to be anything in particular.
* Returns 0 if successful, otherwise returns -1. Will only be unsuccessful if 
* length is invalid or there is insufficient memory available.
*******************************************************************************/
int rc_alloc_vector(rc_vector_t* v, int length){
	// sanity checks
	if(unlikely(length<1)){
		fprintf(stderr,"ERROR in rc_alloc_vector, length must be >=1\n");
		return -1;
	}
	if(unlikely(v==NULL)){
		fprintf(stderr,"ERROR in rc_alloc_vector, received NULL pointer\n");
		return -1;
	}
	// if v is already allocated and of the right size, nothing to do!
	if(v->initialized && v->len==length) return 0;
	// free any old memory 
	rc_free_vector(v);
	// allocate contiguous memory for the vector
	v->d = (float*)malloc(length*sizeof(float));
	if(unlikely(v->d==NULL)){
		fprintf(stderr,"ERROR in rc_alloc_vector, not enough memory\n");
		return -1;
	}
	v->len = length;
	v->initialized = 1;
	return 0;
}

/*******************************************************************************
* int rc_free_vector(rc_vector_t* v)
*
* Frees the memory allocated for vector v and importantly sets the length and
* initialized flag of the rc_vector_t struct to 0 to indicate to other functions
* that v no longer points to allocated memory and cannot be used until more
* memory is allocated such as with rc_alloc_vector or rc_vector_zeros.
* Returns 0 on success. Will only fail and return -1 if it is passed a NULL
* pointer.
*******************************************************************************/
int rc_free_vector(rc_vector_t* v){
	if(unlikely(v==NULL)){
		fprintf(stderr,"ERROR rc_free_vector, received NULL pointer\n");
		return -1;
	}
	// free memory
	if(v->initialized)free(v->d);
	// zero out the struct
	*v = rc_empty_vector();
	return 0;
}

/*******************************************************************************
* rc_vector_t rc_empty_vector()
*
* Returns an rc_vector_t with no allocated memory and the initialized flag set
* to 0. This is useful for initializing vectors when they are declared since
* local variables declared in a function without global variable scope in C are
* not guaranteed to be zeroed out which can lead to bad memory pointers and 
* segfaults if not handled carefully. We recommend initializing all
* vectors with this function before using rc_alloc_matrix or any other function.
*******************************************************************************/
rc_vector_t rc_empty_vector(){
	rc_vector_t out;
	out.d = NULL;
	out.len = 0;
	out.initialized = 0;
	return out;
}

/*******************************************************************************
* int rc_vector_zeros(rc_vector_t* v, int length)
*
* Resizes vector v and allocates memory for a vector with specified length.
* The new memory is pre-filled with zeros. Any existing memory allocated for v
* is freed if necessary to avoid memory leaks.
* Returns 0 on success or -1 on error.
*******************************************************************************/
int rc_vector_zeros(rc_vector_t* v, int length){
	if(unlikely(length<1)){
		fprintf(stderr,"ERROR in rc_vector_zeros, length must be >=1\n");
		return -1;
	}
	if(unlikely(v==NULL)){
		fprintf(stderr,"ERROR in rc_vector_zeros, received NULL pointer\n");
		return -1;
	}
	// free any old memory 
	rc_free_vector(v);
	// allocate contiguous zeroed-out memory for the vector
	v->d = (float*)calloc(length,sizeof(float));
	if(unlikely(v->d==NULL)){
		fprintf(stderr,"ERROR in rc_vector_zeros, not enough memory\n");
		return -1;
	}
	v->len = length;
	v->initialized = 1;
	return 0;
}

/*******************************************************************************
* int rc_vector_ones(rc_vector_t* v, int length)
*
* Resizes vector v and allocates memory for a vector with specified length.
* The new memory is pre-filled with floating-point ones. Any existing memory
* allocated for v is freed if necessary to avoid memory leaks.
* Returns 0 on success or -1 on error.
*******************************************************************************/
int rc_vector_ones(rc_vector_t* v, int length){
	int i;
	if(unlikely(rc_alloc_vector(v, length))){
		fprintf(stderr,"ERROR in rc_vector_ones, failed to allocate vector\n");
		return -1;
	}
	for(i=0;i<length;i++) v->d[i] = 1.0f;
	return 0;
}

/*******************************************************************************
* int rc_random_vector(rc_vector_t* v, int length)
*
* Resizes vector v and allocates memory for a vector with specified length.
* The new memory is pre-filled with random floating-point values between -1.0f
* and 1.0f. Any existing memory allocated for v is freed if necessary to avoid 
* memory leaks.
* Returns 0 on success or -1 on error.
*******************************************************************************/
int rc_random_vector(rc_vector_t* v, int length){
	int i;
	if(unlikely(rc_alloc_vector(v, length))){
		fprintf(stderr,"ERROR rc_random_vector, failed to allocate vector\n");
		return -1;
	}
	for(i=0;i<length;i++) v->d[i]=rc_get_random_float();
	return 0;
}

/*******************************************************************************
* int rc_vector_fibonnaci(rc_vector_t* v, int length)
*
* Creates a vector of specified length populated with the fibonnaci sequence.
* Returns 0 on success or -1 on error.
*******************************************************************************/
int rc_vector_fibonnaci(rc_vector_t* v, int length){
	int i;
	if(unlikely(rc_alloc_vector(v, length))){
		fprintf(stderr,"ERROR rc_vector_fibonnaci, failed to allocate vector\n");
		return -1;
	}
	v->d[0]=1.0f;
	if(length>1) v->d[1]=1.0f;
	for(i=2;i<length;i++) v->d[i]=v->d[i-1]+v->d[i-2];
	return 0;
}


/*******************************************************************************
* int rc_vector_from_array(rc_vector_t* v, float* ptr, int length)
*
* Sometimes you will have a normal C-array of floats and wish to convert to 
* rc_vector_t format for use with the other linear algebra functions.
* This function duplicates the contents of an array of floats into vector v and
* ensures v is sized correctly. Existing data in v (if any) is freed and lost.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_vector_from_array(rc_vector_t* v, float* ptr, int length){
	// sanity check pointer
	if(unlikely(ptr==NULL)){
		fprintf(stderr,"ERROR in rc_vector_from_array, received NULL pointer\n");
		return -1;
	}
	// make sure there is enough space in v
	if(unlikely(rc_alloc_vector(v, length))){
		fprintf(stderr,"ERROR in rc_vector_from_array, failed to allocate vector\n");
		return -1;
	}
	// duplicate memory over
	memcpy(v->d, ptr, length*sizeof(float));
	return 0;
}

/*******************************************************************************
* int rc_duplicate_vector(rc_vector_t a, rc_vector_t* b)
*
* Allocates memory for a duplicate of vector a and copies the contents into
* the new vector b. Simply making a copy of the rc_vector_t struct is not
* sufficient as the rc_vector_t struct simply contains a pointer to the memory
* allocated to contain the contents of the vector. rc_duplicate_vector sets b
* to be a new rc_vector_t with a pointer to freshly-allocated memory.
* Returns 0 on success or -1 on error.
*******************************************************************************/
int rc_duplicate_vector(rc_vector_t a, rc_vector_t* b){
	// sanity check
	if(unlikely(!a.initialized)){
		fprintf(stderr,"ERROR in rc_duplicate_vector, a not initialized\n");
		return -1;
	}
	// make sure there is enough space in b
	if(unlikely(rc_alloc_vector(b, a.len))){
		fprintf(stderr,"ERROR in rc_duplicate_vector, failed to allocate vector\n");
		return -1;
	}
	// copy memory over
	memcpy(b->d, a.d, a.len*sizeof(float));
	return 0;
}


/*******************************************************************************
* int rc_set_vector_entry(rc_vector_t* v, int pos, float val)
*
* Sets the entry of vector 'v' in position 'pos' to 'val' where the position is
* zero-indexed. In practice this is never used as it is much easier for the user
* to set values directly with this code:
*
* v.d[pos]=val;
*
* However, we provide this function for completeness. It is not strictly
* necessary for v to be provided as a pointer as a copy of the struct v
* would also contain the correct pointer to the original vector's allocated 
* memory. However, in this library we use the convention of passing an 
* rc_vector_t struct or rc_matrix_struct as a pointer when its data is to be 
* modified by the function, and as a normal argument when it is only to be read 
* by the function. 
* Returns 0 on success or -1 on error.
*******************************************************************************/
int rc_set_vector_entry(rc_vector_t* v, int pos, float val){
	if(unlikely(v==NULL)){
		fprintf(stderr,"ERROR in rc_set_vector_entry, received NULL pointer\n");
		return -1;
	}
	if(unlikely(!v->initialized)){
		fprintf(stderr,"ERROR in rc_set_vector_entry, v not initialized yet\n");
		return -1;
	}
	if(unlikely(pos<0 || pos>=v->len)){
		fprintf(stderr,"ERROR in rc_set_vector_entry, position out of bounds\n");
		return -1;
	}
	v->d[pos] = val;
	return 0;
}

/*******************************************************************************
* float rc_get_vector_entry(rc_vector_t v, int pos)
*
* Returns the entry of vector 'v' in position 'pos' where the position is
* zero-indexed. Returns -1.0f on failure and prints an error message to stderr.
* In practice this is never used as it is much easier for the user to read
* values directly with this code:
*
* val = v.d[pos];
*
* However, we provide this function for completeness. It also provides sanity
* checks to avoid possible segfaults.
*******************************************************************************/
float rc_get_vector_entry(rc_vector_t v, int pos){
	if(unlikely(!v.initialized)){
		fprintf(stderr,"ERROR in rc_get_vector_entry, v not initialized yet\n");
		return -1.0f;
	}
	if(unlikely(pos<0 || pos>=v.len)){
		fprintf(stderr,"ERROR in rc_get_vector_entry, position out of bounds\n");
		return -1.0f;
	}
	return v.d[pos];
}

/*******************************************************************************
* int rc_print_vector(rc_vector_t v)
*
* Prints to stdout the contents of vector v in one line. This is not advisable
* for extremely long vectors but serves for quickly debugging or printing 
* results. It prints 4 decimal places with padding for a sign. We recommend 
* rc_print_vector_sci() for very small or very large numbers where scientific
* notation would be more appropriate. Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_print_vector(rc_vector_t v){
	int i;
	if(unlikely(!v.initialized)){
		fprintf(stderr,"ERROR in rc_print_vector, vector not initialized yet\n");
		return -1;
	}
	for(i=0;i<v.len;i++) printf("%7.4f  ",v.d[i]);
	printf("\n");
	return 0;
}

/*******************************************************************************
* int rc_print_vector_sci(rc_vector_t v)
*
* Prints to stdout the contents of vector v in one line. This is not advisable
* for extremely long vectors but serves for quickly debugging or printing 
*******************************************************************************/
int rc_print_vector_sci(rc_vector_t v){
	int i;
	if(unlikely(!v.initialized)){
		fprintf(stderr,"ERROR in rc_print_vector_sci, vector not initialized yet\n");
		return -1;
	}
	for(i=0;i<v.len;i++) printf("%11.4e  ",v.d[i]);
	printf("\n");
	return 0;
}


/*******************************************************************************
* int rc_vector_times_scalar(rc_vector_t* v, float s)
*
* Multiplies every entry in vector v by scalar s. It is not strictly
* necessary for v to be provided as a pointer since a copy of the struct v
* would also contain the correct pointer to the original vector's allocated 
* memory. However, in this library we use the convention of passing an 
* rc_vector_t struct or rc_matrix_struct as a pointer when its data is to be 
* modified by the function, and as a normal argument when it is only to be read 
* by the function.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_vector_times_scalar(rc_vector_t* v, float s){
	int i;
	if(unlikely(!v->initialized)){
		fprintf(stderr,"ERROR in rc_vector_times_scalar, vector uninitialized\n");
		return -1;
	}
	for(i=0;i<(v->len);i++) v->d[i] *= s;
	return 0;
}


/*******************************************************************************
* float rc_vector_norm(rc_vector_t v, float p)
*
* Just like the matlab norm(v,p) function, returns the vector norm defined by
* sum(abs(v)^p)^(1/p), where p is any positive real value. Most common norms
* are the 1 norm which gives the sum of absolute values of the vector and the
* 2-norm which is the square root of sum of squares.
* for infinity and -infinity norms see vector_max and vector_min
*******************************************************************************/
float rc_vector_norm(rc_vector_t v, float p){
	float norm = 0.0f;
	int i;
	if(unlikely(!v.initialized)){
		fprintf(stderr,"ERROR in rc_vector_norm, vector not initialized yet\n");
		return -1;
	}
	if(unlikely(p<=0.0)){
		fprintf(stderr,"ERROR in rc_vector_norm, p must be a positive real value\n");
		return -1;
	}
	// shortcut for 1-norm
	if(p<1.001f && p>0.999f){
		for(i=0;i<v.len;i++) norm+=fabs(v.d[i]);
		return norm;
	}
	// shortcut for 2-norm
	if(p<2.001f && p>1.999f){
		for(i=0;i<v.len;i++) norm+=v.d[i]*v.d[i];
		return sqrt(norm);
	}
	// generic norm formula, rarely used.
	for(i=0;i<v.len;i++) norm+=pow(fabs(v.d[i]),p);
	// take the pth root
	return pow(norm,(1.0/p));
}

/*******************************************************************************
* int rc_vector_max(rc_vector_t v)
*
* Returns the index of the maximum value in v or -1 on failure. The value 
* contained in the returned index is the equivalent to the infinity norm. If the
* max value occurs multiple times then the first instance is returned.
*******************************************************************************/
int rc_vector_max(rc_vector_t v){
	int i;
	int index = 0;
	float tmp = -FLT_MAX;
	if(unlikely(!v.initialized)){
		fprintf(stderr,"ERROR in rc_vector_max, vector not initialized yet\n");
		return -1;
	}
	for(i=0;i<v.len;i++){
		if(v.d[i]>tmp){
			index = i;
			tmp = v.d[i];
		}
	}
	return index;
}


/*******************************************************************************
* int rc_vector_min(rc_vector_t v)
*
* Returns the index of the minimum value in v or -1 on failure. The value 
* contained in the returned index is the equivalent to the minus-infinity norm.
* If the min value occurs multiple times then the first instance is returned.
*******************************************************************************/
int rc_vector_min(rc_vector_t v){
	int i;
	int index = 0;
	float tmp = FLT_MAX;
	if(unlikely(!v.initialized)){
		fprintf(stderr,"ERROR in rc_vector_min, vector not initialized yet\n");
		return -1;
	}
	for(i=0; i<v.len; i++){
		if(v.d[i]<tmp){
			index=i;
			tmp=v.d[i];
		}
	}
	return index;
}

/*******************************************************************************
* float rc_std_dev(rc_vector_t v)
*
* Returns the standard deviation of the values in a vector or -1.0f on failure.
*******************************************************************************/
float rc_std_dev(rc_vector_t v){
	int i;
	float mean, mean_sqr, diff;
	if(unlikely(!v.initialized)){
		fprintf(stderr,"ERROR in rc_std_dev, vector not initialized yet\n");
		return -1.0f;
	}
	// shortcut for length 1
	if(v.len == 1) return 0.0f;
	// calculate mean
	mean = 0.0f;
	for(i=0;i<v.len;i++) mean+=v.d[i];
	mean = mean/(float)v.len;
	// calculate mean square
	mean_sqr = 0.0f;
	for(i=0;i<v.len;i++){
		diff = v.d[i]-mean;
		mean_sqr += diff*diff;
	}
	return sqrt(mean_sqr/(float)v.len);
}

/*******************************************************************************
* float rc_vector_mean(rc_vector_t v)
*
* Returns the mean (average) of all values in vector v or -1.0f on error.
*******************************************************************************/
float rc_vector_mean(rc_vector_t v){
	int i;
	float sum = 0.0f;
	if(unlikely(!v.initialized)){
		fprintf(stderr,"ERROR in rc_vector_mean, vector not initialized yet\n");
		return -1.0f;
	}
	// calculate mean
	for(i=0;i<v.len;i++) sum+=v.d[i];
	return sum/(float)v.len;
}

/*******************************************************************************
* int rc_vector_projection(rc_vector_t v, rc_vector_t e, rc_vector_t* p)
*
* Populates vector p with the projection of vector v onto e.
* Returns 0 on success, otherwise -1.
*******************************************************************************/
int rc_vector_projection(rc_vector_t v, rc_vector_t e, rc_vector_t* p){
	int i;
	float factor;
	// sanity checks
	if(unlikely(!v.initialized || !e.initialized)){
		fprintf(stderr,"ERROR in rc_vector_projection, received uninitialized vector\n");
		return -1;
	}
	if(unlikely(v.len!=e.len)){
		fprintf(stderr,"ERROR in rc_vector_projection, vectors not of same length\n");
		return -1;
	}
	if(unlikely(rc_alloc_vector(p,v.len))){
		fprintf(stderr,"ERROR in rc_vector_projection, failed to allocate p\n");
		return -1;
	}
	factor = rc_vector_dot_product(v,e)/rc_vector_dot_product(e,e);
	for(i=0;i<v.len;i++) p->d[i]=factor*e.d[i];
	return 0;
}

/*******************************************************************************
* float rc_vector_dot_product(rc_vector_t v1, rc_vector_t v2)
*
* Returns the dot product of two equal-length vectors or floating-point -1.0f
* on error.
*******************************************************************************/
float rc_vector_dot_product(rc_vector_t v1, rc_vector_t v2){
	if(unlikely(!v1.initialized || !v2.initialized)){
		fprintf(stderr,"ERROR in rc_vector_dot_product, vector uninitialized\n");
		return -1.0f;
	}
	if(unlikely(v1.len != v2.len)){
		fprintf(stderr,"ERROR in rc_vector_dot_product, dimension mismatch\n");
		return -1.0f;
	}
	return rc_mult_accumulate(v1.d,v2.d,v1.len);
}

/*******************************************************************************
* int rc_vector_outer_product(rc_vector_t v1, rc_vector_t v2, rc_matrix_t* A)
* 
* Computes v1 times v2 where v1 is a column vector and v2 is a row vector.
* Output is a matrix with same rows as v1 and same columns as v2.
* Returns 0 on success, otherwise -1.
*******************************************************************************/
int rc_vector_outer_product(rc_vector_t v1, rc_vector_t v2, rc_matrix_t* A){
	int i, j;
	if(unlikely(!v1.initialized || !v2.initialized)){
		fprintf(stderr,"ERROR in rc_vector_outer_product, vector uninitialized\n");
		return -1;
	}
	if(unlikely(rc_alloc_matrix(A,v1.len,v2.len))){
		fprintf(stderr,"ERROR in rc_vector_outer_product, failed to allocate A\n");
		return -1;
	}
	// rc_matrix_t is row-major so rows are contiguous in memory. Thus, we fill
	// in the rows of 'out' in the inner loop so writes are continuous. Both the
	// vectors are contiguous in memory and are just interpreted as a row or
	// column vector.
	for(j=0;j<v2.len;j++){
		for(i=0;i<v1.len;i++){
			A->d[j][i] = v1.d[i]*v2.d[j];
		}
	}
	return 0;
}

/*******************************************************************************
* int rc_vector_cross_product(rc_vector_t v1, rc_vector_t v2, rc_vector_t* p)
*
* Computes the cross-product of two vectors, each of length 3. The result is
* placed in vector p and and existing memory used by p is freed and lost.
* Returns 0 on success, otherwise -1.
*******************************************************************************/
int rc_vector_cross_product(rc_vector_t v1, rc_vector_t v2, rc_vector_t* p){
	// sanity checks
	if(unlikely(!v1.initialized || !v2.initialized)){
		fprintf(stderr,"ERROR in rc_vector_cross_product, vector not initialized yet.\n");
		return -1;
	}
	if(unlikely(v1.len!=3 || v2.len!=3)){
		fprintf(stderr,"ERROR in rc_vector_cross_product, vector must have length 3\n");
		return -1;
	}
	if(unlikely(rc_alloc_vector(p,3))){
		fprintf(stderr,"ERROR in rc_vector_cross_product, failed to allocate p\n");
		return -1;
	}
	p->d[0] = (v1.d[1]*v2.d[2]) - (v1.d[2]*v2.d[1]);
	p->d[1] = (v1.d[2]*v2.d[0]) - (v1.d[0]*v2.d[2]);
	p->d[2] = (v1.d[0]*v2.d[1]) - (v1.d[1]*v2.d[0]);
	return 0;	
}


/*******************************************************************************
* int rc_vector_sum(rc_vector_t v1, rc_vector_t v2, rc_vector_t* s)
*
* Populates vector s with the sum of vectors v1 and v2. Any existing memory
* allocated for s is freed and lost, new memory is allocated if necessary.
* Returns 0 on success, otherwise -1.
*******************************************************************************/
int rc_vector_sum(rc_vector_t v1, rc_vector_t v2, rc_vector_t* s){
	int i;
	// sanity checks
	if(unlikely(!v1.initialized || !v2.initialized)){
		fprintf(stderr,"ERROR in rc_vector_sum, received uninitialized vector\n");
		return -1;
	}
	if(unlikely(v1.len!=v2.len)){
		fprintf(stderr,"ERROR in rc_vector_sum, vectors not of same length\n");
		return -1;
	}
	if(unlikely(rc_alloc_vector(s,v1.len))){
		fprintf(stderr,"ERROR in rc_vector_sum, failed to allocate s\n");
		return -1;
	}
	for(i=0;i<v1.len;i++) s->d[i]=v1.d[i]+v2.d[i];
	return 0;
}

/*******************************************************************************
* int rc_vector_sum_inplace(rc_vector_t* v1, rc_vector_t v2)
*
* Adds vector v2 to v1 and leaves the result in v1. The original contents of v1
* are lost and v2 is left untouched.
* Returns 0 on success, otherwise -1.
*******************************************************************************/
int rc_vector_sum_inplace(rc_vector_t* v1, rc_vector_t v2){
	int i;
	// sanity checks
	if(unlikely(!v1->initialized || !v2.initialized)){
		fprintf(stderr,"ERROR in rc_vector_sum_inplace, received uninitialized vector\n");
		return -1;
	}
	if(unlikely(v1->len!=v2.len)){
		fprintf(stderr,"ERROR in rc_vector_sum_inplace, vectors not of same length\n");
		return -1;
	}
	for(i=0;i<v1->len;i++) v1->d[i]+=v2.d[i];
	return 0;
}
