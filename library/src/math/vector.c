/**
 * @headerfile math/vector.c
 *
 * @brief      A collection of hardware-accelerated linear algebra functions
 *             used heavily by the rest of the rc_math API.
 *
 *             A small rc_vector_t struct contains information about the
 *             vector's size and a pointer to where dynamically allocated memory
 *             exists that stores the actual data for the vector. Use
 *             rc_vector_alloc to dynamically allocate memory for each new
 *             vector. Then use rc_vector_free and to free the memory when you
 *             are done using it. See the remaining vector, matrix, and linear
 *             algebra functions for more details.
 *
 * @author     James Strawson
 * @date       2016
 */

#include <stdio.h>
#include <stdlib.h>	// for malloc,calloc,free
#include <string.h>	// for memcpy
#include <math.h>	// for sqrt, pow, etc
#include <float.h>	// for FLT_MAX DBL_MAX

#include <rc/math/other.h>
#include <rc/math/vector.h>
#include "algebra_common.h"


int rc_vector_alloc(rc_vector_t* v, int length)
{
	// sanity checks
	if(unlikely(length<1)){
		fprintf(stderr,"ERROR in rc_vector_alloc, length must be >=1\n");
		return -1;
	}
	if(unlikely(v==NULL)){
		fprintf(stderr,"ERROR in rc_vector_alloc, received NULL pointer\n");
		return -1;
	}
	// if v is already allocated and of the right size, nothing to do!
	if(v->initialized && v->len==length) return 0;
	// free any old memory
	rc_vector_free(v);
	// allocate contiguous memory for the vector
	v->d = (double*)malloc(length*sizeof(double));
	if(unlikely(v->d==NULL)){
		fprintf(stderr,"ERROR in rc_vector_alloc, not enough memory\n");
		return -1;
	}
	v->len = length;
	v->initialized = 1;
	return 0;
}

int rc_vector_free(rc_vector_t* v)
{
	rc_vector_t new = RC_VECTOR_INITIALIZER;
	if(unlikely(v==NULL)){
		fprintf(stderr,"ERROR rc_vector_free, received NULL pointer\n");
		return -1;
	}
	// free memory
	if(v->initialized)free(v->d);
	// zero out the struct
	*v = new;
	return 0;
}


rc_vector_t rc_vector_empty(void)
{
	rc_vector_t out = RC_VECTOR_INITIALIZER;
	return out;
}


int rc_vector_zeros(rc_vector_t* v, int length)
{
	if(unlikely(length<1)){
		fprintf(stderr,"ERROR in rc_vector_zeros, length must be >=1\n");
		return -1;
	}
	if(unlikely(v==NULL)){
		fprintf(stderr,"ERROR in rc_vector_zeros, received NULL pointer\n");
		return -1;
	}
	// free any old memory
	rc_vector_free(v);
	// allocate contiguous zeroed-out memory for the vector
	v->d = (double*)calloc(length,sizeof(double));
	if(unlikely(v->d==NULL)){
		fprintf(stderr,"ERROR in rc_vector_zeros, not enough memory\n");
		return -1;
	}
	v->len = length;
	v->initialized = 1;
	return 0;
}


int rc_vector_ones(rc_vector_t* v, int length)
{
	int i;
	if(unlikely(rc_vector_alloc(v, length))){
		fprintf(stderr,"ERROR in rc_vector_ones, failed to allocate vector\n");
		return -1;
	}
	for(i=0;i<length;i++) v->d[i] = 1.0f;
	return 0;
}


int rc_vector_random(rc_vector_t* v, int length)
{
	int i;
	if(unlikely(rc_vector_alloc(v, length))){
		fprintf(stderr,"ERROR rc_vector_random, failed to allocate vector\n");
		return -1;
	}
	for(i=0;i<length;i++) v->d[i]=rc_get_random_double();
	return 0;
}

int rc_vector_fibonnaci(rc_vector_t* v, int length)
{
	int i;
	if(unlikely(rc_vector_alloc(v, length))){
		fprintf(stderr,"ERROR rc_vector_fibonnaci, failed to allocate vector\n");
		return -1;
	}
	v->d[0]=1.0f;
	if(length>1) v->d[1]=1.0f;
	for(i=2;i<length;i++) v->d[i]=v->d[i-1]+v->d[i-2];
	return 0;
}


int rc_vector_from_array(rc_vector_t* v, double* ptr, int length)
{
	// sanity check pointer
	if(unlikely(ptr==NULL)){
		fprintf(stderr,"ERROR in rc_vector_from_array, received NULL pointer\n");
		return -1;
	}
	// make sure there is enough space in v
	if(unlikely(rc_vector_alloc(v, length))){
		fprintf(stderr,"ERROR in rc_vector_from_array, failed to allocate vector\n");
		return -1;
	}
	// duplicate memory over
	memcpy(v->d, ptr, length*sizeof(double));
	return 0;
}


int rc_vector_duplicate(rc_vector_t a, rc_vector_t* b)
{
	// sanity check
	if(unlikely(!a.initialized)){
		fprintf(stderr,"ERROR in rc_duplicate_vector, a not initialized\n");
		return -1;
	}
	// make sure there is enough space in b
	if(unlikely(rc_vector_alloc(b, a.len))){
		fprintf(stderr,"ERROR in rc_duplicate_vector, failed to allocate vector\n");
		return -1;
	}
	// copy memory over
	memcpy(b->d, a.d, a.len*sizeof(double));
	return 0;
}


int rc_vector_print(rc_vector_t v)
{
	int i;
	if(unlikely(!v.initialized)){
		fprintf(stderr,"ERROR in rc_vector_print, vector not initialized yet\n");
		return -1;
	}
	for(i=0;i<v.len;i++) printf("%7.4f  ",v.d[i]);
	printf("\n");
	return 0;
}

int rc_vector_print_sci(rc_vector_t v)
{
	int i;
	if(unlikely(!v.initialized)){
		fprintf(stderr,"ERROR in rc_vector_print_sci, vector not initialized yet\n");
		return -1;
	}
	for(i=0;i<v.len;i++) printf("%11.4e  ",v.d[i]);
	printf("\n");
	return 0;
}

int rc_vector_zero_out(rc_vector_t* v)
{
	int i;
	if(unlikely(v->initialized!=1)){
		fprintf(stderr,"ERROR in rc_vector_zero_out,vector not initialized yet\n");
		return -1;
	}
	for(i=0;i<v->len;i++)	v->d[i]=0.0;
	return 0;
}

int rc_vector_times_scalar(rc_vector_t* v, double s)
{
	int i;
	if(unlikely(!v->initialized)){
		fprintf(stderr,"ERROR in rc_vector_times_scalar, vector uninitialized\n");
		return -1;
	}
	for(i=0;i<(v->len);i++) v->d[i] *= s;
	return 0;
}


double rc_vector_norm(rc_vector_t v, double p)
{
	double norm = 0.0f;
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
	if(p<1.001 && p>0.999){
		for(i=0;i<v.len;i++) norm+=fabs(v.d[i]);
		return norm;
	}
	// shortcut for 2-norm
	if(p<2.001 && p>1.999){
		for(i=0;i<v.len;i++) norm+=v.d[i]*v.d[i];
		return sqrt(norm);
	}
	// generic norm formula, rarely used.
	for(i=0;i<v.len;i++) norm+=pow(fabs(v.d[i]),p);
	// take the pth root
	return pow(norm,(1.0/p));
}


int rc_vector_max(rc_vector_t v)
{
	int i;
	int index = 0;
	double tmp = -DBL_MAX;
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


int rc_vector_min(rc_vector_t v)
{
	int i;
	int index = 0;
	double tmp = DBL_MAX;
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


double rc_vector_std_dev(rc_vector_t v)
{
	int i;
	double mean, mean_sqr, diff;
	if(unlikely(!v.initialized)){
		fprintf(stderr,"ERROR in rc_vector_std_dev, vector not initialized yet\n");
		return -1.0f;
	}
	// shortcut for length 1
	if(v.len == 1) return 0.0f;
	// calculate mean
	mean = 0.0f;
	for(i=0;i<v.len;i++) mean+=v.d[i];
	mean = mean/(double)v.len;
	// calculate mean square
	mean_sqr = 0.0f;
	for(i=0;i<v.len;i++){
		diff = v.d[i]-mean;
		mean_sqr += diff*diff;
	}
	return sqrt(mean_sqr/(double)(v.len-1));
}


double rc_vector_mean(rc_vector_t v)
{
	int i;
	double sum = 0.0f;
	if(unlikely(!v.initialized)){
		fprintf(stderr,"ERROR in rc_vector_mean, vector not initialized yet\n");
		return -1.0f;
	}
	// calculate mean
	for(i=0;i<v.len;i++) sum+=v.d[i];
	return sum/(double)v.len;
}


int rc_vector_projection(rc_vector_t v, rc_vector_t e, rc_vector_t* p)
{
	int i;
	double factor;
	// sanity checks
	if(unlikely(!v.initialized || !e.initialized)){
		fprintf(stderr,"ERROR in rc_vector_projection, received uninitialized vector\n");
		return -1;
	}
	if(unlikely(v.len!=e.len)){
		fprintf(stderr,"ERROR in rc_vector_projection, vectors not of same length\n");
		return -1;
	}
	if(unlikely(rc_vector_alloc(p,v.len))){
		fprintf(stderr,"ERROR in rc_vector_projection, failed to allocate p\n");
		return -1;
	}
	factor = rc_vector_dot_product(v,e)/rc_vector_dot_product(e,e);
	for(i=0;i<v.len;i++) p->d[i]=factor*e.d[i];
	return 0;
}


double rc_vector_dot_product(rc_vector_t v1, rc_vector_t v2)
{
	if(unlikely(!v1.initialized || !v2.initialized)){
		fprintf(stderr,"ERROR in rc_vector_dot_product, vector uninitialized\n");
		return -1.0f;
	}
	if(unlikely(v1.len != v2.len)){
		fprintf(stderr,"ERROR in rc_vector_dot_product, dimension mismatch\n");
		return -1.0f;
	}
	return __vectorized_mult_accumulate(v1.d,v2.d,v1.len);
}


int rc_vector_cross_product(rc_vector_t v1, rc_vector_t v2, rc_vector_t* p)
{
	// sanity checks
	if(unlikely(!v1.initialized || !v2.initialized)){
		fprintf(stderr,"ERROR in rc_vector_cross_product, vector not initialized yet.\n");
		return -1;
	}
	if(unlikely(v1.len!=3 || v2.len!=3)){
		fprintf(stderr,"ERROR in rc_vector_cross_product, vector must have length 3\n");
		return -1;
	}
	if(unlikely(rc_vector_alloc(p,3))){
		fprintf(stderr,"ERROR in rc_vector_cross_product, failed to allocate p\n");
		return -1;
	}
	p->d[0] = (v1.d[1]*v2.d[2]) - (v1.d[2]*v2.d[1]);
	p->d[1] = (v1.d[2]*v2.d[0]) - (v1.d[0]*v2.d[2]);
	p->d[2] = (v1.d[0]*v2.d[1]) - (v1.d[1]*v2.d[0]);
	return 0;
}


int rc_vector_sum(rc_vector_t v1, rc_vector_t v2, rc_vector_t* s)
{
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
	if(unlikely(rc_vector_alloc(s,v1.len))){
		fprintf(stderr,"ERROR in rc_vector_sum, failed to allocate s\n");
		return -1;
	}
	for(i=0;i<v1.len;i++) s->d[i]=v1.d[i]+v2.d[i];
	return 0;
}


int rc_vector_sum_inplace(rc_vector_t* v1, rc_vector_t v2)
{
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


int rc_vector_subtract(rc_vector_t v1, rc_vector_t v2, rc_vector_t* s)
{
	int i;
	// sanity checks
	if(unlikely(!v1.initialized || !v2.initialized)){
		fprintf(stderr,"ERROR in rc_vector_substract, received uninitialized vector\n");
		return -1;
	}
	if(unlikely(v1.len!=v2.len)){
		fprintf(stderr,"ERROR in rc_vector_substract, vectors not of same length\n");
		return -1;
	}
	if(unlikely(rc_vector_alloc(s,v1.len))){
		fprintf(stderr,"ERROR in rc_vector_substract, failed to allocate s\n");
		return -1;
	}
	for(i=0;i<v1.len;i++) s->d[i]=v1.d[i]-v2.d[i];
	return 0;
}
