/*******************************************************************************
* vector.c
*
* James Strawson & Matt Atlas 2016
*******************************************************************************/

#include "../roboticscape.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h> // for memset
#include <float.h> // for DBL_MAX

#define PI (double)M_PI

/*******************************************************************************
* vector_t create_vector(int n)
*
* 
*******************************************************************************/
vector_t create_vector(int n){
	vector_t v;
	if(n<1){
		printf("error creating vector, n must be >=1");
		return empty_vector();
	}
	v.len = n;
	v.data = (double*)calloc(n, sizeof(double));
	v.initialized = 1;
	return v;
}

/*******************************************************************************
* void destroy_vector(vector_t* v)
*
* 
*******************************************************************************/
void destroy_vector(vector_t* v){
	if(v==NULL){
		printf("ERROR: Can't destroy vector, NULL pointer detected\n");
		return;
	}
	if(v->initialized==1){
		free(v->data);
	}
	memset(v, 0, sizeof(vector_t));
	return;
}

/*******************************************************************************
* vector_t empty_vector()
*
* 
*******************************************************************************/
vector_t empty_vector(){
	vector_t out;
	memset(&out, 0, sizeof(vector_t));
	return out;
}


/*******************************************************************************
* vector_t duplicate_vector(vector_t v)
*
* 
*******************************************************************************/
vector_t duplicate_vector(vector_t v){
	vector_t out;
	int i;

	if(!v.initialized){
		printf("ERROR: vector not initialized yet\n");
		return empty_vector();
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
	vector_t v;
	int i;

	if(len<1){
		printf("error creating vector, len must be >=1");
		return empty_vector();
	}

	v = create_vector(len);
	for(i=0;i<len;i++){
		v.data[i]=get_random_double();
	}
	return v;
}

/*******************************************************************************
* vector_t create_vector_of_ones(int len)
*
* 
*******************************************************************************/
vector_t create_vector_of_ones(int len){
	vector_t v;
	int i;

	if(len<1){
		printf("error creating vector, len must be >=1");
		return empty_vector();
	}

	v = create_vector(len);
	for(i=0;i<len;i++){
		v.data[i]=1;
	}
	return v;
}

/*******************************************************************************
* vector_t create_vector_from_array(double* array, int len)
*
* 
*******************************************************************************/
vector_t create_vector_from_array(double* array, int len){
	vector_t v;
	if(len<1){
		printf("ERROR: len must be greater than 0\n");
		return empty_vector();
	}
	v = create_vector(len);
	int i;
	for(i=0;i<len;i++){
		v.data[i] = array[i];
	}
	return v;
}

/*******************************************************************************
* int set_vector_entry(vector_t* v, int pos, double val)
*
* 
*******************************************************************************/
int set_vector_entry(vector_t* v, int pos, double val){
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
* double get_vector_entry(vector_t v, int pos)
*
* 
*******************************************************************************/
double get_vector_entry(vector_t v, int pos){
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
		printf("%7.4f  ",v.data[i]);
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
* int vector_times_scalar(vector_t* v, double s)
*
* 
*******************************************************************************/
int vector_times_scalar(vector_t* v, double s){
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
* double vector_norm(vector_t v, double p)
*
* Just like the matlab norm(v,p) function, returns the vector norm defined by
* sum(abs(v)^p)^(1/p), where p is any positive real value.
* for infinity and -infinity norms see vector_max and vector_min
*******************************************************************************/
double vector_norm(vector_t v, double p){
	double norm = 0;
	int i;

	if(!v.initialized){
		printf("ERROR: vector not initialized yet\n");
		return -1;
	}
	if(p<=0.0){
		printf("ERROR: p must be a positive real value\n");
		return -1;
	}

	// sum each term
	for(i=0;i<v.len;i++) norm+=pow(fabs(v.data[i]),p);
	
	norm = pow(norm,(1.0/p));
	return norm;
}

/*******************************************************************************
* int vector_max(vector_t v)
*
* returns the index of the maximum value in v. 
* This is the equalivalent to the infinity norm.
*******************************************************************************/
int vector_max(vector_t v){
	int i, index;
	double tmp = -DBL_MAX;

	if(!v.initialized){
		printf("ERROR: vector not initialized yet\n");
		return -1;
	}

	index = 0;
	for(i=0; i<v.len; i++){
		if(v.data[i]>tmp){
			index=i;
			tmp=v.data[i];
		}
	}

	return index;
}


/*******************************************************************************
* int vector_min(vector_t v)
*
* returns the index of the maximum value in v.
* This is the equalivalent to the infinity norm.
*******************************************************************************/
int vector_min(vector_t v){
	int i, index;
	double tmp = DBL_MAX;

	if(!v.initialized){
		printf("ERROR: vector not initialized yet\n");
		return -1;
	}

	index = 0;
	for(i=0; i<v.len; i++){
		if(v.data[i]<tmp){
			index=i;
			tmp=v.data[i];
		}
	}

	return index;
}

/*******************************************************************************
* double standard_deviation(vector_t v)
*
* 
*******************************************************************************/
double standard_deviation(vector_t v){
	int i;
	double mean, mean_sqr;
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

/*******************************************************************************
* double vector_mean(vector_t v)
*
* 
*******************************************************************************/
double vector_mean(vector_t v){
	int i;
	double sum = 0;

	if(v.initialized != 1){
		printf("ERROR: vector not initialied\n");
		return -1;
	}
	// calculate mean
	for(i=0;i<v.len;i++){
		sum += v.data[i];
	}
	return sum/ v.len;
}




/*******************************************************************************
* vector_t vector_projection(vector_t v, vector_t e)
*
* Projects vector v onto e
*******************************************************************************/
vector_t vector_projection(vector_t v, vector_t e){
	int i;
	double factor;
	vector_t out = empty_vector();
	
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
	matrix_t out = empty_matrix();
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
* double vector_dot_product(vector_t v1, vector_t v2)
*
* 
*******************************************************************************/
double vector_dot_product(vector_t v1, vector_t v2){
	double out = 0.0;
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
		out += (v1.data[i] * v2.data[i]);
	}
	return out;
}

/*******************************************************************************
* vector_t vector_cross_product(vector_t v1, vector_t v2)
*
* 
*******************************************************************************/
vector_t vector_cross_product(vector_t v1, vector_t v2){
	vector_t out = empty_vector();
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


