/*******************************************************************************
* polynomial Manipulation
*
* We represent polynomials as a vector of coefficients with the highest power
* term on the left at vector index 0. The following polynomial manipulation
* functions are designed to behave like their counterparts in the Numerical
* Renaissance codebase.
*******************************************************************************/
#include "../roboticscape.h"
#include <math.h>
#include <stdio.h>

/*******************************************************************************
* vector_t poly_conv(vector_t v1, vector_t v2)
*
* 
*******************************************************************************/
vector_t poly_conv(vector_t v1, vector_t v2){
	vector_t out;
	int m,n,i,j,k;
	if(!v1.initialized || !v2.initialized){
		printf("ERROR: vector not initialized yet\n");
		return empty_vector();
	}
	m = v1.len;
	n = v2.len;
	k = m+n-1;
	out = create_vector(k);
	
	for(i=0;i<m;i++){
		for(j=0;j<n;j++){
			out.data[i+j] += v1.data[i] * v2.data[j];
		}
	}
	return out;	
}

/*******************************************************************************
* vector_t poly_power(vector_t v, int N)
*
* 
*******************************************************************************/
vector_t poly_power(vector_t v, int N){
	vector_t out;
	int i;
	if(!v.initialized){
		printf("ERROR: vector not initialized yet\n");
		return empty_vector();
	}
	if(N < 0){
		printf("ERROR: no negative exponents\n");
		return empty_vector();
	}
	if(N == 0){
		out = create_vector(1);
		out.data[0] = 1;
		return out;
	}	
	out = duplicate_vector(v);
	if(N == 1){
		return out;
	}
	vector_t temp;
	for(i=2;i<=N;i++){
		temp = poly_conv(out, v);
		out  = duplicate_vector(temp);
		destroy_vector(&temp);
	}
	return out;
}

/*******************************************************************************
* vector_t poly_add(vector_t a, vector_t b)
*
* add two polynomials with right justification
*******************************************************************************/
vector_t poly_add(vector_t a, vector_t b){
	vector_t out;
	int i, diff;

	if(!a.initialized){
		printf("ERROR: vector a not initialized yet\n");
		return empty_vector();
	}
	if(!b.initialized){
		printf("ERROR: vector b not initialized yet\n");
		return empty_vector();
	}

	// fill in out vector with longest input vector
	// if b is longer, replace reference a with contents of b
	if(a.len>b.len){
		out = duplicate_vector(a);
	}
	else{
		out = duplicate_vector(b);
		b=a;
	}

	// now the 'b' variable contains the shorter vector
	// itterate from the left
	diff = out.len - b.len;
	for(i=diff; i<out.len; i++)	out.data[i] += b.data[i-diff];

	return out;
}

/*******************************************************************************
* int poly_add_in_place(vector_t* a, vector_t b)
*
* adds b to a with right justification. A stays in place and new memory is
* allocated only if b is longer than a.
*******************************************************************************/
int poly_add_in_place(vector_t* a, vector_t b){
	int i, diff;

	if(!a->initialized){
		printf("ERROR: vector a not initialized yet\n");
		return -1;
	}
	if(!b.initialized){
		printf("ERROR: vector b not initialized yet\n");
		return -1;
	}

	// difference in length will be used later
	diff = b.len - a->len;

	// if b is longer than a, allocate more memory in a
	if(diff>0){
		vector_t tmp = *a;
		*a = create_vector(b.len);
		for(i=0; i<tmp.len; i++){
			a->data[i+diff] = tmp.data[i];
		}
		destroy_vector(&tmp);
	}

	// finally do the simply add
	for(i=0; i<b.len; i++)	a->data[i] += b.data[i];

	return 0;
}

/*******************************************************************************
* int poly_subtract_in_place(vector_t* a, vector_t b)
*
* subtracts b from a with right justification. a stays in place and new memory 
* is allocated only if b is longer than a.
*******************************************************************************/
int poly_subtract_in_place(vector_t* a, vector_t b){
	int i, diff;

	if(!a->initialized){
		printf("ERROR: vector a not initialized yet\n");
		return -1;
	}
	if(!b.initialized){
		printf("ERROR: vector b not initialized yet\n");
		return -1;
	}

	// difference in length will be used later
	diff = b.len - a->len;

	// if b is longer than a, allocate more memory in a
	if(diff>0){
		vector_t tmp = *a;
		*a = create_vector(b.len);
		for(i=0; i<tmp.len; i++){
			a->data[i+diff] = tmp.data[i];
		}
		destroy_vector(&tmp);
	}

	// finally do the simply subtract
	for(i=0; i<b.len; i++)	a->data[i] -= b.data[i];

	return 0;
}

/*******************************************************************************
* vector_t poly_diff(vector_t a, int d)
*
* calculate the dth derivative of the polynomial a
*******************************************************************************/
vector_t poly_diff(vector_t a, int d){
	vector_t out = empty_vector();
	int i, order;

	if(!a.initialized){
		printf("ERROR: vector a not initialized yet\n");
		return empty_vector();
	}
	if(d<=0){
		printf("ERROR: d must be greater than or equal to 0");
		return empty_vector();
	}
	// 0th derivative is easy, return the input
	if(d==0){
		out = duplicate_vector(a);
		return out;
	}
	// do one derivative
	out = create_vector(a.len-1);
	order = a.len-1;
	for(i=0; i<(a.len-1); i++){
		out.data[i] = a.data[i] * (order-i);
	}
	// if 1st derivative was requested, return now
	if(d==1) return out;

	// for higher derivatives, call recursively
	vector_t out_r = poly_diff(out, d-1);
	destroy_vector(&out);
	return out_r;
}

/*******************************************************************************
* vector_t poly_div(vector_t num, vector_t den, vector_t* remainder)
*
* divide denominator into numerator. Remainder is put in the user's remainder
* vector passed by pointer.
*******************************************************************************/
vector_t poly_div(vector_t num, vector_t den, vector_t* remainder){
	vector_t divisor;
	int i, j, diff;

	if(!num.initialized){
		printf("ERROR: numerator not initialized yet\n");
		return empty_vector();
	}
	if(!den.initialized){
		printf("ERROR: denominator not initialized yet\n");
		return empty_vector();
	}

	// difference in length, to be used later
	diff = num.len-den.len;

	if(diff<0){
		printf("ERROR: numerator must be of equal or larger length to denominator\n");
		return empty_vector();
	}

	// allocate memory for divisor and copy numerator into remainder
	divisor = create_vector(diff+1);
	*remainder = duplicate_vector(num); 

	// calculate each entry in divisor, if num and den are same length 
	// this will happen only once with i=0
	for(i=0;i<=diff;i++){
		divisor.data[i] = remainder->data[i]/den.data[i];
		// now subtract that multiple of denominator from remainder
		for(j=i; j<den.len; j++){
			remainder->data[j] -= divisor.data[i]*den.data[j];
		}
	}

	return divisor;
}


/*******************************************************************************
* vector_t poly_butter(int N, double wc)
*
* Return vector of coefficients for continuous-time Butterworth polynomial
* of order N and cutoff wc (rad/s)
*******************************************************************************/
vector_t poly_butter(int N, double wc){
	int i;
	vector_t filter = empty_vector();
	vector_t P2, P3, temp;
	if(N < 1){
		printf("ERROR: order must be > 1\n");
		return filter;
	}
	if(N > 10){
		printf("ERROR: order must be <= 10 to prevent overflow\n");
		return filter;
	}
	filter = create_vector(1);
	filter.data[0] = 1;
	P2 = create_vector(2);
	P3 = create_vector(3);
	if(N%2 == 0){
		for(i=1;i<=N/2;i++){
			P3.data[0] = 1/(wc*wc);
			P3.data[1] = -2.0*cos((2.0*i + N - 1.0)*M_PI/(2.0*N))/wc;
			P3.data[2] = 1;
			temp = duplicate_vector(filter);
			filter = poly_conv(temp,P3);
			destroy_vector(&temp);
		}
	}
	if(N%2 == 1){	
		P2.data[0] = 1/wc;
		P2.data[1] = 1;
		temp = duplicate_vector(filter);
		filter = poly_conv(temp,P2);
		destroy_vector(&temp);
		for(i=1;i<=(N-1)/2;i++){
			P3.data[0] = 1/(wc*wc);
			P3.data[1] = -2.0*cos((2.0*i + N - 1.0)*M_PI/(2.0*N))/wc;
			P3.data[2] = 1;
			temp = duplicate_vector(filter);
			filter = poly_conv(temp,P3);
			destroy_vector(&temp);
		}
	}
	destroy_vector(&P2);
	destroy_vector(&P3);
	return filter;
}


