/**
 * @headerfile math/polynomial.h <rc/math/polynomial.h>
 *
 * @brief      Collection of polynomial manipulation functions
 *
 *             We represent polynomials as a vector of coefficients with the
 *             highest power term on the left at vector index 0. The following
 *             polynomial manipulation functions are designed to behave like
 *             their counterparts in the Numerical Renaissance codebase.
 *
 * @author     James Strawson
 * @date       2016
 */

#include <stdio.h>
#include <math.h>	// for sqrt, pow, etc

#include <rc/math/polynomial.h>

#include "algebra_common.h"


int rc_poly_print(rc_vector_t v)
{
	int i;
	static char *super[] = {"\xe2\x81\xb0", "\xc2\xb9", "\xc2\xb2",
		"\xc2\xb3", "\xe2\x81\xb4", "\xe2\x81\xb5", "\xe2\x81\xb6",
		"\xe2\x81\xb7", "\xe2\x81\xb8", "\xe2\x81\xb9"};
	if(unlikely(!v.initialized)){
		fprintf(stderr,"ERROR in rc_poly_print, vector not initialized yet\n");
		return -1;
	}
	if(unlikely(v.len>10)){
		fprintf(stderr,"ERROR in rc_poly_print, vector length must be <=10\n");
		return -1;
	}
	for(i=0;i<(v.len-2);i++){
		printf("%7.4fx%s + ",v.d[i],super[v.len-i-1]);
	}
	// penultimate coefficient with no power
	if(v.len>=2) printf("%7.4fx  + ",v.d[v.len-2]);
	// last coefficient with no x
	printf("%7.4f\n", v.d[v.len-1]);
	return 0;
}


int rc_poly_conv(rc_vector_t a, rc_vector_t b, rc_vector_t* c)
{
	int i,j;
	// sanity checks
	if(unlikely(!a.initialized || !b.initialized)){
		fprintf(stderr,"ERROR in rc_poly_conv, vector uninitialized\n");
		return -1;
	}
	if(unlikely(rc_vector_zeros(c,a.len+b.len-1))){
		fprintf(stderr,"ERROR in rc_poly_conv, failed to alloc vector\n");
		return -1;
	}
	for(i=0;i<a.len;i++){
		for(j=0;j<b.len;j++){
			c->d[i+j] += a.d[i]*b.d[j];
		}
	}
	return 0;
}


int rc_poly_power(rc_vector_t a, int n, rc_vector_t* b)
{
	int i;
	rc_vector_t tmp = RC_VECTOR_INITIALIZER;
	// sanity checks
	if(unlikely(!a.initialized)){
		fprintf(stderr,"ERROR in rc_poly_power, vector uninitialized\n");
		return -1;
	}
	if(unlikely(n<0)){
		fprintf(stderr,"ERROR in rc_poly_power, negative exponents not allowed\n");
		return -1;
	}
	// shortcut for power 0
	if(n==0){
		if(unlikely(rc_vector_alloc(b,1))){
			fprintf(stderr,"ERROR in rc_poly_power, failed to alloc vector\n");
			return -1;
		}
		b->d[0] = 1.0f;
		return 0;
	}
	// for power 1 and above we start with duplicate
	if(unlikely(rc_vector_duplicate(a,b))){
		fprintf(stderr,"ERROR in rc_poly_power, failed to duplicate vector\n");
		return -1;
	}
	// for power 1 that's all, just return
	if(n==1) return 0;
	// for higher powers we need to keep multiplying
	for(i=2;i<=n;i++){
		if(unlikely(rc_poly_conv(a,*b,&tmp))){
			fprintf(stderr,"ERROR in rc_poly_power, failed to poly_conv\n");
			rc_vector_free(&tmp);
			rc_vector_free(b);
			return -1;
		}
		rc_vector_free(b);
		*b = tmp;
		tmp = rc_vector_empty();
	}
	return 0;
}


int rc_poly_add(rc_vector_t a, rc_vector_t b, rc_vector_t* c)
{
	int i, diff;
	rc_vector_t longest;
	// sanity checks
	if(unlikely(!a.initialized || !b.initialized)){
		fprintf(stderr,"ERROR in rc_poly_add, vector uninitialized\n");
		return -1;
	}
	// fill in out vector with longest input vector
	// if b is longer, replace reference a with contents of b
	if(a.len>b.len)	longest=a;
	else{
		longest=b;
		b=a;
	}
	if(unlikely(rc_vector_duplicate(longest,c))){
		fprintf(stderr,"ERROR in rc_poly_add, failed to duplicate vector\n");
		return -1;
	}
	// now the 'b' variable contains the shorter vector
	// itterate from the left
	diff = c->len-b.len;
	for(i=diff;i<c->len;i++) c->d[i]+=b.d[i-diff];
	return 0;
}


int rc_poly_add_inplace(rc_vector_t* a, rc_vector_t b)
{
	rc_vector_t tmp = RC_VECTOR_INITIALIZER;
	// sanity checks
	if(unlikely(!a->initialized || !b.initialized)){
		fprintf(stderr,"ERROR in rc_poly_add_in_place, vector uninitialized\n");
		return -1;
	}
	if(unlikely(rc_poly_add(*a,b,&tmp))){
		fprintf(stderr,"ERROR in rc_poly_add_in_place, add failed\n");
		return -1;
	}
	rc_vector_free(a);
	*a=tmp;
	return 0;
}


int rc_poly_subtract(rc_vector_t a, rc_vector_t b, rc_vector_t* c)
{
	int i, diff;
	rc_vector_t longest;
	// sanity checks
	if(unlikely(!a.initialized || !b.initialized)){
		fprintf(stderr,"ERROR in rc_poly_subtract, vector uninitialized\n");
		return -1;
	}
	// fill in out vector with longest input vector
	// if b is longer, replace reference a with contents of b
	if(a.len>b.len)	longest=a;
	else{
		longest=b;
		b=a;
	}
	if(unlikely(rc_vector_duplicate(longest,c))){
		fprintf(stderr,"ERROR in rc_poly_subtract, failed to duplicate vector\n");
		return -1;
	}
	// now the 'b' variable contains the shorter vector
	// itterate from the left
	diff = c->len-b.len;
	for(i=diff;i<c->len;i++) c->d[i]-=b.d[i-diff];
	return 0;
}


int rc_poly_subtract_inplace(rc_vector_t* a, rc_vector_t b)
{
	rc_vector_t tmp = RC_VECTOR_INITIALIZER;
	// sanity checks
	if(unlikely(!a->initialized || !b.initialized)){
		fprintf(stderr,"ERROR in rc_poly_subtract_in_place, vector uninitialized\n");
		return -1;
	}
	if(unlikely(rc_poly_subtract(*a,b,&tmp))){
		fprintf(stderr,"ERROR in rc_poly_subtract_in_place, subtract failed\n");
		return -1;
	}
	rc_vector_free(a);
	*a=tmp;
	return 0;
}


int rc_poly_differentiate(rc_vector_t a, int d, rc_vector_t* b)
{
	rc_vector_t tmp = RC_VECTOR_INITIALIZER;
	rc_vector_t tmp_r = RC_VECTOR_INITIALIZER;
	int i, new_order;
	// sanity checks
	if(unlikely(!a.initialized)){
		fprintf(stderr,"ERROR in rc_poly_differentiate, vector uninitialized\n");
		return -1;
	}
	if(unlikely(d<0)){
		fprintf(stderr,"ERROR in rc_poly_differentiate, d must be >=0\n");
		return -1;
	}
	// derivatives larger than the order of a polynomial are always zero
	if(d>=a.len) return rc_vector_zeros(b,1);
	// 0th derivative is the original polynomial
	if(d==0) return rc_vector_duplicate(a,b);
	// do one derivative
	new_order = a.len-1;
	if(unlikely(rc_vector_alloc(&tmp,new_order))){
		fprintf(stderr,"ERROR in rc_poly_differentiate, failed to alloc vector\n");
		return -1;
	}
	for(i=0; i<new_order; i++)	tmp.d[i]=a.d[i]*(new_order-i);
	// if 1st derivative was requested, all finished
	if(d==1){
		rc_vector_free(b);
		*b=tmp;
		return 0;
	}
	// for higher derivatives, call recursively
	if(unlikely(rc_poly_differentiate(tmp,d-1,&tmp_r))){
		fprintf(stderr,"ERROR in rc_poly_differentiate, failed to differentiate recursively\n");
		rc_vector_free(&tmp);
		return -1;
	}
	// free up memory except the answer tmp_r which goes into b
	rc_vector_free(&tmp);
	rc_vector_free(b);
	*b=tmp_r;
	return 0;
}


int rc_poly_divide(rc_vector_t n, rc_vector_t d, rc_vector_t* div, rc_vector_t* rem)
{
	int i, j, diff;
	rc_vector_t tmp = RC_VECTOR_INITIALIZER;
	// sanity checks
	if(unlikely(!n.initialized || !d.initialized)){
		fprintf(stderr,"ERROR in rc_poly_divide, vector uninitialized\n");
		return -1;
	}
	// difference in length, to be used later
	diff=n.len-d.len;
	// make sure den isn't bigger than num
	if(unlikely(diff<0)){
		fprintf(stderr,"ERROR in rc_poly_divide, order of num must be >= to den\n");
		return -1;
	}
	// allocate memory for divisor and copy numerator into remainder
	if(unlikely(rc_vector_zeros(div,diff+1))){
		fprintf(stderr,"ERROR in rc_poly_divide, failed to alloc vector\n");
		return -1;
	}
	if(unlikely(rc_vector_duplicate(n,&tmp))){
		fprintf(stderr,"ERROR in rc_poly_divide, failed to duplicate vector\n");
		rc_vector_free(div);
		return -1;
	}
	// calculate each entry in divisor, if num and den are same length
	// this will happen only once with i=0
	for(i=0;i<=diff;i++){
		div->d[i]=tmp.d[i]/d.d[0];
		// now subtract that multiple of denominator from remainder
		for(j=0;j<d.len;j++){
			tmp.d[j+i]-=div->d[i]*d.d[j];
		}
	}
	// fill in remainder from tmp
	if(unlikely(rc_vector_alloc(rem,d.len-1))){
		fprintf(stderr,"ERROR in rc_poly_divide, failed alloc rem vector\n");
		rc_vector_free(&tmp);
		return -1;
	}
	for(i=0;i<d.len-1;i++) rem->d[i]=tmp.d[i+diff+1];
	rc_vector_free(&tmp);
	return 0;
}


int rc_poly_butter(int N, double wc, rc_vector_t* b)
{
	int i;
	int ret=0;
	rc_vector_t P2	= RC_VECTOR_INITIALIZER;
	rc_vector_t P3	= RC_VECTOR_INITIALIZER;
	rc_vector_t tmp	= RC_VECTOR_INITIALIZER;
	// sanity checks
	if(unlikely(N<1)){
		fprintf(stderr,"ERROR in rc_poly_butter, order must be >1\n");
		return -1;
	}
	// Initialize polynomial as order 0 with coefficient 1.
	if(unlikely(rc_vector_ones(b,1))){
		fprintf(stderr,"ERROR in rc_poly_butter, failed to alloc vector\n");
		return -1;
	}
	// P2 will be of the form (s + 1) and is only used when desired order is odd.
	if(unlikely(rc_vector_alloc(&P2,2))){
		fprintf(stderr,"ERROR in rc_poly_butter, failed to alloc vector\n");
		rc_vector_free(b);
		return -1;
	}
	// P3 will be a 2nd order poly coresponding to imaginary poll pairs.
	if(unlikely(rc_vector_alloc(&P3,3))){
		fprintf(stderr,"ERROR in rc_poly_butter, failed to alloc vector\n");
		rc_vector_free(b);
		rc_vector_free(&P2);
		return -1;
	}
	// for even orders
	if(N%2 == 0){
		P3.d[0] = 1.0/(wc*wc);		// Initialize leading coefficient based on crossover
		P3.d[2] = 1.0;			// zeroth order coefficient is always 1
		for(i=1;i<=N/2;i++){
			// formula for first order poly coefficient based on desired order filter
			P3.d[1] = -2.0*cos(((2*i) + (N-1))*M_PI/(2.0*N))/wc;
			// duplicate b (which starts off as 1) to tmp
			rc_vector_duplicate(*b,&tmp);
			// perform convolution between tmp and P3. Send to b and loop through i to
			// desired order filter/2.  This opperation is equivalent to polynomial
			// multiplication of the form b = 1*(s^2 + s + 1)*(s^2 + s + 1)*(etc)
			if(unlikely(rc_poly_conv(tmp,P3,b))){
				fprintf(stderr,"ERROR in rc_poly_butter, failed to polyconv\n");
				ret = -1;
				goto POLY_END;
			}
		}
	}
	// for odd orders the opperation is similar to above except for a real poll at -1.
	// This is why P2, as in (s + 1), is convolved first, then subsequent P3s.
	if(N%2 == 1){
		P2.d[0] = 1.0/wc;
		P2.d[1] = 1.0;
		rc_vector_duplicate(*b,&tmp);
		if(unlikely(rc_poly_conv(tmp,P2,b))){
			fprintf(stderr,"ERROR in rc_poly_butter, failed to polyconv\n");
			ret = -1;
			goto POLY_END;
		}
		P3.d[0] = 1.0/(wc*wc);
		P3.d[2] = 1.0;
		for(i=1;i<=(N-1)/2;i++){
			P3.d[1] = -2.0*cos(((2*i) + (N-1))*M_PI/(2.0*N))/wc;
			rc_vector_duplicate(*b,&tmp);
			if(unlikely(rc_poly_conv(tmp,P3,b))){
				fprintf(stderr,"ERROR in rc_poly_butter, failed to polyconv\n");
				ret = -1;
				goto POLY_END;
			}
		}
	}
POLY_END:
	// free up memory
	rc_vector_free(&tmp);
	rc_vector_free(&P2);
	rc_vector_free(&P3);
	return ret;
}


