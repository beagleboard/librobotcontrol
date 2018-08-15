/**
 * @file algebra_common.c
 *
 * see algebra_common.h
 **/

#include "algebra_common.h"

double __vectorized_mult_accumulate(double * __restrict__ a, double * __restrict__ b, int n)
{
	int i;
	double sum = 0.0;
	for(i=0;i<n;i++){
		sum+=a[i]*b[i];
	}
	return sum;
}


double __vectorized_square_accumulate(double * __restrict__ a, int n)
{
	int i;
	double sum = 0.0;
	for(i=0;i<n;i++){
		sum+=a[i]*a[i];
	}
	return sum;
}