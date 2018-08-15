/**
 * @file math/other.c
 *
 * @brief      general low-level math functions that don't fit elsewhere
 *
 * @author     James Strawson
 * @date       2016
 *
 */

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <rc/math/other.h>

// preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)

typedef union {
	uint32_t i;
	float f;
}rc_int_float_t;

float rc_get_random_float(void)
{
	rc_int_float_t new;
	// get random 32-bit int, mask out all but the mantissa (right 23 bits)
	// with the & operator, then set the leftmost exponent bit to scale it
	new.i = (rand()&0x007fffff) | 0x40000000;
	return new.f - 3.0f; // convert to float and shift to range from -1 to 1
}


typedef union {
	uint64_t i;
	double d;
}rc_int_double_t;

double rc_get_random_double(void)
{
	rc_int_double_t new;
	// get random 64-bit int, mask out all but the mantissa (right 52 bits)
	// with the & operator, then set the leftmost exponent bit to scale it
	new.i = ((((uint64_t)rand()<<32)|rand()) & 0x000fffffffffffff) | 0x4000000000000000;
	return new.d - 3.0; // convert to double and shift to range from -1 to 1
}

int rc_saturate_float(float* val, float min, float max)
{
	// sanity checks
	if(unlikely(min>max)){
		fprintf(stderr,"ERROR: in rc_saturate_float, min must be less than max\n");
		return -1;
	}
	// bound val
	if(*val>max){
		*val = max;
		return 1;
	}else if(*val<min){
		*val = min;
		return 1;
	}
	return 0;
}

int rc_saturate_double(double* val, double min, double max)
{
	// sanity checks
	if(unlikely(min>max)){
		fprintf(stderr,"ERROR: in rc_saturate_double, min must be less than max\n");
		return -1;
	}
	// bound val
	if(*val>max){
		*val = max;
		return 1;
	}else if(*val<min){
		*val = min;
		return 1;
	}
	return 0;
}