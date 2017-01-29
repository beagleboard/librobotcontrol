/*******************************************************************************
* rc_algebra_common.h
*
* all things shared between rc_vector.c, rc_matrix.c, and rc_linear_algebra.c
*******************************************************************************/

#include "../roboticscape.h"
#include "../preprocessor_macros.h"
#include <stdio.h>	// for fprintf
#include <stdlib.h>	// for malloc,calloc,free
#include <math.h>	// for sqrt, pow, etc
#include <float.h>	// for FLT_MAX
#include <string.h>	// for memcpy

#define ZERO_TOLERANCE 1e-6 // consider v to be zero if fabs(v)<ZERO_TOLERANCE

/*******************************************************************************
* float rc_mult_accumulate(float * __restrict__ a, float * __restrict__ b, int n)
* 
* Performs a vector dot product on the contents of a and b over n values.
* This is a dangerous function that could segfault if not used properly. Hence
* it is only for internal use in the RC library. the 'restrict' attributes tell
* the C compiler that the pointers are not aliased which helps the vectorization
* process for optimization with the NEON FPU.
*******************************************************************************/
float rc_mult_accumulate(float * __restrict__ a, float * __restrict__ b, int n);

