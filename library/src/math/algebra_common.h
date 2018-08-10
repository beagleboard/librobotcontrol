/**
 * @ file rc_algebra_common.h
 *
 * all things shared between rc_vector.c, rc_matrix.c, and rc_linear_algebra.c
 */

#ifndef RC_ALGEBRA_COMMON_H
#define RC_ALGEBRA_COMMON_H

#ifndef unlikely
#define unlikely(x)	__builtin_expect (!!(x), 0)
#endif

#ifndef likely
#define likely(x)	__builtin_expect (!!(x), 1)
#endif

/*
 * Performs a vector dot product on the contents of a and b over n values.
 *
 * This is a dangerous function that could segfault if not used properly. Hence
 * it is only for internal use in the RC library. the 'restrict' attributes tell
 * the C compiler that the pointers are not aliased which helps the vectorization
 * process for optimization with the NEON FPU or similar
 */
double __vectorized_mult_accumulate(double * __restrict__ a, double * __restrict__ b, int n);

/*
 * Performs a vector dot product on the contents of a with itself
 *
 * This is a dangerous function that could segfault if not used properly. Hence
 * it is only for internal use in the RC library. the 'restrict' attributes tell
 * the C compiler that the pointers are not aliased which helps the vectorization
 * process for optimization with the NEON FPU or similar
 */
double __vectorized_square_accumulate(double * __restrict__ a, int n);

#endif // RC_ALGEBRA_COMMON_H
