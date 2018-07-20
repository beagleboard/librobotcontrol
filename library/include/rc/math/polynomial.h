/**
 * <rc/math/polynomial.h>
 *
 * @brief      Functions for polynomial manipulation.
 *
 * We represent polynomials as a vector of coefficients with the highest power
 * term on the left at vector index 0. The following polynomial manipulation
 * functions are designed to behave like their counterparts in the Numerical
 * Renaissance codebase.
 *
 * @author     James Strawson
 * @date       2016
 *
 * @addtogroup Polynomial
 * @ingroup    Math
 * @{
 */


#ifndef RC_POLYNOMIAL_H
#define RC_POLYNOMIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <rc/math/vector.h>

/**
 * @brief      Prints a polynomial in human-readable format in one line.
 *
 * Like rc_print_vector, but assumes the contents represent a polynomial and
 * prints the coefficients with trailing powers of x for easier reading. This
 * relies on your terminal supporting unicode UTF-8. numer of coefficients and
 * there the length of vector v must be less than or equal to 10.
 *
 * @param[in]  v     polynomial coefficients to be printed
 *
 * @return     0 on success or -1 on failure
 */
int rc_poly_print(rc_vector_t v);

/**
 * @brief      Convolutes the polynomials a&b and places the result in vector c.
 *
 * This finds the coefficients of the polynomials resulting from multiply a*b.
 * The original contents of c are freed and new memory is allocated if
 * necessary.
 *
 * @param[in]  a     First set of coefficients
 * @param[in]  b     Second set of coefficients
 * @param[out] c     Vector to output resulting coefficients
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_poly_conv(rc_vector_t a, rc_vector_t b, rc_vector_t* c);

/**
 * @brief      Raises a polynomial a to itself n times where n is greater than
 * or equal to 0.
 *
 * Places the result in vector b, any existing memory allocated for b is freed
 * and its contents are lost. Returns 0 on success and -1 on failure.
 *
 * @param[in]  a     Initial coefficients
 * @param[in]  n     Power, must be >=0
 * @param[out] b     resulting coefficients
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_poly_power(rc_vector_t a, int n, rc_vector_t* b);

/**
 * @brief      Add two polynomials a&b with right justification and place the
 * result in c.
 *
 * Any existing memory allocated for c is freed and its contents are lost.
 *
 * @param[in]  a     First input
 * @param[in]  b     second input
 * @param[out] c     output
 *
 * @return     Returns 0 on success and -1 on failure.
 */
int rc_poly_add(rc_vector_t a, rc_vector_t b, rc_vector_t* c);

/**
 * @brief      Adds polynomials a&b with right justification.
 *
 * The result is placed in vector a and a's original contents are lost. More
 * memory is allocated for a if necessary.
 *
 * @param      a     First input and where output is written
 * @param[in]  b     second input
 *
 * @return     Returns 0 on success and -1 on failure.
 */
int rc_poly_add_inplace(rc_vector_t* a, rc_vector_t b);

/**
 * @brief      Subtracts two polynomials a-b with right justification and places
 * the result in c.
 *
 * Any existing memory allocated for c is freed and its contents are lost.
 * Returns 0 on success and -1 on failure.
 *
 * @param[in]  a     First input
 * @param[in]  b     second input
 * @param[out] c     output
 *
 * @return     Returns 0 on success and -1 on failure.
 */
int rc_poly_subtract(rc_vector_t a, rc_vector_t b, rc_vector_t* c);

/**
 * @brief      Subtracts b from a with right justification.
 *
 * a stays in place and new memory is allocated only if b is longer than a.
 *
 * @param      a     First input and where output is written
 * @param[in]  b     second input
 *
 * @return     Returns 0 on success and -1 on failure.
 */
int rc_poly_subtract_inplace(rc_vector_t* a, rc_vector_t b);

/**
 * @brief      Calculates the dth derivative of the polynomial a and places the
 * result in vector b.
 *
 * @param[in]  a     Input polynomial coefficients
 * @param[in]  d     which derivative to take (>=0)
 * @param[out] b     result
 *
 * @return     Returns 0 on success and -1 on failure.
 */
int rc_poly_differentiate(rc_vector_t a, int d, rc_vector_t* b);

/**
 * @brief      Divides denominator d into numerator n. The remainder is placed
 * into vector rem and the divisor is placed into vector div.
 *
 * @param[in]  n     numerator
 * @param[in]  d     denominator
 * @param      div   The resulting divisor
 * @param      rem   The resulting remainder
 *
 * @return     Returns 0 on success and -1 on failure.
 */
int rc_poly_divide(rc_vector_t n, rc_vector_t d, rc_vector_t* div, rc_vector_t* rem);

/**
 * @brief      Calculates coefficients for continuous-time Butterworth
 * polynomial of order N and cutoff wc (rad/s) and places them in vector b.
 *
 * @param[in]  N     Order of the polynomial
 * @param[in]  wc    cutoff frequency in rad/s
 * @param[out] b     resulting coefficients
 *
 * @return     Returns 0 on success and -1 on failure.
 */
int rc_poly_butter(int N, double wc, rc_vector_t* b);



#ifdef __cplusplus
}
#endif

#endif // RC_POLYNOMIAL_H

/** @} end group math*/