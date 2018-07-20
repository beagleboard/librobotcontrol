/**
 * <rc/math/algebra.h>
 *
 * @brief      advanced linear algebra functions
 *
 * @addtogroup Algebra
 * @ingroup Math
 * @{
 */

#ifndef RC_ALGEBRA_H
#define RC_ALGEBRA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <rc/math/matrix.h>

/**
 * @brief      Performs LUP decomposition on matrix A with partial pivoting.
 *
 * Places the result in matrices L,U,&P. Matrix A remains untouched and the
 * original contents of LUP (if any) are freed and LUP are resized
 * appropriately.
 *
 * @param[in]  A     input matrix
 * @param[out] L     lower triangular
 * @param[out] U     upper triangular
 * @param[out] P     permutation matrix
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_algebra_lup_decomp(rc_matrix_t A, rc_matrix_t* L, rc_matrix_t* U, rc_matrix_t* P);

/**
 * @brief      Calculate the QR decomposition of matrix A.
 *
 * Uses householder reflection method. Matrix A remains untouched and the
 * original contents of Q&R (if any) are freed and resized appropriately.
 *
 * @param[in]  A     input matrix
 * @param[out] Q     orthogonal matrix output
 * @param[out] R     upper triangular matrix output
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_algebra_qr_decomp(rc_matrix_t A, rc_matrix_t* Q, rc_matrix_t* R);

/**
 * @brief      Inverts matrix A via LUP decomposition method.
 *
 * Places the result in matrix Ainv. Any existing memory allocated for Ainv is
 * freed if necessary and its contents are overwritten. Returns -1 if matrix is
 * not invertible.
 *
 * @param[in]  A     input matrix
 * @param[out] Ainv  resulting inverted matrix
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_algebra_invert_matrix(rc_matrix_t A, rc_matrix_t* Ainv);

/**
 * @brief      Inverts matrix A in place.
 *
 * The original contents of A are lost. Returns -1 if A is not invertible.
 *
 * @param      A     matrix to be inverted
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_algebra_invert_matrix_inplace(rc_matrix_t* A);

/**
 * @brief      Solves Ax=b for given matrix A and vector b.
 *
 * Places the result in vector x. existing contents of x are freed and new
 * memory is allocated if necessary.
 *
 * @param[in]  A     matrix A
 * @param[in]  b     column vector b
 * @param[out] x     solution column vector
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_algebra_lin_system_solve(rc_matrix_t A, rc_vector_t b, rc_vector_t* x);

/**
 * @brief      Sets the zero tolerance for detecting singular matrices.
 *
 * When inverting matrices or solving a linear system, this library first checks
 * that the determinant of the matrix is non-zero. Due to the rounding errors
 * that come from float-point math, we cannot check if the determinant is
 * exactly zero. Instead, it is checked to be smaller in magnitude than the
 * zero-tolerance.
 *
 * The default value is 10^-8 but it can be changed here if the user is dealing
 * with unusually small or large floating point values.
 *
 * This only effects the operation of rc_algebra_invert_matrix,
 * rc_algebra_invert_matrix_inplace, and rc_algebra_lin_system_solve.
 *
 * @param[in]  tol   The zero-tolerance
 */
void rc_algebra_set_zero_tolerance(double tol);

/**
 * @brief      Finds a least-squares solution to the system Ax=b for non-square
 * A using QR decomposition method.
 *
 * Places the solution in x.
 *
 * @param[in]  A     matrix A
 * @param[in]  b     column vector b
 * @param[out] x     solution column vector
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_algebra_lin_system_solve_qr(rc_matrix_t A, rc_vector_t b, rc_vector_t* x);

/**
 * @brief      Fits an ellipsoid to a set of points in 3D space.
 *
 * The principle axes of the fitted ellipsoid align with the global coordinate
 * system. Therefore there are 6 degrees of freedom defining the ellipsoid: the
 * x,y,z coordinates of the centroid and the lengths from the centroid to the
 * surface in each of the 3 directions.
 *
 * rc_matrix_t 'points' is a tall matrix with 3 columns and at least 6 rows.
 * Each row must contain the x,y&z components of each individual point to be
 * fit. If only 6 rows are provided, the resulting ellipsoid will be an exact
 * fit. Otherwise the result is a least-squares fit to the over-defined dataset.
 *
 * The final x,y,z position of the centroid will be placed in vector 'center'
 * and the lengths or radius from the centroid to the surface along each axis
 * will be placed in the vector 'lengths'
 *
 * @param[in]  points   datapoints to fit
 * @param[out] center   center of ellipse
 * @param[out] lengths  lengths along principle axis
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_algebra_fit_ellipsoid(rc_matrix_t points, rc_vector_t* center, rc_vector_t* lengths);


#ifdef  __cplusplus
}
#endif

#endif // RC_ALGEBRA_H

/** @}  end group math*/