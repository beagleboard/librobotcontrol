/**
 * <rc/math/quaternion.h>
 *
 * @brief      Functions for quaternion manipulation.
 *
 * All functions have two versions, one that acts on an rc_vector_t of length 4
 * and another that acts on normal C arrays, also of length 4. They are
 * functionally identical and just give the user options depending on how they
 * represent quaternions in their code.
 *
 * @author     James Strawson
 * @date       2016
 *
 * @addtogroup Quaternion
 * @ingroup    Math
 * @{
 */


#ifndef RC_QUATERNION_H
#define RC_QUATERNION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <rc/math/vector.h>
#include <rc/math/matrix.h>

/**
 * @brief      Returns the length of a quaternion vector by finding its 2-norm.
 *
 * @param[in]  q     The quarternion in form of a vector of length 4
 *
 * @return     Returns the norm, or prints an error message and returns -1.0f on
 * error.
 */
double rc_quaternion_norm(rc_vector_t q);

/**
 * @brief      Returns the length of a quaternion vector by finding its 2-norm.
 *
 * @param[in]  q     The quarternion in form of an array of length 4
 *
 * @return     Returns the norm, or prints an error message and returns -1.0f on
 * error.
 */
double rc_quaternion_norm_array(double q[4]);

/**
 * @brief      Normalizes a quaternion in-place to have length 1.0
 *
 * @param      q     The quarternion in form of a vector of lenth 4
 *
 * @return     Returns 0 on success or -1 on failure
 */
int   rc_normalize_quaternion(rc_vector_t* q);

/**
 * @brief      Normalizes a quaternion in-place to have length 1.0
 *
 * @param      q     The quarternion in form of an array of length 4
 *
 * @return     Returns 0 on success or -1 on failure
 */
int   rc_normalize_quaternion_array(double q[4]);

/**
 * @brief      Calculates 321 Tait Bryan angles in array order XYZ with
 * operation order 321(yaw-Z, pitch-Y, roll-x).
 *
 * If tb is already allocated and of length 3 then the new values are written in
 * place, otherwise any existing memory is freed and a new vector of length 3 is
 * allocated for tb.
 *
 * @param[in]  q     The quarternion in form of a vector of lenth 4
 * @param      tb    Output tait-bryan angles
 *
 * @return     Returns 0 on success or -1 on failure
 */
int   rc_quaternion_to_tb(rc_vector_t q, rc_vector_t* tb);

/**
 * @brief      Calculates 321 Tait Bryan angles in array order XYZ with
 * operation order 321(yaw-Z, pitch-Y, roll-x).
 *
 * @param[in]  q     The quarternion in form of an array of lenth 4
 * @param[out] tb    Output tait-bryan angles
 *
 * @return     Returns 0 on success or -1 on failure
 */
int  rc_quaternion_to_tb_array(double q[4], double tb[3]);

/**
 * @brief      Calculates quaternion vector q from tait-bryan angles tb.
 *
 * If q is already of length 4 then old contents are simply overwritten.
 * Otherwise q'd existing memory is freed and new memory is allocated to aint
 * memory leaks. tb angles are 321 Tait Bryan angles in array order XYZ with
 * operation order 321(yaw-Z, pitch-Y, roll-x).
 *
 * @param[in]  tb    input tait-bryan angles
 * @param[out] q     output quaternion
 *
 * @return     Returns 0 on success or -1 on failure
 */
int   rc_quaternion_from_tb(rc_vector_t tb, rc_vector_t* q);

/**
 * @brief      Calculates quaternion vector q from tait-bryan angles tb.
 *
 * tb angles are 321 Tait Bryan angles in array order XYZ with operation order
 * 321(yaw-Z, pitch-Y, roll-x).
 *
 * @param[in]  tb    input tait-bryan angles
 * @param[out] q     output quaternion
 *
 * @return     Returns 0 on success or -1 on failure
 */
int  rc_quaternion_from_tb_array(double tb[3], double q[4]);

/**
 * @brief      Calculates conjugate of quaternion q.
 *
 * Populates quaternion vector c with the conjugate of quaternion q where the 3
 * imaginary parts ijk are multiplied by -1. If c is already of length 4 then
 * the old values are overwritten. Otherwise the old memory in c is freed and
 * new memory is allocated to help prevent memory leaks.
 *
 * @param[in]  q     The quarter
 * @param[out] c     output conjugate
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int   rc_quaternion_conjugate(rc_vector_t q, rc_vector_t* c);

/**
 * @brief      Calculates conjugate of quaternion q and overwrites the old q.
 *
 * Populates quaternion vector q with its conjugate where the 3 imaginary parts
 * ijk are multiplied by -1.
 *
 * @param[in]  q     The quarternion
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int   rc_quaternion_conjugate_inplace(rc_vector_t* q);

/**
 * @brief      Calculates conjugate of quaternion q.
 *
 * Populates quaternion vector c with the conjugate of quaternion q where the 3
 * imaginary parts ijk are multiplied by -1.
 *
 * @param[in]  q     The quarter
 * @param[out] c     output conjugate
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int  rc_quaternion_conjugate_array(double q[4], double c[4]);

/**
 * @brief      Calculates conjugate of quaternion q and overwrites the old q.
 *
 * Populates quaternion vector q with its conjugate where the 3 imaginary parts
 * ijk are multiplied by -1.
 *
 * @param[in]  q     The quarternion
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int  rc_quaternion_conjugate_array_inplace(double q[4]);

/**
 * @brief      Populates vector i with the imaginary components ijk of of
 * quaternion vector q.
 *
 * If img is already of length 3 then its original contents are overwritten.
 * Otherwise the original allocated memory is freed and new memory is allocated.
 *
 * @param[in]  q     The quarternion
 * @param[out] img   imaginary part
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int   rc_quaternion_imaginary_part(rc_vector_t q, rc_vector_t* img);

/**
 * @brief      Calculates the quaternion Hamilton product ab=c
 *
 * Places the result in vector argument c. If c is already of length 4 then the
 * old values are overwritten. Otherwise the old memory in c is freed and new
 * memory is allocated to help prevent memory leaks.
 *
 * @param[in]  a     First input
 * @param[in]  b     second input
 * @param[out] c     output
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int   rc_quaternion_multiply(rc_vector_t a, rc_vector_t b, rc_vector_t* c);

/**
 * @brief      Calculates the quaternion Hamilton product ab=c
 *
 * Places the result in vector argument c.
 *
 * @param[in]  a     First input
 * @param[in]  b     second input
 * @param[out] c     output
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int  rc_quaternion_multiply_array(double a[4], double b[4], double c[4]);

/**
 * @brief      Rotates the quaternion p by quaternion q with the operation
 * p'=qpq*
 *
 * p is modified in place, q is unmodified.
 *
 * @param      p     quaternion to be rotated
 * @param[in]  q     rotation quarternion
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int   rc_quaternion_rotate(rc_vector_t* p, rc_vector_t q);

/**
 * @brief      Rotates the quaternion p by quaternion q with the operation
 * p'=qpq*
 *
 * p is modified in place, q is unmodified.
 *
 * @param      p     quaternion to be rotated
 * @param[in]  q     rotation quarternion
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int  rc_quaternion_rotate_array(double p[4], double q[4]);

/**
 * @brief      Rotate a 3D vector v in-place about the origin by quaternion q by
 * converting v to a quaternion and performing the operation p'=qpq*
 *
 * v is modified in place, q is unmodified.
 *
 * @param      v     vectpr to be rotated
 * @param[in]  q     rotation quarternion
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int   rc_quaternion_rotate_vector(rc_vector_t* v, rc_vector_t q);

/**
 * @brief      Rotate a 3D vector v in-place about the origin by quaternion q by
 * converting v to a quaternion and performing the operation p'=qpq*
 *
 * v is modified in place, q is unmodified.
 *
 * @param      v     vectpr to be rotated
 * @param[in]  q     rotation quarternion
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int  rc_quaternion_rotate_vector_array(double v[3], double q[4]);

/**
 * @brief      Converts a normalized quaternion to a 3x3 orthogonal rotation
 * matrix.
 *
 * q must be normalized before calling this! The orthogonal matrix corresponds
 * to a rotation by the unit quaternion q when post-multiplied with a column
 * vector as such: v_rotated=mv.
 *
 * If m is already 3x3 then its contents are overwritten, otherwise its existing
 * memory is freed and new memory is allocated.
 *
 * @param[in]  q     The quarter
 * @param      m     output 3x3 rotation matrix
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int   rc_quaternion_to_rotation_matrix(rc_vector_t q, rc_matrix_t* m);



#ifdef __cplusplus
}
#endif

#endif // RC_QUATERNION_H

/** @} end group math*/