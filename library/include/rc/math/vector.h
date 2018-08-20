/**
 * <rc/math/vector.h>
 *
 * @brief      Functions for vector manipulation.
 *
 * The small rc_vector_t struct contains information about the vector's size and
 * a pointer to where dynamically allocated memory exists that stores the actual
 * data for the vector. Use rc_vector_alloc to dynamically allocate memory for
 * each new vector. Then use rc_vector_free() and to free the memory when you
 * are done using it. See the remaining vector, matrix, and linear algebra
 * functions for more details.
 *
 * @author     James Strawson
 * @date       2016
 *
 * @addtogroup Vector
 * @ingroup    Math
 * @{
 */


#ifndef RC_VECTOR_H
#define RC_VECTOR_H

#ifdef  __cplusplus
extern "C" {
#endif

extern double zero_tolerance;

/**
 * @brief      Struct containing the state of a vector and a pointer to
 * dynamically allocated memory to hold its contents.
 *
 * Set and read values directly with this code:
 * @code{.c}
 * vec.d[position]=new_value; // set value in the vector
 * value = v.d[pos]; // get value from vector
 * @endcode
 */
typedef struct rc_vector_t{
	int len;	///< number of elements in the vector
	double* d;	///< pointer to dynamically allocated data
	int initialized;///< initialization flag
} rc_vector_t;


#define RC_VECTOR_INITIALIZER {\
	.len = 0,\
	.d = NULL,\
	.initialized = 0}


/**
 * @brief      Returns an rc_vector_t with no allocated memory and the
 * initialized flag set to 0.
 *
 * This is essential for initializing vectors when they are declared since
 * local variables declared in a function without global variable scope in C are
 * not guaranteed to be zeroed out which can lead to bad memory pointers and
 * segfaults if not handled carefully. We recommend initializing all vectors
 * with this function before using rc_vector_alloc or any other function.
 *
 * @return     rc_vector_t with no allocated memory and the initialized flag set
 * to 0.
 */
rc_vector_t rc_vector_empty(void);

/**
 * @brief      Allocates memory for vector v to have specified length.
 *
 * If v is initially the right length then nothing is done and the data in v is
 * preserved. If v is uninitialized or of the wrong length then any existing
 * memory is freed and new memory is allocated, helping to prevent accidental
 * memory leaks.
 *
 * The contents of the new vector is not guaranteed to be anything in particular
 * as it is allocated with malloc. Use rc_vector_zeros or rc_vector_ones if you
 * require known starting values.
 *
 * Returns 0 if successful, otherwise returns -1. Will only be unsuccessful if
 * length is invalid or there is insufficient memory available.
 *
 * @param      v       Pointer to user's rc_vector_t struct
 * @param[in]  length  Length of vector to allocate memory for
 *
 * @return     Returns 0 if successful, otherwise returns -1.
 */
int rc_vector_alloc(rc_vector_t* v, int length);

/**
 * @brief      Frees the memory allocated for vector v.
 *
 * Also sets the length and initialized flag of the rc_vector_t struct to 0 to
 * indicate to other functions that v no longer points to allocated memory and
 * cannot be used until more memory is allocated such as with rc_vector_alloc or
 * rc_vector_zeros. Returns 0 on success. Will only fail and return -1 if it is
 * passed a NULL pointer.
 *
 * @param      v     Pointer to user's rc_vector_t struct
 *
 * @return     Returns 0 if successful, otherwise returns -1.
 */
int rc_vector_free(rc_vector_t* v);

/**
 * @brief      Resizes vector v and fills with zeros.
 *
 * uses calloc to allocate new memory. Any existing memory allocated for v is
 * freed if necessary to avoid memory leaks. It is not necessary to call
 * rc_alloc_vector before this.
 *
 * @param      v       Pointer to user's rc_vector_t struct
 * @param[in]  length  Length of vector to allocate memory for
 *
 * @return     Returns 0 if successful, otherwise returns -1.
 */
int rc_vector_zeros(rc_vector_t* v, int length);

/**
 * @brief      Resizes vector v and fills with ones.
 *
 * Any existing memory allocated for v is freed if necessary to avoid memory
 * leaks. It is not necessary to call rc_alloc_vector before this.
 *
 * @param      v       Pointer to user's rc_vector_t struct
 * @param[in]  length  Length of vector to allocate memory for
 *
 * @return     Returns 0 if successful, otherwise returns -1.
 */
int rc_vector_ones(rc_vector_t* v, int length);

/**
 * @brief      Resizes vector v and fills with random numbers between -1.0 and
 * 1.0
 *
 * Any existing memory allocated for v is freed if necessary to avoid memory
 * leaks. It is not necessary to call rc_alloc_vector before this.
 *
 * @param      v       Pointer to user's rc_vector_t struct
 * @param[in]  length  Length of vector to allocate memory for
 *
 * @return     Returns 0 if successful, otherwise returns -1.
 */
int rc_vector_random(rc_vector_t* v, int length);

/**
 * @brief      Resizes vector v and fills with Fibonnaci sequence
 *
 * Any existing memory allocated for v is freed if necessary to avoid memory
 * leaks. It is not necessary to call rc_alloc_vector before this.
 *
 * @param      v       Pointer to user's rc_vector_t struct
 * @param[in]  length  Length of vector to allocate memory for
 *
 * @return     Returns 0 if successful, otherwise returns -1.
 */
int rc_vector_fibonnaci(rc_vector_t* v, int length);

/**
 * @brief      Resizes vector v and populates with values from specified array
 * ptr.
 *
 * Any existing memory allocated for v is freed if necessary to avoid memory
 * leaks. It is not necessary to call rc_alloc_vector before this. This is
 * generally used when the user has an existing array of data and wants to use
 * it with other math functions.
 *
 * @param      v       Pointer to user's rc_vector_t struct
 * @param[in]  ptr     pointer to array to read values from
 * @param[in]  length  Length of vector to allocate memory for
 *
 * @return     Returns 0 if successful, otherwise returns -1.
 */
int rc_vector_from_array(rc_vector_t* v, double* ptr, int length);

/**
 * @brief      Duplicates the contents of vector a and into a new vector b.
 *
 * Simply making a copy of an rc_vector_t struct is not sufficient as the
 * rc_vector_t struct simply contains a pointer to the memory allocated to
 * contain the contents of the vector. rc_vector_duplicate sets b to be a new
 * rc_vector_t with a pointer to freshly-allocated memory.
 *
 * @param[in]  a     Vector to be duplicated
 * @param      b     pointer to new vector to be allocated and written
 *
 * @return     Returns 0 if successful, otherwise returns -1.
 */
int rc_vector_duplicate(rc_vector_t a, rc_vector_t* b);

/**
 * @brief      Prints to stdout the contents of vector v in one line.
 *
 * This is not advisable for extremely long vectors but serves for quickly
 * debugging or printing results. It prints 4 decimal places with padding for a
 * sign. We recommend rc_vector_print_sci() for very small or very large numbers
 * where scientific notation would be more appropriate.
 *
 * @param[in]  v     Pointer to user's rc_vector_t struct
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_vector_print(rc_vector_t v);

/**
 * @brief      Prints to stdout the contents of vector v in one line.
 *
 * Like rc_vector_print but prints with scientific notation. This is not
 * advisable for extremely long vectors but serves for quickly debugging or
 * printing.
 *
 * @param[in]  v     Pointer to user's rc_vector_t struct
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_vector_print_sci(rc_vector_t v);

/**
 * @brief      Sets all values of an already-allocated vector to 0
 *
 * @param      v     pointer to vector to be zero'd out
 *
 * @return     0 on success, -1 on failure.
 */
int rc_vector_zero_out(rc_vector_t* v);

/**
 * @brief      Multiplies every entry in vector v by scalar s.
 *
 * It is not strictly necessary for v to be provided as a pointer since a copy
 * of the struct v would also contain the correct pointer to the original
 * vector's allocated memory. However, in this library we use the convention of
 * passing an rc_vector_t struct or rc_matrix_struct as a pointer when its data
 * is to be modified by the function, and as a normal argument when it is only
 * to be read by the function.
 *
 * @param      v     Pointer to user's rc_vector_t struct
 * @param[in]  s     scalar multiplier
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_vector_times_scalar(rc_vector_t* v, double s);

/**
 * @brief      Returns the vector norm defined by sum(abs(v)^p)^(1/p), where p
 * is any positive real value.
 *
 * Just like the matlab norm(v,p) function.
 *
 * Most common norms are the 1 norm which gives the sum of absolute values of
 * the vector and the 2-norm which is the square root of sum of squares. for
 * infinity and -infinity norms see rc_vector_max and rc_vector_min
 *
 * @param[in]  v     User's vector struct
 * @param[in]  p     Which norm to use. Positive real values only.
 *
 * @return     vector norm. Prints error message and returns -1.0f on error.
 */
double rc_vector_norm(rc_vector_t v, double p);

/**
 * @brief      Returns the index of the maximum value in v.
 *
 * The value contained in the returned index is the equivalent to the infinity
 * norm. If the max value occurs multiple times then the index of the first
 * instance is returned.
 *
 * @param[in]  v     User's vector struct
 *
 * @return     Returns the index of the maximum value in v or -1 on error.
 */
int rc_vector_max(rc_vector_t v);

/**
 * @brief      Returns the index of the minimum value in v.
 *
 * The value contained in the returned index is the equivalent to the -infinity
 * norm. If the minimum value occurs multiple times then the index of the first
 * instance is returned.
 *
 * @param[in]  v     User's vector struct
 *
 * @return     Returns the index of the minimum value in v or -1 on error.
 */
int rc_vector_min(rc_vector_t v);

/**
 * @brief      Returns the standard deviation of the values in a vector.
 *
 * @param[in]  v     User's vector struct
 *
 * @return     Returns the standard deviation or prints and error message and
 * return -1.0f on error.
 */
double rc_vector_std_dev(rc_vector_t v);

/**
 * @brief      Returns the mean (average) of all values in vector v or -1.0f on
 * error.
 *
 * @param[in]  v     User's vector struct
 *
 * @return     Returns the mean (average) of all values in vector v or -1.0f on
 * error.
 */
double rc_vector_mean(rc_vector_t v);

/**
 * @brief      Populates vector p with the projection of vector v onto e.
 *
 * p is resized appropriately and any exising memory is freed to avoid memory
 * leaks.
 *
 * @param[in]  v     User's vector struct
 * @param[in]  e     User's vector struct
 * @param[out] p     output
 *
 * @return     Returns 0 on success, otherwise -1.
 */
int rc_vector_projection(rc_vector_t v, rc_vector_t e, rc_vector_t* p);

/**
 * @brief      Calculates the dot product of two equal-length vectors.
 *
 * @param[in]  v1    User's vector struct
 * @param[in]  v2    User's vector struct
 *
 * @return     Returns the dot product, or prints and error message and returns
 * -1.0f on error.
 */
double rc_vector_dot_product(rc_vector_t v1, rc_vector_t v2);

/**
 * @brief      Computes the cross-product of two vectors, each of length 3.
 *
 * The result is placed in vector p and and any existing memory used by p is
 * freed to avoid memory leaks.
 *
 * @param[in]  v1    User's vector struct
 * @param[in]  v2    User's vector struct
 * @param[out] p     resulting cross product
 *
 * @return     Returns 0 on success, otherwise -1.
 */
int rc_vector_cross_product(rc_vector_t v1, rc_vector_t v2, rc_vector_t* p);

/**
 * @brief      Populates vector s with the sum of vectors v1 and v2.
 *
 * v1 and v2 must be of the same length. Any existing memory allocated for s is
 * freed and lost, new memory is allocated if necessary.
 *
 * @param[in]  v1    User's vector struct
 * @param[in]  v2    User's vector struct
 * @param[out] s     output sum
 *
 * @return     Returns 0 on success, otherwise -1.
 */
int rc_vector_sum(rc_vector_t v1, rc_vector_t v2, rc_vector_t* s);

/**
 * @brief      Adds vector v2 to v1 and leaves the result in v1.
 *
 * The original contents of v1 are lost and v2 is left untouched. v1 and v2 must
 * be of the same length.
 *
 * @param      v1    User's vector struct, holds the result
 * @param[in]  v2    User's vector struct
 *
 * @return     Returns 0 on success, otherwise -1.
 */
int rc_vector_sum_inplace(rc_vector_t* v1, rc_vector_t v2);

/**
 * @brief      Populates vector s with the difference v1 - v2.
 *
 * v1 and v2 must be of the same length. Any existing memory allocated for s is
 * freed and lost, new memory is allocated if necessary.
 *
 * @param[in]  v1    User's vector struct
 * @param[in]  v2    User's vector struct
 * @param[out] s     output difference v1-v2
 *
 * @return     Returns 0 on success, otherwise -1.
 */
int rc_vector_subtract(rc_vector_t v1, rc_vector_t v2, rc_vector_t* s);


#ifdef __cplusplus
}
#endif

#endif // RC_VECTOR_H

/** @} end group math*/
