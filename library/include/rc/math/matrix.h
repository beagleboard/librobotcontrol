/**
 * <rc/math/matrix.h>
 *
 * @brief      functions for masic matrix manipulation
 *
 *
 * @addtogroup Matrix
 * @ingroup Math
 * @{
 */


#ifndef RC_MATRIX_H
#define RC_MATRIX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <rc/math/vector.h>

/**
 * @brief      Struct containing the state of a matrix and a pointer to
 * dynamically allocated memory to hold its contents.
 *
 * Set and read values directly with this code:
 * @code{.c}
 * matrix.d[row][col] = new_value; // set value in the matrix
 * value = matrix.d[row][col];     // get value from the matrix
 * @endcode
 */
typedef struct rc_matrix_t{
	int rows;	///< number of rows in the matrix
	int cols;	///< number of columns in the matrix
	double** d;	///< pointer to allocated 2d array
	int initialized;///< set to 1 once memory has been allocated
} rc_matrix_t;

#define RC_MATRIX_INITIALIZER {\
	.rows = 0,\
	.cols = 0,\
	.d = NULL,\
	.initialized = 0}

/**
 * @brief      Returns an rc_matrix_t with no allocated memory and the
 * initialized flag set to 0.
 *
 * This is essential for initializing rc_matrix_t structs when they are declared
 * since local variables declared in a function without global variable scope in
 * C are not guaranteed to be zeroed out which can lead to bad memory pointers
 * and segfaults if not handled carefully. We recommend initializing all
 * matrices with this before using rc_matrix_alloc or any other function.
 *
 * @return     Returns an empty rc_matrix_t
 */
rc_matrix_t rc_matrix_empty(void);

/**
 * @brief      Allocates memory for matrix A to have size rows&cols.
 *
 * If A is initially the right size, nothing is done and the data in A is
 * preserved. If A is uninitialized or of the wrong size then any existing
 * memory is freed and new memory is allocated, helping to prevent accidental
 * memory leaks. The contents of the new matrix is not guaranteed to be anything
 * in particular as the memory is allocated with malloc. Will only be
 * unsuccessful if rows&cols are invalid or there is insufficient memory
 * available.
 *
 * @param      A     Pointer to user's matrix struct
 * @param[in]  rows  number of rows
 * @param[in]  cols  number of columns
 *
 * @return     0 on success, -1 on failure.
 */
int rc_matrix_alloc(rc_matrix_t* A, int rows, int cols);

/**
 * @brief      Frees the memory allocated for a matrix A
 *
 * Also sets the dimensions and initialized flag to 0 to indicate to other
 * functions that A no longer points to allocated memory and cannot be used
 * until more memory is allocated such as with rc_matrix_alloc or
 * rc_matrix_zeros. Will only fail and return -1 if it is passed a NULL pointer.
 *
 * @param      A     Pointer to user's matrix struct
 *
 * @return     0 on success, -1 on failure.
 */
int rc_matrix_free(rc_matrix_t* A);

/**
 * @brief      Resizes matrix A and allocates memory for a matrix with specified rows &
* columns. The new memory is pre-filled with zeros using calloc. Any existing memory
* allocated for A is freed if necessary to avoid memory leaks.
 *
 * @param      A     Pointer to user's matrix struct
 * @param[in]  rows  number of rows
 * @param[in]  cols  number of columns
 *
 * @return     0 on success, -1 on failure.
 */
int rc_matrix_zeros(rc_matrix_t* A, int rows, int cols);

/**
 * @brief      Resizes A to be a square identity matrix with dimensions
 * dim-by-dim.
 *
 * Any existing memory allocated for A is freed if necessary to avoid memory
 * leaks before new memory is allocated for the specified dimension.
 *
 * @param      A     Pointer to user's matrix struct
 * @param[in]  dim   The dimension of one side of square matrix
 *
 * @return     0 on success, -1 on failure.
 */
int rc_matrix_identity(rc_matrix_t* A, int dim);

/**
 * @brief      Generates a matrix populated with random numbers between -1 and
 * 1.
 *
 * Resizes A to be a matrix with the specified number of rows and columns and
 * populates the new memory with random numbers evenly distributed between -1.0
 * and 1.0. Any existing memory allocated for A is freed if necessary to avoid
 * memory leaks.
 *
 * @param      A     Pointer to user's matrix struct
 * @param[in]  rows  number of rows
 * @param[in]  cols  number of columns
 *
 * @return     0 on success, -1 on failure.
 */
int rc_matrix_random(rc_matrix_t* A, int rows, int cols);

/**
 * @brief      Generates a diagonal matrix with the elements of specified vector
 * v.
 *
 * Resizes A to be a square matrix with the same number of rows and columns as
 * vector v's length. The diagonal entries of A are then populated with the
 * contents of v and the off-diagonal entries are set to 0. The original
 * contents of A are freed to avoid memory leaks.
 *
 * @param      A     Pointer to user's matrix struct
 * @param[in]  v     vector of diagonal entries
 *
 * @return     0 on success, -1 on failure.
 */
int rc_matrix_diagonal(rc_matrix_t* A, rc_vector_t v);

/**
 * @brief      Duplicates the contents of matrix A and into matrix B.
 *
 * If B is already the right size then its contents are overwritten. If B is
 * unallocated or is of the wrong size then the memory is freed and new memory
 * is allocated to hold the duplicate of A.
 *
 * @param[in]  A     Matrix to be duplicated
 * @param[out] B     new matrix
 *
 * @return     0 on success, -1 on failure.
 */
int rc_matrix_duplicate(rc_matrix_t A, rc_matrix_t* B);

/**
 * @brief      Prints the contents of matrix A to stdout in decimal notation
 * with 4 decimal places.
 *
 * Not recommended for very large matrices as rows will typically linewrap if
 * the terminal window is not wide enough.
 *
 * @param[in]  A     Matrix to print
 *
 * @return     0 on success, -1 on failure.
 */
int rc_matrix_print(rc_matrix_t A);

/**
 * @brief      Prints the contents of matrix A to stdout in scientific notation.
 *
 * Prints 4 significant figures. Not recommended for very large matrices as rows
 * will typically linewrap if the terminal window is not wide enough.
 *
 * @param[in]  A     Matrix to print
 *
 * @return     0 on success, -1 on failure.
 */
int rc_matrix_print_sci(rc_matrix_t A);

/**
 * @brief      Sets all values of an already-allocated matrix to 0
 *
 * @param      A     pointer to matrix to be zero'd out
 *
 * @return     0 on success, -1 on failure.
 */
int rc_matrix_zero_out(rc_matrix_t* A);

/**
 * @brief      Multiplies every entry in A by scalar value s.
 *
 * It is not strictly necessary for A to be provided as a pointer since a copy
 * of the struct A would also contain the correct pointer to the original
 * matrix's allocated memory. However, in this library we use the convention of
 * passing an rc_vector_t struct or rc_matrix_t struct as a pointer when its
 * data is to be modified by the function, and as a normal argument when it is
 * only to be read by the function.
 *
 * @param      A     Matrix to be modified
 * @param[in]  s     scalar to multiply by
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_matrix_times_scalar(rc_matrix_t* A, double s);

/**
 * @brief      Multiplies A*B=C.
 *
 * C is resized and its original contents are freed if necessary to avoid memory
 * leaks.
 *
 * @param[in]  A     first input
 * @param[in]  B     second input
 * @param[out] C     result
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_matrix_multiply(rc_matrix_t A, rc_matrix_t B, rc_matrix_t* C);

/**
 * @brief      Multiplies A*B and puts the result back in the place of B.
 *
 * B is resized and its original contents are freed if necessary to avoid memory
 * leaks.
 *
 * @param[in]  A     left matrix in the multiplication
 * @param      B     right matrix in the multiplication and holder of the
 * result.
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_matrix_left_multiply_inplace(rc_matrix_t A, rc_matrix_t* B);

/**
 * @brief      Multiplies A*B and puts the result back in the place of A.
 *
 * A is resized and its original contents are freed if necessary to avoid memory
 * leaks.
 *
 * @param      A     left matrix in the multiplication and holder of result
 * @param[in]  B     right matrix in the multiplication
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_matrix_right_multiply_inplace(rc_matrix_t* A, rc_matrix_t B);

/**
 * @brief      Adds matrices A+B and places the result in C.
 *
 * The original contents of C are safely freed if necessary to avoid memory
 * leaks. Use rc_matrix_add_inplace if you do not need to keep the contents of
 * one of these matrices after addition.
 *
 * @param[in]  A     First matrix
 * @param[in]  B     second matrix
 * @param[out] C     result
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_matrix_add(rc_matrix_t A, rc_matrix_t B, rc_matrix_t* C);

/**
 * @brief      Adds matrix A to B and places the result back in A.
 *
 * The original contents of A are lost. Use rc_matrix_add if you wish to keep
 * the contents of both matrix A and B after addition.
 *
 * @param      A     First matrix for addition and holder of the result
 * @param[in]  B     Second matrix for addition
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_matrix_add_inplace(rc_matrix_t* A, rc_matrix_t B);

/**
 * @brief      Subtracts matrix B from A and leaves the result in A
 *
 * The original contents of A are lost.
 *
 * @param      A     First matrix for subtraction and holder of the result
 * @param[in]  B     Second matrix for subtraction
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_matrix_subtract_inplace(rc_matrix_t* A, rc_matrix_t B);

/**
 * @brief      Transposes the contents of A and places the result in T.
 *
 * Resizes matrix T to hold the transposed contents of A and leaves A untouched.
 * Original contents of T are safely freed and lost. If the original contents of
 * A are not needed after transposing then use rc_matrix_transpose_inplace
 * instead.
 *
 * @param[in]  A     input matrix struct
 * @param[out] T     resulting transpose
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_matrix_transpose(rc_matrix_t A, rc_matrix_t* T);

/**
 * @brief      Transposes matrix A in place.
 *
 * Use as an alternative to rc_matrix_transpose if you no longer have need for
 * the original contents of matrix A.
 *
 * @param      A     Pointer to matrix to be transposed
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_matrix_transpose_inplace(rc_matrix_t* A);


/**
 * @brief      Multiplies matrix A times column vector v and places the result
 * in column vector c.
 *
 * Any existing data in c is freed if necessary and c is resized appropriately.
 * Vectors v and c are interpreted as column vectors, but nowhere in their
 * definitions are they actually specified as one or the other.
 *
 * @param[in]  A     input matrix
 * @param[in]  v     input vector
 * @param[out] c     output vector
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int   rc_matrix_times_col_vec(rc_matrix_t A, rc_vector_t v, rc_vector_t* c);

/**
 * @brief      Multiplies row vector v times matrix A and places the result in
 * row vector c.
 *
 * Any existing data in c is freed if necessary and c is resized appropriately.
 * Vectors v and c are interpreted as row vectors, but nowhere in their
 * definitions are they actually specified as one or the other.
 *
 * @param[in]  v     input vector
 * @param[in]  A     input matrix
 * @param[out] c     output vector
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int   rc_matrix_row_vec_times_matrix(rc_vector_t v, rc_matrix_t A, rc_vector_t* c);

/**
 * @brief      Computes v1 times v2 where v1 is a column vector and v2 is a row
 * vector.
 *
 * @param[in]  v1    Column vector v1
 * @param[in]  v2    Row vector v2
 * @param      A     Output matrix
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_matrix_outer_product(rc_vector_t v1, rc_vector_t v2, rc_matrix_t* A);


/**
 * @brief      Calculates the determinant of square matrix A
 *
 * @param[in]  A     input matrix
 *
 * @return     Returns the determinant or prints error message and returns -1.0f
 * of error.
 */
double rc_matrix_determinant(rc_matrix_t A);

/**
 * @brief      Symmetrizes a square matrix
 *
 * P_sym = (P+P^T)/2
 *
 * @param      P     pointer to matrix to symmetrize
 *
 * @return     0 on success, -1 on failure
 */
int rc_matrix_symmetrize(rc_matrix_t* P);


#ifdef __cplusplus
}
#endif

#endif // RC_MATRIX_H

/** @} end group math*/
