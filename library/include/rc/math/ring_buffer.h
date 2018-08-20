/**
 * <rc/math/ring_buffer.h>
 *
 * @brief      ring buffer implementation for double-precision floats
 *
 * Ring buffers are FIFO (first in first out) buffers of fixed length which
 * efficiently boot out the oldest value when full. They are particularly well
 * suited for storing the last n values in a discrete time filter.
 *
 * The user creates their own instance of a buffer and passes a pointer to the
 * these ring_buf functions to perform normal operations.
 *
 * @author     James Strawson
 * @date       2016
 *
 * @addtogroup Ring_Buffer
 * @ingroup    Math
 * @{
 */


#ifndef RC_RING_BUFFER_H
#define RC_RING_BUFFER_H

#ifdef  __cplusplus
extern "C" {
#endif


/**
 * @brief      Struct containing state of a ringbuffer and pointer to
 * dynamically allocated memory.
 */
typedef struct rc_ringbuf_t {
	double* d;	///< pointer to dynamically allocated data
	int size;	///< number of elements the buffer can hold
	int index;	///< index of the most recently added value
	int initialized;///< flag indicating if memory has been allocated for the buffer
} rc_ringbuf_t;

#define RC_RINGBUF_INITIALIZER {\
	.d = NULL,\
	.size = 0,\
	.index = 0,\
	.initialized = 0}

/**
 * @brief      Returns an rc_ringbuf_t struct which is completely zero'd out
 * with no memory allocated for it.
 *
 * This is essential for declaring new ring buffers since structs declared
 * inside of functions are not necessarily zero'd out which can cause the struct
 * to contain problematic contents leading to segfaults. New ring buffers should
 * be initialized with this before calling rc_ringbuf_alloc.
 *
 * @return     empty and ready-to-allocate rc_ringbuf_t
 */
rc_ringbuf_t rc_ringbuf_empty(void);

/**
 * @brief      Allocates memory for a ring buffer and initializes an
 * rc_ringbuf_t struct.
 *
 * If buf is already the right size then it is left untouched. Otherwise any
 * existing memory allocated for buf is freed to avoid memory leaks and new
 * memory is allocated.
 *
 * @param      buf   Pointer to user's buffer
 * @param[in]  size  Number of elements to allocate space for
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int   rc_ringbuf_alloc(rc_ringbuf_t* buf, int size);

/**
 * @brief      Frees the memory allocated for buffer buf.
 *
 * Also set the initialized flag to 0 so other functions don't try to access
 * unallocated memory.
 *
 * @param      buf   Pointer to user's buffer
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int   rc_ringbuf_free(rc_ringbuf_t* buf);

/**
 * @brief      Sets all values in the buffer to 0.0f and sets the buffer index
 * back to 0.
 *
 * @param      buf   Pointer to user's buffer
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int   rc_ringbuf_reset(rc_ringbuf_t* buf);

/**
 * @brief      Puts a new float into the ring buffer and updates the index
 * accordingly.
 *
 * If the buffer was full then the oldest value in the buffer is automatically
 * removed.
 *
 * @param      buf   Pointer to user's buffer
 * @param[in]  val   The value to be inserted
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int   rc_ringbuf_insert(rc_ringbuf_t* buf, double val);

/**
 * @brief      Fetches the float which is 'position' steps behind the last value
 * added to the buffer.
 *
 * If 'position' is given as 0 then the most recent value is returned. The
 * position obviously can't be larger than (buffer size - 1).
 *
 * @param      buf       Pointer to user's buffer
 * @param[in]  position  steps back in the buffer to fetch the value from
 *
 * @return     Returns the requested float. Prints an error message and returns
 * -1.0f on error.
 */
double rc_ringbuf_get_value(rc_ringbuf_t* buf, int position);

/**
 * @brief      Returns the standard deviation of all values in the ring buffer.
 *
 * Note that if the buffer has not yet been filled completely before calling
 * this, then the starting values of 0.0f in the unfilled portion of the buffer
 * will still be part of the calculation.
 *
 * @param[in]  buf   Pointer to user's buffer
 *
 * @return     Returns the standard deviation of all values in the ring buffer.
 */
double rc_ringbuf_std_dev(rc_ringbuf_t buf);


#ifdef __cplusplus
}
#endif

#endif // RC_RING_BUFFER_H

/** @} end group math*/