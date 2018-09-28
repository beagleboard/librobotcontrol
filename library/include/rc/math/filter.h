/**
 * <rc/math/filter.h>
 *
 * @brief      Functions for generating and implementing discrete SISO filters.
 *
 * This discrete filter implementation allows for easy implementation of
 * arbitrary transfer functions in code. It keeps track of all previous inputs
 * and outputs in ring buffers for efficiency. All computation is done using
 * single-precision doubleing point values.
 *
 * Several functions are providing for generating filter coefficients for the
 * most common functions such as low/high pass filters, moving average filters,
 * PID, and integrators.
 *
 * See the rc_test_filter.c example for use case.
 *
 * @author     James Strawson
 * @date       2016
 *
 * @addtogroup SISO_Filter
 * @ingroup    Math
 * @{
 */

#ifndef RC_FILTER_H
#define RC_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <rc/math/vector.h>
#include <rc/math/ring_buffer.h>

/**
 * @brief      Struct containing configuration and state of a SISO filter.
 *
 * Also points to dynamically allocated memory which make it necessary to use
 * the allocation and free function in this API for proper use. The user can
 * read and modify values directly from ths struct.
 */
typedef struct rc_filter_t{
	/** @name transfer function properties */
	///@{
	int order;		///< transfer function order
	double dt;		///< timestep in seconds
	double gain;		///< Additional gain multiplier, usually 1.0
	rc_vector_t num;	///< numerator coefficients
	rc_vector_t den;	///< denominator coefficients
	///@}

	/** @name saturation settings */
	///@{
	int sat_en;		///< set to 1 by enable_saturation()
	double sat_min;		///< lower saturation limit
	double sat_max;		///< upper saturation limit
	int sat_flag;		///< 1 if saturated on the last step
	///@}

	/** @name soft start settings */
	///@{
	int ss_en;		///< set to 1 by enbale_soft_start()
	double ss_steps;		///< steps before full output allowed
	///@}

	/** @name dynamically allocated ring buffers */
	///@{
	rc_ringbuf_t in_buf;
	rc_ringbuf_t out_buf;
	///@}

	/** @name other */
	///@{
	double newest_input;	///< shortcut for the most recent input
	double newest_output;	///< shortcut for the most recent output
	uint64_t step;		///< steps since last reset
	int initialized;	///< initialization flag
	///@}
} rc_filter_t;

#define RC_FILTER_INITIALIZER {\
	.order		= 0,\
	.dt		= 0.0,\
	.gain		= 1.0,\
	.num		= RC_VECTOR_INITIALIZER,\
	.den		= RC_VECTOR_INITIALIZER,\
	.sat_en		= 0,\
	.sat_min	= 0.0,\
	.sat_max	= 0.0,\
	.sat_flag	= 0,\
	.ss_en		= 0,\
	.ss_steps	= 0,\
	.in_buf		= RC_RINGBUF_INITIALIZER,\
	.out_buf	= RC_RINGBUF_INITIALIZER,\
	.newest_input	= 0.0,\
	.newest_output	= 0.0,\
	.step		= 0,\
	.initialized	= 0}

/**
 * @brief      Critical function for initializing rc_filter_t structs.
 *
 * This is a very important function. If your rc_filter_t struct is not a global
 * variable, then its initial contents cannot be guaranteed to be anything in
 * particular. Therefore it could contain problematic contents which could
 * interfere with functions in this library. Therefore, you should always
 * initialize your filters with rc_filter_empty before using with any other
 * function in this library such as rc_filter_alloc. This serves the same
 * purpose as rc_matrix_empty, rc_vector_empty, and rc_ringbuf_empty.
 *
 * @return     Empty zero-filled rc_filter_t struct
 */
rc_filter_t rc_filter_empty(void);

/**
 * @brief      Allocate memory for a discrete-time filter & populates it with
 * the transfer function coefficients provided in vectors num and den.
 *
 * The memory in num and den is duplicated so those vectors can be reused or
 * freed after allocating a filter without fear of disturbing the function of
 * the filter. Argument dt is the timestep in seconds at which the user expects
 * to operate the filter. The length of demonimator den must be at least as
 * large as numerator num to avoid describing an improper transfer function. If
 * rc_filter_t pointer f points to an existing filter then the old filter's
 * contents are freed safely to avoid memory leaks. We suggest initializing
 * filter f with rc_filter_empty before calling this function if it is not a
 * global variable to ensure it does not accidentally contain invlaid contents
 * such as null pointers. The filter's order is derived from the length of the
 * denominator polynomial.
 *
 * @param[out] f     Pointer to user's rc_filter_t struct
 * @param[in]  num   The numerator vector
 * @param[in]  den   The denomenator vector
 * @param[in]  dt    Timestep in seconds
 *
 * @return     0 on success or -1 on failure.
 */
int rc_filter_alloc(rc_filter_t* f, rc_vector_t num, rc_vector_t den, double dt);

/**
 * @brief      Like rc_filter_alloc(), but takes arrays for the numerator and
 * denominator coefficients instead of vectors.
 *
 * Arrays num and den must have lengths that form a proper or semi-proper
 * transfer function.
 *
 * @param[out] f       Pointer to user's rc_filter_t struct
 * @param[in]  dt      Timestep in seconds
 * @param[in]  num     pointer to numerator array
 * @param[in]  numlen  The numerator length
 * @param[in]  den     pointer to denominator array
 * @param[in]  denlen  The denominator length
 *
 * @return     0 on success or -1 on failure.
 */
int rc_filter_alloc_from_arrays(rc_filter_t* f,double dt,double* num,int numlen,\
							double* den,int denlen);

/**
 * @brief      duplicates a filter
 *
 * This allocates new memory in filter f so it can be used independently from
 * the old filter but with the same configuration.
 *
 * @param      f     new filter to allocate
 * @param[in]  old   old filter to copy
 *
 * @return     { description_of_the_return_value }
 */
int rc_filter_duplicate(rc_filter_t* f, rc_filter_t old);

/**
 * @brief      Frees the memory allocated by a filter's buffers and coefficient
 * vectors. Also resets all filter properties back to 0.
 *
 * @param      f     Pointer to user's rc_filter_t struct
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_filter_free(rc_filter_t* f);

/**
 * @brief      Prints the transfer function and other statistic of a filter to
 * the screen.
 *
 * Only works on filters up to order 9.
 *
 * @param      f     Pointer to user's rc_filter_t struct
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_filter_print(rc_filter_t f);

/**
 * @brief      March a filter forward one step with new input provided as an
 * argument.
 *
 * If saturation or soft-start are enabled then the output will automatically be
 * bound appropriately. The steps counter is incremented by one and internal
 * ring buffers are updated accordingly. Once a filter is created, this is
 * typically the only function required afterwards.
 *
 * @param      f          Pointer to user's rc_filter_t struct
 * @param[in]  new_input  The new input
 *
 * @return     Returns the new output which could also be accessed with the
 * newest_output field in the filter struct.
 */
double rc_filter_march(rc_filter_t* f, double new_input);

/**
 * @brief      Resets all previous inputs and outputs to 0. Also resets the step
 * counter & saturation flag.
 *
 * This is sufficient to start the filter again as if it were just created.
 *
 * @param      f     Pointer to user's rc_filter_t struct
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_filter_reset(rc_filter_t* f);

/**
 * @brief      Enables saturation between bounds min and max.
 *
 * If saturation is enabled for a specified filter, the filter will
 * automatically bound the output between min and max. You may ignore this
 * function if you wish the filter to run unbounded. Max must be greater than or
 * equal to min. If max==min, the output will be fixed at that value. Any
 * double-precision floating point value is allowed, positive or negative.
 *
 * @param      f     Pointer to user's rc_filter_t struct
 * @param[in]  min   The lower bound
 * @param[in]  max   The upper bound
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_filter_enable_saturation(rc_filter_t* f, double min, double max);

/**
 * @brief      Checks if the filter saturated the last time step.
 *
 * This information could also be retrieved by looking at the 'sat_flag' value
 * in the filter struct.
 *
 * @param      f     Pointer to user's rc_filter_t struct
 *
 * @return     Returns 1 if the filter saturated the last time step. Returns 0
 * otherwise.
 */
int rc_filter_get_saturation_flag(rc_filter_t* f);

/**
 * @brief      Enables soft start functionality where the output bound is
 * gradually opened linearly from 0 to the normal saturation range.
 *
 * This occurs over the time specified from argument 'seconds' from when the
 * filter is first created or reset. Saturation must already be enabled for this
 * to work. This assumes that the user does indeed call rc_filter_march at
 * roughly the same time interval as the 'dt' variable in the filter struct
 * which is set at creation time. The soft-start property is maintained through
 * a call to rc_filter_reset so the filter will soft-start again after each
 * reset. This feature should only really be used for feedback controllers to
 * prevent jerky starts. The saturation flag will not be set during this period
 * as the output is usually expected to be bounded and we don't want to falsely
 * trigger alarms or saturation counters.
 *
 * @param      f        Pointer to user's rc_filter_t struct
 * @param[in]  seconds  Time in seconds
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_filter_enable_soft_start(rc_filter_t* f, double seconds);

/**
 * @brief      Returns the input 'steps' back in time. Steps=0 returns most
 * recent input.
 *
 * 'steps' must be between 0 and order inclusively as those are the only steps
 * retained in memory for normal filter operation. To record values further back
 * in time we suggest creating your own rc_ringbuf_t ring buffer.
 *
 * @param      f      Pointer to user's rc_filter_t struct
 * @param[in]  steps  The steps back in time, steps=0 returns most recent input.
 *
 * @return     Returns the requested previous input. If there is an error,
 * returns -1.0f and prints an error message.
 */
double rc_filter_previous_input(rc_filter_t* f, int steps);

/**
 * @brief      Returns the output 'steps' back in time. Steps = 0 returns most
 * recent output.
 *
 * 'steps' must be between 0 and order inclusively as those are the only steps
 * retained in memory for normal filter operation. To record values further back
 * in time we suggest creating your own rc_ringbuf_t ring buffer.
 *
 * @param      f      Pointer to user's rc_filter_t struct
 * @param[in]  steps  The steps back in time, steps=0 returns most recent
 * output.
 *
 * @return     Returns the requested previous output. If there is an error,
 * returns -1.0f and prints an error message.
 */
double rc_filter_previous_output(rc_filter_t* f, int steps);

/**
 * @brief      Fills all previous inputs to the filter as if they had been equal
 * to 'in'
 *
 * Most useful when starting high-pass filters to prevent unwanted jumps in the
 * output when starting with non-zero input.
 *
 * @param      f     Pointer to user's rc_filter_t struct
 * @param[in]  in    Input value to fill
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_filter_prefill_inputs(rc_filter_t* f, double in);

/**
 * @brief      Fills all previous outputs of the filter as if they had been
 * equal to 'out'.
 *
 * Most useful when starting low-pass filters to prevent unwanted settling time
 * when starting with non-zero input.
 *
 * @param      f     Pointer to user's rc_filter_t struct
 * @param[in]  out   output value to fill
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_filter_prefill_outputs(rc_filter_t* f, double out);

/**
 * @brief      Creates a new filter 'out' by multiplying f1*f2.
 *
 * The contents of f3 are freed safely if necessary and new memory is allocated
 * to avoid memory leaks.
 *
 * @param[in]  f1    Pointer to user's rc_filter_t struct to be multiplied
 * @param[in]  f2    Pointer to user's rc_filter_t struct to be multiplied
 * @param[out] out   Pointer to newly created filter struct
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_filter_multiply(rc_filter_t f1, rc_filter_t f2, rc_filter_t* out);
/**
 * @brief      Creates a new filter 'out' by multiplying f1*f2*f3
 *
 * The contents of f3 are freed safely if necessary and new memory is allocated
 * to avoid memory leaks.
 *
 * @param[in]  f1    Pointer to user's rc_filter_t struct to be multiplied
 * @param[in]  f2    Pointer to user's rc_filter_t struct to be multiplied
 * @param[in]  f3    Pointer to user's rc_filter_t struct to be multiplied
 * @param[out] out   Pointer to newly created filter struct
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_filter_multiply_three(rc_filter_t f1, rc_filter_t f2, rc_filter_t f3, rc_filter_t* out);

/**
 * @brief      Creates a discrete time filter with similar dynamics to a
 * provided continuous time transfer function using tustin's approximation with
 * prewarping about a frequency of interest 'w' in radians per second.
 *
 * Any existing memory allocated for f is freed is necessary to prevent memory
 * leaks. Returns 0 on success or -1 on failure.
 *
 * @param[out] f     Pointer to user's rc_filter_t struct
 * @param[in]  dt    desired timestep of discrete filter in seconds
 * @param[in]  num   continuous time numerator coefficients
 * @param[in]  den   continuous time denominator coefficients
 * @param[in]  w     prewarping frequency in rad/s
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_filter_c2d_tustin(rc_filter_t* f, double dt,rc_vector_t num,rc_vector_t den,double w);

/**
 * @brief      Normalizes a discrete time filter so that the leading demoninator
 * coefficient is 1
 *
 * @param      f     Pointer to user's rc_filter_t struct
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_filter_normalize(rc_filter_t* f);

/**
 * @brief      Creates a first order low pass filter.
 *
 * Any existing memory allocated for f is freed safely to avoid memory leaks and
 * new memory is allocated for the new filter. dt is in units of seconds and
 * time_constant is the number of seconds it takes to rise to 63.4% of a
 * steady-state input. This can be used alongside rc_first_order_highpass to
 * make a complementary filter pair.
 *
 * @param[out] f     Pointer to user's rc_filter_t struct
 * @param[in]  dt    desired timestep of discrete filter in seconds
 * @param[in]  tc    time constant: Seconds it takes to rise to 63.4% of a
 * steady-state input
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_filter_first_order_lowpass(rc_filter_t* f, double dt, double tc);

/**
 * @brief      Creates a first order high pass filter.
 *
 * Any existing memory allocated for f is freed safely to avoid memory leaks and
 * new memory is allocated for the new filter. dt is in units of seconds and
 * time_constant is the number of seconds it takes to decay by 63.4% of a
 * steady-state input. This can be used alongside rc_first_order_highpass to
 * make a complementary filter pair.
 *
 * @param[out] f     Pointer to user's rc_filter_t struct
 * @param[in]  dt    desired timestep of discrete filter in seconds
 * @param[in]  tc    time constant: Seconds it takes to decay by 63.4% of a
 * steady-state input
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_filter_first_order_highpass(rc_filter_t* f, double dt, double tc);

/**
 * @brief      Creates a Butterworth low pass filter of specified order and
 * cutoff frequency.
 *
 * Any existing memory allocated for f is freed safely to avoid memory leaks and
 * new memory is allocated for the new filter.
 *
 * @param[out] f      Pointer to user's rc_filter_t struct
 * @param[in]  order  The order (>=1)
 * @param[in]  dt     desired timestep of discrete filter in seconds
 * @param[in]  wc     Cuttoff freqauency in rad/s
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_filter_butterworth_lowpass(rc_filter_t* f, int order, double dt, double wc);

/**
 * @brief      Creates a Butterworth high pass filter of specified order and
 * cutoff frequency.
 *
 * Any existing memory allocated for f is freed safely to avoid memory leaks and
 * new memory is allocated for the new filter.
 *
 * @param[out] f      Pointer to user's rc_filter_t struct
 * @param[in]  order  The order (>=1)
 * @param[in]  dt     desired timestep of discrete filter in seconds
 * @param[in]  wc     Cuttoff freqauency in rad/s
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_filter_butterworth_highpass(rc_filter_t* f, int order, double dt, double wc);

/**
 * @brief      Makes a FIR moving average filter that averages over specified
 * number of samples.
 *
 * Any existing memory allocated for f is freed safely to avoid memory leaks and
 * new memory is allocated for the new filter.
 *
 * Note that the timestep dt does not effect the dynamics of the produced
 * filter. It is simply copied into the 'dt' field of the rc_filter_t struct.
 * However, it is necessary for creation of this filter for compatability with
 * the soft-start feature and any other user codepaths that may be dependent on
 * a filter's timestep.
 *
 * @param[out] f        Pointer to user's rc_filter_t struct
 * @param[in]  samples  The samples to average over (>=2)
 * @param[in]  dt       desired timestep of discrete filter in seconds
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_filter_moving_average(rc_filter_t* f, int samples, double dt);

/**
 * @brief      Creates a first order integrator.
 *
 * Like most functions here, the dynamics are only accurate if the filter is
 * called with a timestep corresponding to dt. Any existing memory allocated for
 * f is freed safely to avoid memory leaks and new memory is allocated for the
 * new filter.
 *
 * @param[out] f     Pointer to user's rc_filter_t struct
 * @param[in]  dt    desired timestep of discrete filter in seconds
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_filter_integrator(rc_filter_t *f, double dt);

/**
 * @brief      Creates a second order double integrator.
 *
 * Like most functions here, the dynamics are only accurate if the filter is
 * called with a timestep corresponding to dt. Any existing memory allocated for
 * f is freed safely to avoid memory leaks and new memory is allocated for the
 * new filter.
 *
 * @param[out] f     Pointer to user's rc_filter_t struct
 * @param[in]  dt    desired timestep of discrete filter in seconds
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_filter_double_integrator(rc_filter_t* f, double dt);

/**
 * @brief      Creates a discrete-time implementation of a parallel PID
 * controller with high-frequency rolloff using the forward-Euler integration
 * method.
 *
 * This is equivalent to the Matlab function: C = pid(Kp,Ki,Kd,Tf,Ts)
 *
 * It is not possible to implement a pure differentiator with a discrete
 * transfer function so this filter has high frequency rolloff with time
 * constant Tf. Smaller Tf results in less rolloff, but Tf must be greater than
 * dt/2 for stability. Returns 0 on success or -1 on failure.
 *
 * @param      f     Pointer to user's rc_filter_t struct
 * @param[in]  kp    Proportional constant
 * @param[in]  ki    Integration constant
 * @param[in]  kd    Derivative constant
 * @param[in]  Tf    High Frequency rolloff time constant (seconds)
 * @param[in]  dt    desired timestep of discrete filter in seconds
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_filter_pid(rc_filter_t* f,double kp,double ki,double kd,double Tf,double dt);


/**
 * @brief      Creates a third order symmetric complementary pair of high/low
 * pass filters
 *
 * @param      lp    lowpass filter to be populated
 * @param      hp    highpass filter to be populated
 * @param[in]  freq  crossover frequency in rad/s
 * @param[in]  damp  Damping ratio >0, 1 is critically damped, lower than that
 * gets wobbly
 * @param[in]  dt    desired timestep of discrete filter in seconds
 *
 * @return     0 on success or -1 on failure.
 */
int rc_filter_third_order_complement(rc_filter_t* lp, rc_filter_t* hp, double freq, double damp, double dt);


#ifdef __cplusplus
}
#endif

#endif // RC_FILTER_H

/** @} end ingroup math*/