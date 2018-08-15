/**
 * @headerfile time.h <rc/time.h>
 *
 * @brief      sleep and timing functions
 *
 * All functions are POSIX compliant and should work on any linux system.
 *
 * @author     James Strawson
 * @date       1/31/2018
 *
 * @addtogroup time
 * @{
 */

#ifndef RC_TIME_H
#define RC_TIME_H

#ifdef  __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <time.h>	// for timespec
#include <sys/time.h>	// for timeval

typedef struct timespec	timespec;
typedef struct timeval timeval;

/**
 * @brief      Sleep in nanoseconds
 *
 * A wrapper for the normal UNIX nanosleep function which takes a number of
 * nanoseconds instead of a timeval struct. This also handles restarting
 * nanosleep with the remaining time in the event that nanosleep is interrupted
 * by a signal. There is no upper limit on the time requested.
 *
 * @param[in]  ns    nanoseconds to sleep
 */
void rc_nanosleep(uint64_t ns);


/**
 * @brief      Sleep in microseconds
 *
 * The traditional usleep function, however common, is deprecated in linux as it
 * uses SIGALARM which interferes with alarm and timer functions. This uses the
 * new POSIX standard nanosleep to accomplish the same thing which further
 * supports sleeping for lengths longer than 1 second. This also handles
 * restarting nanosleep with the remaining time in the event that nanosleep is
 * interrupted by a signal. There is no upper limit on the time requested.
 *
 * @param[in]  us    microseconds to sleep
 */
void rc_usleep(unsigned int us);


/**
 * @brief      Returns the number of nanoseconds since epoch using system
 * CLOCK_REALTIME
 *
 * This function itself takes about 1100ns to complete on a beaglebone at 1ghz
 * under ideal circumstances.
 *
 * @return     nanoseconds since epoch
 */
uint64_t rc_nanos_since_epoch(void);


/**
 * @brief      Returns the number of nanoseconds since system boot using
 * CLOCK_MONOTONIC
 *
 * This function itself takes about 1100ns to complete on a bealgebone at 1ghz
 * under ideal circumstances.
 *
 * @return     nanoseconds since system boot
 */
uint64_t rc_nanos_since_boot(void);


/**
 * @brief      Returns the number of nanoseconds from when when the calling
 * thread was started in CPU time.
 *
 * This uses CLOCK_THREAD_CPUTIME_ID. This time only increments when the
 * processor is working on the calling thread and not when the thread is
 * sleeping. This is usually for timing how long blocks of user-code take to
 * execute. This function itself takes about 2100ns to complete on a beaglebone
 * at 1ghz under ideal circumstances.
 *
 * @return     nanoseconds of CPU time since thread started
 */
uint64_t rc_nanos_thread_time(void);


/**
 * @brief      Returns a number of microseconds corresponding to a timespec
 * struct.
 *
 * Useful because timespec structs are annoying.
 *
 * @param[in]  ts    timespec struct to convert
 *
 * @return     time in microseconds
 */
uint64_t rc_timespec_to_micros(timespec ts);


/**
 * @brief      Returns a number of milliseconds corresponding to a timespec
 * struct.
 *
 * Useful because timespec structs are annoying.
 *
 * @param[in]  ts    timespec struct to convert
 *
 * @return     time in milliseconds
 */
uint64_t rc_timespec_to_millis(timespec ts);


/**
 * @brief      Returns a number of microseconds corresponding to a timeval
 * struct.
 *
 * Useful because timeval structs are annoying.
 *
 * @param[in]  tv    timeval struct to convert
 *
 * @return     time in microseconds
 */
uint64_t rc_timeval_to_micros(timeval tv);


/**
 * @brief      Returns a number of milliseconds corresponding to a timeval
 * struct.
 *
 * Useful because timespec structs are annoying.
 *
 * @param[in]  tv    timeval struct to convert
 *
 * @return     time in microseconds
 */
uint64_t rc_timeval_to_millis(timeval tv);


/**
 * @brief      Returns the time difference between two timespec structs as
 * another timespec.
 *
 *
 * Convenient for use with nanosleep() function and accurately timed loops.
 * Unlike timespec_sub defined in time.h, rc_timespec_diff does not care which
 * came first, A or B. A positive difference in time is always returned.
 *
 * @param[in]  A     timespec struct
 * @param[in]  B     timespec struct
 *
 * @return     timespec struct of the difference, always positive
 */
timespec rc_timespec_diff(timespec A, timespec B);


/**
 * @brief      Adds an amount of time in seconds to a timespec struct.
 *
 * The time added is a double-precision floating point value to make
 * respresenting fractions of a second easier. The timespec is passed as a
 * pointer so it can be modified in place. Seconds may be negative.
 *
 * @param      start    The start timspec, modified in place
 * @param[in]  seconds  The seconds
 */
void rc_timespec_add(timespec* start, double seconds);


#ifdef __cplusplus
}
#endif

#endif // RC_TIME_H

/** @} end group time*/