/**
 * <rc/pthread.h>
 *
 * @brief      manage pthreads and process niceness
 *
 * @author     James Strawson
 * @date       1/19/2018
 *
 * @addtogroup pthread
 * @{
 */


#ifndef RC_PTHREAD_H
#define RC_PTHREAD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <pthread.h>

/**
 * @brief      starts a pthread with specified policy and priority
 *
 * Note that using anything other than SCHED_OTHER with priority 0 is only
 * available to root users or after giving special permission to the executable
 * with the setcap command line utility.
 *
 * If a policy is selected which requires privaldges the user does not have,
 * then a friendly warning will be displayed and the thread will still be set
 * with the priority and scheduler inherited from the calling process.
 *
 * @param      thread    pointer to user's pthread_t handle
 * @param      func      function pointer for thread to start
 * @param      arg       argument to pass to thread function when it starts
 * @param[in]  policy    SCHED_FIFO SCHED_RR or SCHED_OTHER
 * @param[in]  priority  between 1-99 for FIFO and RR, defualt 0 for SCHED_OTHER
 *
 * @return     0 on success or -1 on error
 */
int rc_pthread_create(pthread_t *thread, void*(*func)(void*), void *arg, int policy, int priority);


/**
 * @brief      Joins a thread with timeout given in seconds.
 *
 * If no timeout is necessary, just use the standard system pthread_join
 * function.
 *
 * @param[in]  thread       pthread_t handle
 * @param      retval       place to put the exit status of target thread, see
 * pthread_join
 * @param[in]  timeout_sec  floating point timeout in seconds
 *
 * @return     Returns 0 if the thread joined within the timeout period, 1 if
 * the thread timed out and was forced to close, -1 on error.
 */
int rc_pthread_timed_join(pthread_t thread, void** retval, float timeout_sec);


/**
 * @brief      Prints human-readable scheduler and priority for a pthread_t
 * handle
 *
 * @param[in]  thread  pthread_t handle
 *
 * @return     0 on success or -1 on failure
 */
int rc_pthread_print_properties(pthread_t thread);


/**
 * @brief      Lets a thread change it's own scheduler policy and priority while
 * running
 *
 * @param[in]  policy    SCHED_FIFO SCHED_RR or SCHED_OTHER
 * @param[in]  priority  between 1-99 for FIFO and RR, defualt 0 for SCHED_OTHER
 *
 * @return     0 on success or -1 on failure
 */
int rc_pthread_set_properties_self(int policy, int priority);


/**
 * @brief      fetches the niceness value of the executing process
 *
 * This is just a helpful wrapper for the system getpriority function and
 * returns the same value.
 *
 * @return     niceness (-20 to 20) or -1 on failure and sets errno
 */
int rc_pthread_get_process_niceness(void);


/**
 * @brief      Lets a process change its own niceness value
 *
 * Note that this requires special privaledges and will print an error if run by
 * a normal user. This is just a helpful wrapper for the system setpriority
 * funtion and returns the same thing.
 *
 * @param[in]  niceness  new niceness (-20 to 20)
 *
 * @return     0 on success, -1 on failure
 */
int rc_pthread_set_process_niceness(int niceness);


#ifdef __cplusplus
}
#endif

#endif // RC_PTHREAD_HELPERS_H

///@} end group pthread_helpers