/**
 * <rc/start_stop.h>
 *
 * @brief      cleanly start and stop a process, signal handling, program flow
 *
 * It can be tricky to manage the starting and stopping of mutiple threads and
 * loops. Since the robot control library has several background threads in
 * addition to any user-created threads, we encourage the use of the
 * consolidated high-level process control method described here. These
 * functions allow easy implementation of good-practice process handling
 * including a global shutdown flag, a signal handler, making & deleting a PID
 * file, stopping previously running copies of a program, and stopping a program
 * when it's in the background.
 *
 * The rc_state_t struct tries to cover the majority of use cases in the context
 * of a robotics application. The user should start their program main()
 * function by setting the state to UNINITIALIZED and enabling the signal
 * handler before doing hardware initialization and starting threads. When the
 * user's own initialization sequence is complete they should set the flow state
 * to PAUSED or RUNNING to indicate to the newly created threads that the
 * program should now behave in normal ongoing operational mode.
 *
 * During normal operation, the user may elect to implement a PAUSED state where
 * the user's threads may keep running to read sensors but do not actuate
 * motors, leaving their robot in a safe state. For example, pressing the pause
 * button could be assigned to change this state back and forth between RUNNING
 * and PAUSED. This is entirely optional.
 *
 * The most important state here is EXITING. The signal handler described here
 * intercepts the SIGINT signal when the user presses Ctrl-C and sets the state
 * to EXITING. It is then up to the user's threads to watch for this condition
 * and exit cleanly. The user may also set the state to EXITING at any time to
 * trigger the closing of their own threads and robot control library's own
 * background threads.
 *
 * The flow state variable is kept in the robot control library's memory space
 * and should be read and modified by the rc_get_state() and rc_set_state()
 * functions above. The user may optionally use the rc_print_state() function to
 * print a human readable version of the state enum to the screen.
 *
 *
 * @addtogroup start_stop
 * @{
 */

#ifndef RC_START_STOP_H
#define RC_START_STOP_H

#ifdef __cplusplus
extern "C" {
#endif

#define RC_PID_DIR	"/run/shm/"
#define RC_PID_FILE	"/run/shm/robotcontrol.pid"

/**
 * @brief      process state variable, read and modify by rc_get_state,
 * rc_set_state, and rc_print_state. Starts as UNINITIALIZED.
 */
typedef enum rc_state_t {
	UNINITIALIZED,
	RUNNING,
	PAUSED,
	EXITING
} rc_state_t;


/**
 * @brief      fetches the current process state as set by the user or signal
 * handler
 *
 * @return     current process state
 */
rc_state_t rc_get_state(void);


/**
 * @brief      sets the current process state.
 *
 * Except for the signal handler setting the state to EXITING, this is the only
 * way that the process state can be changed.
 *
 * @param[in]  new_state  The new state
 */
void rc_set_state(rc_state_t new_state);


/**
 * @brief      prints the process state to stdout in human-readble form
 *
 * @return     0 on success, -1 on failure
 */
int rc_print_state(void);


/**
 * @brief      Makes a PID file RC_PID_FILE (/run/shm/robotcontrol.pid)
 * containing the current PID of your process
 *
 * @return     Returns 0 if successful. If that file already exists then it is
 * not touched and this function returns 1 at which point we suggest you run
 * rc_kill_exising_process() to kill that process. Returns -1 if there is some
 * other problem writing to the file.
 */
int rc_make_pid_file(void);


/**
 * @brief      This function is used to make sure any existing program using the
 * PID file is stopped.
 *
 * The user doesn't need to integrate this in their own program However, the
 * user may call the rc_kill example program from the command line to close
 * whatever program is running in the background.
 *
 * @param[in]  timeout_s  timeout period to wait for process to close cleanly,
 * must be >=0.1
 *
 * @return     return values:
 * - -4: invalid argument or other error
 * - -3: insufficient privileges to kill existing process
 * - -2: unreadable or invalid contents in RC_PID_FILE
 * - -1: existing process failed to close cleanly and had to be killed
 * -  0: No existing process was running
 * -  1: An existing process was running but it shut down cleanly.
 */
int rc_kill_existing_process(float timeout_s);


/**
 * @brief      Removes the PID file created by rc_make_pid_file().
 *
 * This should be called before your program closes to make sure it's not left
 * behind.
 *
 * @return     Returns 0 whether or not the file was actually there. Returns -1
 * if there was a file system error.
 */
int rc_remove_pid_file(void);


/**
 * @brief      Enables a generic signal handler. Optional but recommended.
 *
 * This catches SIGINT, SIGTERM, SIGHUP, and SIGSEGV and does the following:
 *
 * - SIGINT (ctrl-c) Sets process state to EXITING indicating to the user
 *   threads to shut down cleanly. All user threads should check rc_get_state to
 *   catch this.
 * - SITERM: Same as SIGINT above
 * - SIGHUP: Ignored to prevent process from stopping due to loose USB network
 *   connection. Also allows robot control programs to keep running after USB
 *   cable in intentionally removed.
 * - SIGSEGV:  Segfaults will be caught and print some debugging info to the
 *   screen before setting rc_state to EXITING. Behavior with segfaults is no
 *   guaranteed to be predictable.
 *
 * @return     Returns 0 on success or -1 on error
 */
int rc_enable_signal_handler(void);


/**
 * @brief      resets the system signal callbacks to defualt, generally this is
 * never needed unless you are intentionally changing signal handlers in the
 * middle of a program.
 *
 * @return     Returns 0 on success or -1 on error
 */
int rc_disable_signal_handler(void);


#ifdef __cplusplus
}
#endif

#endif // RC_START_STOP_H

/** @} end group start_stop*/

