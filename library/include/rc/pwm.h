/**
 * <rc/pwm.h>
 *
 * @brief C interface for the Sitara PWM driver
 *
 * These functions provide a general interface to all 3 PWM subsystems, each of
 * which have two available channels A and B. PWM subsystems 1 and 2 are used
 * for controlling the 4 motor drivers on the Robotics Cape, however they may be
 * controlled by the user directly instead of using the motor API. PWM subsystem
 * 0 channels A and B can be accessed on the GPS header if set up with the
 * Pinmux API to do so. The user may have exclusive use of that subsystem.
 *
 *
 * @author     James Strawson
 * @date       1/31/2018
 *
 * @addtogroup PWM
 * @ingroup IO
 * @{
 */

#ifndef RC_PWM_H
#define RC_PWM_H

#ifdef  __cplusplus
extern "C" {
#endif


/**
 * @brief      Configures subsystem 0, 1, or 2 to operate at a particular
 * frequency.
 *
 * This may be called at runtime to change the pwm frequency without stopping
 * the motors or pwm signal.
 *
 * @param[in]  ss         Subsystem 0 1 or 2
 * @param[in]  frequency  The frequency in HZ
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_pwm_init(int ss, int frequency);

/**
 * @brief      Stops a subsystem and puts it into a low-power state.
 *
 * Recommended to call before userspace program exits to ensure the PWM hardware
 * stops.
 *
 * @param[in]  ss    subsystem 0,1,2
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_pwm_cleanup(int ss);

/**
 * @brief      Sets the duty cycle of a specific pwm channel.
 *
 * @param[in]  ss    subsystem 0,1,2
 * @param[in]  ch    channel 'A' or 'B'
 * @param[in]  duty  between 0.0 (off) and 1.0(full on)
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_pwm_set_duty(int ss, char ch, double duty);

/**
 * @brief      Like rc_pwm_set_duty() but takes a pulse width in nanoseconds.
 *
 * duty_ns which must range from 0 (off) to the number of nanoseconds in a
 * single cycle as determined by the freqency specified when calling
 * rc_pwm_init(). For example, a pwm frequency of 25kz corresponds to a maximum
 * pulse width of 40,000ns.
 *
 * @param[in]  ss       subsystem 0,1,2
 * @param[in]  ch       channel 'A' or 'B'
 * @param[in]  duty_ns  The duty cycle (pulse width) in nanoseconds
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_pwm_set_duty_ns(int ss, char ch, unsigned int duty_ns);


#ifdef __cplusplus
}
#endif

#endif // RC_PWM_H

/** @} end group PWM*/