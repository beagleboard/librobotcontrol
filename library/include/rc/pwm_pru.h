/**
 * <rc/pwm_pru.h>
 *
 * @brief PWM implementation on PRU based on pwm.h
 *
 * @author     Jason Kridner
 * @date       4/25/2022
 *
 * @addtogroup PWM
 * @ingroup PRU
 * @{
 */

#ifndef RC_PWM_PRU_H
#define RC_PWM_PRU_H

#ifdef  __cplusplus
extern "C" {
#endif

#define RC_PWM_PRU_CH_MIN	1 ///< pwm channels range from 1-4
#define RC_PWM_PRU_CH_MAX	4 ///< pwm channels range from 1-4
#define RC_PWM_PRU_CH_ALL	0 ///< providing this as an argument writes the same pulse to all channels


/**
 * @brief      Configures to operate at a particular
 * frequency.
 *
 * This may be called at runtime to change the pwm frequency without stopping
 * the motors or pwm signal.
 *
 * @param[in]  frequency  The frequency in HZ
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_pwm_pru_init(int frequency);

/**
 * @brief      Stops a subsystem and puts it into a low-power state.
 *
 * Recommended to call before userspace program exits to ensure the PRU software
 * stops.
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_pwm_pru_cleanup();

/**
 * @brief      Sets the duty cycle of a specific pwm channel.
 *
 * @param[in]  ch    0-3
 * @param[in]  duty  between 0.0 (off) and 1.0(full on)
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_pwm_pru_set_duty(char ch, double duty);

/**
 * @brief      Like rc_pwm_set_duty() but takes a pulse width in nanoseconds.
 *
 * duty_ns which must range from 0 (off) to the number of nanoseconds in a
 * single cycle as determined by the freqency specified when calling
 * rc_pwm_init(). For example, a pwm frequency of 25kz corresponds to a maximum
 * pulse width of 40,000ns.
 *
 * @param[in]  ch       0-3
 * @param[in]  duty_ns  The duty cycle (pulse width) in nanoseconds
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_pwm_pru_set_duty_ns(char ch, unsigned int duty_ns);


#ifdef __cplusplus
}
#endif

#endif // RC_PWM_PRU_H

/** @} end group PWM*/
