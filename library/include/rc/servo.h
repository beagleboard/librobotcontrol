/**
 * <rc/servo.h>
 *
 * @brief      Control Servos and Brushless Motor Controllers.
 *
 * The Robotics Cape has 8 3-pin headers for connecting hobby servos and ESCs.
 * These are driven by the PRU for extremely precise signaling with minimal CPU
 * use. Standard 3-pin servo connectors are not polarized so pay close attention
 * to the symbols printed in white silkscreen on the cape before plugging
 * anything in. The black/brown ground wire should always be closest to the cape
 * PCB. The pinnout for these standard 3 pin connectors is as follows.
 *
 * - 1 Ground
 * - 2 6V Power
 * - 3 Pulse width signal
 *
 * Both servos and Brushless ESCs expect pulse width signals corresponding to
 * the desired output position or speed. These pulses normally range from 900us
 * to 2100us which usually corresponds to +- 60 degrees of rotation from the
 * neutral position. 1500us usually corresponds to the center position. Many
 * servos work up to +- 90 degrees when given pulse widths in the extended range
 * from 600us to 2400us. Test the limits of your servos very carefully to avoid
 * stalling the servos motors.
 *
 * | Normalized Width |  Pulse Width  |  Servo Angle  |
 * |:----------------:|:-------------:|:-------------:|
 * |         -1.5     |     600us     |    90 deg ccw |
 * |         -1.0     |     900us     |    60 deg ccw |
 * |          0.0     |    1500us     |    centered   |
 * |          1.0     |    2100us     |    60 deg cw  |
 * |          1.5     |    2400us     |    90 deg cw  |
 *
 * Unlike PWM which is concerned with the ratio of pulse width to pulse
 * frequency, servos and ESCs are only concerned with the pulse width and can
 * tolerate a wide range of update frequencies. Servos can typically tolerate
 * update pulses from 5-50hz with more expensive digital models sometimes
 * capable of higher update rates. Brushless ESCs are much more tolerant and
 * typically accept update rates up to 200hz with some multirotor ESCs capable
 * of 400hz when using sufficiently short pulse widths.
 *
 * Since ESCs drive motor unidirectionally, it makes more sense to think of
 * their normalized throttle as ranging from 0.0 (stopped) to 1.0 (full power).
 * Thus, these functions translate a normalized value from 0.0 to 1.0 to a pulse
 * width between 1000us and 2000us which is a common factory-calibration range
 * for many ESCs. We suggest using the rc_calibrate_escs example program on all
 * ESCs used with the robotics cape to ensure they are calibrated to this exact
 * pulse range.
 *
 * We HIGHLY recommend the use of ESCs which use the BLHeli firmware because
 * this firmware allows the input pulse range to be programmed to exactly
 * 1000-2000us and the old fashioned calibration mode to be disabled. This
 * prevents accidental triggering of calibration mode during use and removes the
 * need to run rc_calibrate_escs. BLHeli includes a plethora of configurable
 * settings and features such as easily adjustable timing and sounds. More
 * information on the BLHeli open source project here.
 *
 * Unless calibration mode is disabled, most ESCs will go into a failsafe or
 * calibration mode if the first signals they receive when powered up are not
 * their calibrated minimum pulse width corresponding to the throttle-off
 * condition. Therefore it is necessary for your program to start sending pulses
 * with a normalized value of 0.0 for a second or more before sending any other
 * value to ensure expected operation.
 *
 * Some ESCs, including those running BLHeli firmware, will wake up but keep the
 * motor idle when receiving pulses slightly below the minimum. This is largely
 * undocumented but we call this "idle" mode. For this reason we allow inputs to
 * rc_send_esc_pulse_normalized and rc_send_esc_pulse_normalized_all to range
 * from -0.1 to 1.0 where 0.0 results in the lowest throttle the ESC allows and
 * -0.1 can be used for idle where the motor is entirely powered off but the ESC
 * is awake.
 *
 * A recent trend among ESCs is support of "One-Shot" mode which shrinks the
 * pulse range down to 125-250us for reduced latency. Like
 * rc_send_esc_pulse_normalized, these oneshot equivalents also take a range
 * from -0.1 to 1.0 to allow for idle signals.
 *
 * @author     James Strawson
 * @date       3/7/2018
 *
 * @addtogroup Servo
 * @{
 */


#ifndef RC_SERVO_H
#define RC_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#define RC_SERVO_CH_MIN	1 ///< servo channels range from 1-8
#define RC_SERVO_CH_MAX	8 ///< servo channels range from 1-8
#define RC_SERVO_CH_ALL	0 ///< providing this as an argument writes the same pulse to all channels



#define RC_ESC_DEFAULT_MIN_US	1000
#define RC_ESC_DEFAULT_MAX_US	2000
#define RC_ESC_DJI_MIN_US	1120
#define RC_ESC_DJI_MAX_US	1920

/**
 * @brief      Configures the PRU to send servo pulses
 *
 * Also leaves the servo power rail OFF, turn back on with
 * rc_servo_power_rail_en(1) if you need to power servos off of the board.
 *
 * @return     0 on success, -1 on failure
 */
int rc_servo_init(void);

/**
 * @brief      Cleans up servo functionality and turns off the power rail.
 *
 * @return     0 on success, -1 on failure
 */
void rc_servo_cleanup(void);

/**
 * @brief      enables or disables the 6V power rail to drive servos.
 *
 * The Robotics Cape has a 6V 4A high-efficiency switching regulator to power
 * servos from the 2 cell LiPo battery. DO NOT enable this when using
 * BEC-enabled brushless ESCs as it may damage them. Since brushless ESCs only
 * need the ground and signal pins, it is safest to simply cut or disconnect the
 * middle power wire. This will allow the use of servos and ESCs at the same
 * time. Use the enable and disable functions above to control the power rail in
 * software.
 *
 * ALso use this to turn off power to the servos for example when the robot is
 * in a paused state to save power or prevent noisy servos from buzzing.
 *
 * @param[in]  en    0 to disable, non-zero to enable
 *
 * @return     0 on success, -1 on failure
 */
int rc_servo_power_rail_en(int en);


/**
 * @brief      Sets the pulse width range used by the
 * rc_servo_esc_send_pulse_normalized() function.
 *
 * This function is not necessary when using the default range which is
 * RC_ESC_DEFAULT_MIN_US (1000) to RC_ESC_DEFAULT_MAX_US (2000). This is only
 * neccessary when using custom ranges. The most common need for this is when
 * dealing with DJI motor drivers which cannot be calibrated. In this case use
 * the line:
 *
 * rc_servo_set_esc_range(RC_ESC_DJI_MIN_US, RC_ESC_DJI_MAX_US);
 *
 * This will set the range to 1120-1920 for DJI motor drivers. Note that the
 * minimum value is what is sent when calling rc_servo_usc_send_pulse_normalized
 * with a desired motor control input of 0. A slightly negative value is still
 * possible which will send a pulse width shorter than the minimum value given
 * here. These negative values shoul dbe avoided with DJI motor drivers as they
 * don't register.
 *
 * @param[in]  min   The minimum pulse width in microseconds
 * @param[in]  max   The maximum pulse width in microseconds
 *
 * @return     0 on success, -1 on failure.
 */
int rc_servo_set_esc_range(int min, int max);


/**
 * @brief      Sends a single pulse of desired width in microseconds to one or
 * all channels.
 *
 * This function returns right away and the PRU manages the accurate timing of
 * the pulse in the background. Therefore calling this function succesively for
 * each channel will start the pulse for each channel at approximately the same
 * time.
 *
 * As described above, servos and ESCs require regular pulses of at least 5hz to
 * function. Since these pulses do not have to be accurate in frequency, the
 * user can use these functions to start pulses from a userspace program at
 * convenient locations in their program, such as immediately when new positions
 * are calculated from sensor values.
 *
 * @param[in]  ch    Channel to send signal to (1-8) or 0 to send to all
 * channels.
 * @param[in]  us    Pulse Width in microseconds
 *
 * @return     0 on success, -1 on failure
 */
int rc_servo_send_pulse_us(int ch, int us);


/**
 * @brief      Like rc_send_pulse_us but translates a desired servo position
 * from -1.5 to 1.5 to a corresponding pulse width from 600 to 2400us.
 *
 * We cannot gurantee all servos will operate over the full range from -1.5 to
 * 1.5 as that is normally considered the extended range. -1.0 to 1.0 is a more
 * typical safe range but may not utilize the full range of all servos.
 *
 * @param[in]  ch     Channel to send signal to (1-8) or 0 to send to all
 * channels.
 * @param[in]  input  normalized position from -1.5 to 1.5
 *
 * @return     0 on success, -1 on failure
 */
int rc_servo_send_pulse_normalized(int ch, double input);


/**
 * @brief      Like rc_send_pulse_normalized but translates a desired esc
 * throttle position from 0 to 1.0 to a corresponding pulse width from 1000 to
 * 2000us.
 *
 * This only works as expected if your ESCs are calibrated to accept pulse
 * widths from 1000-2000us. This is best done with an ESC programming tool but
 * can also be done with the rc_calibrate_escs example program that comes
 * installed with this package.
 *
 * While the normal operating range for the normalized input is 0.0 to 1.0,
 * inputs as low as -0.1 are allowed. This is because many ESC firmwares such as
 * BLHeli will still turn or chirp the motors at 0.0 throttle, but will be
 * stationary and still armed and awake with throttle values slightly lower. We
 * suggest using a throttle of -0.1 for at least a second at the beginnig of
 * your program to wake the ESCs up from sleep but still keep the motors still.
 *
 * @param[in]  ch     Channel to send signal to (1-8) or 0 to send to all
 * channels.
 * @param[in]  input  normalized position from -0.1 to 1.0
 *
 * @return     0 on success, -1 on failure
 */
int rc_servo_send_esc_pulse_normalized(int ch, double input);


/**
 * @brief      Like rc_send_pulse_normalized but translates a desired esc
 * throttle position from 0 to 1.0 to a corresponding pulse width from 125 to
 * 250us.
 *
 * A recent trend among ESCs is support of "One-Shot" mode which shrinks the
 * pulse range down to 125-250us for reduced latency. If you are sure your ESCs
 * support this then you may try this function.
 *
 * While the normal operating range for the normalized input is 0.0 to 1.0,
 * inputs as low as -0.1 are allowed. This is because many ESC firmwares such as
 * BLHeli will still turn or chirp the motors at 0.0 throttle, but will be
 * stationary and still armed and awake with throttle values slightly lower. We
 * suggest using a throttle of -0.1 for at least a second at the beginnig of
 * your program to wake the ESCs up from sleep but still keep the motors still.
 *
 * @param[in]  ch     Channel to send signal to (1-8) or 0 to send to all
 * channels.
 * @param[in]  input  normalized position from -0.1 to 1.0
 *
 * @return     0 on success, -1 on failure
 */
int rc_servo_send_oneshot_pulse_normalized(int ch, double input);


#ifdef __cplusplus
}
#endif

#endif // RC_SERVO_H

/** @}  end group Servo */