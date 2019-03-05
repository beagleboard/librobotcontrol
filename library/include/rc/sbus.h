/**
 * <rc/sbus.h>
 *
 * @brief      SBUS radio interface
 *
 * SBUS is the Taranis standard for remote control plane and car
 * radios.  Typically, a receiver outputs pulse width modulated
 * signals to individual servos or ESCs over separate servo
 * connectors. Commonly a satellite-receiver is mounted on the outside
 * of an RC plane to provide better signal strength and sends the same
 * information as a serial packet over a 3-pin wire. These receivers
 * have nothing to do with satellites but instead are named due to
 * their remote mounting away from the standard receiver.
 *
 * The Robotics Cape supports direct connection of satellite receivers
 * as well as binding to transmitters without the need for a standard
 * receiver and bind plug as is traditionally used. The software has
 * been tested with the X4R receiver.
 *
 * See rc_balance, rc_test_sbus, rc_bind_sbus, rc_calibrate_sbus, and
 * rc_sbus_passthroguh examples.
 *
 * @addtogroup SBUS
 * @{
 */

#ifndef RC_SBUS_H
#define RC_SBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h> // for int64_t

#define RC_MAX_SBUS_ANALOG_CHANNELS	16
#define RC_MAX_SBUS_BINARY_CHANNELS	2

/**
 * @brief      Starts the SBUS background service
 *
 * @return     0 on success, -1 on failure
 */
int rc_sbus_init(void);


/**
 * @brief      stops the SBUS background service
 *
 * @return     0 on success, -1 on failure. 1 if there was a timeout due to user
 * callback function not returning.
 */
int rc_sbus_cleanup(void);


/**
 * @brief      Returns the pulse width in microseconds commanded by the
 * transmitter for a particular channel.
 *
 * The user can specify channels 1 through 16 but non-zero values will only be
 * returned for channels the transmitter is actually using. The raw values in
 * microseconds typically range from 900-2100us for a standard radio with
 * default settings.
 *
 * @param[in]  ch    channel (1-16)
 *
 * @return     pulse width in microseconds if data is being transmitted, 0 if
 * data is not being transmitted on that channel, -1 on error
 */
int rc_sbus_ch_raw(int ch);


/**
 * @brief	Returns the binary value commanded by the transmitter for a
 * particular channel.
 *
 * The user can specify channels 1 through 2.
 *
 * @param[in]  ch    channel (1-2)
 *
 * @return     value
 */
int rc_sbus_ch_binary(int ch);


/**
 * @brief      Returns a scaled value from -1 to 1 corresponding to the min and
 * max values recorded during calibration.
 *
 * The user MUST run the rc_calibrate_sbus example to ensure the normalized
 * values returned by this function are correct. It is possible that values
 * outside of the range from -1 to 1 are returned if the calibration is not
 * perfect.
 *
 * @param[in]  ch    channel (1-16)
 *
 * @return     normalized input from -1.0 to 1.0 if that channel has data, 0 if
 * that channel has no data, -1 on error.
 */
double rc_sbus_ch_normalized(int ch);


/**
 * @brief      This is a check to see if new data is available.
 *
 * After new data is received this will return 1. It will return 0 as soon as
 * any channel has been read by either rc_sbus_ch_raw or rc_sbus_ch_normalized.
 *
 * @return     returns 1 if new data is ready to be read by the user. otherwise
 * returns 0
 */
int rc_sbus_is_new_data(void);


/**
 * @brief      Set your own callback function to be called when new SBUS data is
 * ready.
 *
 * @param[in]  func  callback function
 */
void rc_sbus_set_callback(void (*func)(void));


/**
 * @brief      Set your own callback function to be called when SBUS loses
 * connection.
 *
 * @param[in]  func  callback function
 */
void rc_sbus_set_disconnect_callback(void (*func)(void));


/**
 * @brief      Easily check on the state of the SBUS radio packets without
 * checking timeouts yourself.
 *
 * @return     returns 1 if packets are arriving in good health without
 * timeouts. returns 0 otherwise.
 */
int rc_sbus_is_connection_active(void);


/**
 * @brief      Measures time since the last SBUS packet was received.
 *
 * @return     Returns the number of nanoseconds since the last sbus packet was
 * received. Return -1 on error or if no packet has ever been received.
 */
int64_t rc_sbus_nanos_since_last_packet(void);


/**
 * @brief      Returns the total number of lost frames since rc_sbus_init.
 *	       Some errors also increment the number of lost frames.
 *
 * @return     total number of lost frames
 */
uint32_t rc_sbus_lost_frames (void);


/**
 * @brief      Returns the total number of receive errors since rc_sbus_init.
 *	       Some errors also increment the number of lost frames.
 *
 * @return     total number of errors
 */
uint32_t rc_sbus_total_errors (void);


/**
 * @brief      Estimates the current signal quality based on good/bad frames.
 *
 * @return     signal quality (0..100)
 */
int rc_sbus_signal_quality (void);


/**
 * @brief      routine for measuring the min and max values from a transmitter
 * on each channel and save to disk for future use.
 *
 * If a channel isn't used by the transmitter then default values are saved. if
 * the user forgot to move one of the channels during the calibration process
 * then defualt values are also saved.
 *
 * @return     0 on success, -1 on failure
 */
int rc_sbus_calibrate_routine(void);


#ifdef __cplusplus
}
#endif

#endif // RC_SBUS_H

/** @} end group SBUS */
