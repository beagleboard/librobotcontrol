/**
 * <rc/dsm.h>
 *
 * @brief      DSM2 and DSMX radio interface
 *
 * DSM2 and DSMX are common standards for remote control plane and car radios.
 * Typically, a receiver outputs pulse width modulated signals to individual
 * servos or ESCs over separate servo connectors. Commonly a satellite-receiver
 * is mounted on the outside of an RC plane to provide better signal strength
 * and sends the same information as a serial packet over a 3-pin wire. These
 * receivers have nothing to do with satellites but instead are named due to
 * their remote mounting away from the standard receiver.
 *
 * The Robotics Cape supports direct connection of satellite receivers as well
 * as binding to transmitters without the need for a standard receiver and bind
 * plug as is traditionally used. The software has been tested with Orange brand
 * DSM2 receivers, as well as Spektrum and JR branded DSMX receivers.
 *
 * See rc_balance, rc_test_dsm, rc_bind_dsm, rc_calibrate_dsm, and
 * rc_dsm_passthroguh examples.
 *
 * @addtogroup DSM
 * @{
 */

#ifndef RC_DSM_H
#define RC_DSM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h> // for int64_t

#define RC_MAX_DSM_CHANNELS	9

/**
 * @brief      Starts the DSM background service
 *
 * @return     0 on success, -1 on failure
 */
int rc_dsm_init(void);


/**
 * @brief      stops the DSM background service
 *
 * @return     0 on success, -1 on failure. 1 if there was a timeout due to user
 * callback function not returning.
 */
int rc_dsm_cleanup(void);


/**
 * @brief      Returns the pulse width in microseconds commanded by the
 * transmitter for a particular channel.
 *
 * The user can specify channels 1 through 9 but non-zero values will only be
 * returned for channels the transmitter is actually using. The raw values in
 * microseconds typically range from 900-2100us for a standard radio with
 * default settings.
 *
 * @param[in]  ch    channel (1-9)
 *
 * @return     pulse width in microseconds if data is being transmitted, 0 if
 * data is not being transmitted on that channel, -1 on error
 */
int rc_dsm_ch_raw(int ch);


/**
 * @brief      Returns a scaled value from -1 to 1 corresponding to the min and
 * max values recorded during calibration.
 *
 * The user MUST run the rc_calibrate_dsm example to ensure the normalized
 * values returned by this function are correct. It is possible that values
 * outside of the range from -1 to 1 are returned if the calibration is not
 * perfect.
 *
 * @param[in]  ch    channel (1-9)
 *
 * @return     normalized input from -1.0 to 1.0 if that channel has data, 0 if
 * that channel has no data, -1 on error.
 */
double rc_dsm_ch_normalized(int ch);


/**
 * @brief      This is a check to see if new data is available.
 *
 * After new data is received this will return 1. It will return 0 as soon as
 * any channel has been read by either rc_dsm_ch_raw or rc_dsm_ch_normalized.
 *
 * @return     returns 1 if new data is ready to be read by the user. otherwise
 * returns 0
 */
int rc_dsm_is_new_data(void);


/**
 * @brief      Set your own callback function to be called when new DSM data is
 * ready.
 *
 * @param[in]  func  callback function
 */
void rc_dsm_set_callback(void (*func)(void));


/**
 * @brief      Set your own callback function to be called when DSM loses
 * connection.
 *
 * @param[in]  func  callback function
 */
void rc_dsm_set_disconnect_callback(void (*func)(void));


/**
 * @brief      Easily check on the state of the DSM radio packets without
 * checking timeouts yourself.
 *
 * @return     returns 1 if packets are arriving in good health without
 * timeouts. returns 0 otherwise.
 */
int rc_dsm_is_connection_active(void);


/**
 * @brief      Measures time since the last DSM packet was received.
 *
 * @return     Returns the number of nanoseconds since the last dsm packet was
 * received. Return -1 on error or if no packet has ever been received.
 */
int64_t rc_dsm_nanos_since_last_packet(void);


/**
 * @brief      Used to determine if DSM packets are arriving with 10 or 11-bit
 * resolution
 *
 * @return     returns 10 or 11 indicating 10-bit or 11-bit resolution returns a
 * 0 if no packet has been received yet or -1 on error
 */
int rc_dsm_resolution(void);


/**
 * @brief      fetches number of DSM channels currently being received.
 *
 * @return     Returns number of channels being received, 0 if no packet has
 * been received yet or -1 on error.
 */
int rc_dsm_channels(void);


/**
 * @brief      Begins the binding routine and prints instructions to the screen
 * along the way.
 *
 * The user doesn't need to call this function unless they really want to. Use
 * the rc_bind_dsm example program instead.
 *
 * DSM satellite receivers are put into bind mode by sending them a sequence of
 * pulses right after it receives power and starts up. This program puts the
 * normally UART signal pin into GPIO pulldown mode temporarily, detects when
 * the user unplugs and plugs back in the receiver, then sends the binding
 * pulses.
 *
 * The number of pulses dictates the mode the satellite receiver will request
 * the transmitter to use. The transmitter may bind but use a different mode. I
 * suggest configuring your radio to use DSMX 11ms fast mode if it allows that.
 *
 * 2048 & 1024 indicates 10 or 11 bit resolution. 11ms & 22ms indicates the time
 * period between the transmitter sending frames. 11ms is required for
 * transmitters with 8 or more channels.
 *
 * Testing done with DX7s, DX6i, DX8, and Orange T-SIX
 *
 * Table of Bind Modes
 *
 * | pulses |      mode       |
 * |:------:|:---------------:|
 * |   3    |  DSM  1024/22ms |
 * |   5    |  DSM  2048/11ms |
 * |   7    |  DSMX 1024/22ms |
 * |   9    |  DSMX 2048/11ms |
 *
 * This is a bit of a finicky process and may require a few attempts to work.
 *
 * @return     0 on success, -1 on failure
 */
int rc_dsm_bind_routine(void);


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
int rc_dsm_calibrate_routine(void);


#ifdef __cplusplus
}
#endif

#endif // RC_DSM_H

/** @} end group DSM */