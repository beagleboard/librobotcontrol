/**
 * <rc/encoder.h>
 *
 * @brief      C interface for quadrature encoder counting.
 *
 * Functions for reading the eQEP hardware-accelerated quadrature encoder
 * counters and the PRU accelerated encoder counter.Channels 1-3 on the Robotics
 * Cape and BeagleBone Blue are counted using the Sitara's eQEP counter. Channel
 * 4 is counted with the PRU.
 *
 *
 * @author     James Strawson
 * @date       5/12/2018
 *
 * @addtogroup Encoder
 * @ingroup    Quadrature_Encoder
 * @{
 */

#ifndef RC_ENCODER_H
#define RC_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief      Initializes counters for channels 1-4
 *
 * This also resets the encoder position to 0 so the first position read is
 * consistent.
 *
 * @return     0 on success or -1 on failure
 */
int rc_encoder_init(void);

/**
 * @brief      Stops the encoder counters and closes file descriptors. This is
 * not strictly necessary but is recommended that the user calls this function
 * at the end of their program.
 *
 * @return     0 on success or -1 on failure.
 */
int rc_encoder_cleanup(void);

/**
 * @brief      Reads the current position of an encoder channel.
 *
 * This is a signed 32-bit integer that wraps around if the position is allowed
 * to read +- 2^31
 *
 * @param[in]  ch    channel 1-4
 *
 * @return     The current position (signed 32-bit integer) or -1 and prints an
 * error message is there is a problem.
 */
int rc_encoder_read(int ch);

/**
 * @brief      Sets the current position of an eQEP encoder channel. Usually for
 * resetting a counter to 0 but can set an arbitrary position if desired.
 *
 * @param[in]  ch    channel 1-4
 * @param[in]  pos   The new position
 *
 * @return     0 on success, -1 on failure
 */
int rc_encoder_write(int ch, int pos);


#ifdef __cplusplus
}
#endif

#endif // RC_ENCODER_H

/** @}  end group Encoder*/