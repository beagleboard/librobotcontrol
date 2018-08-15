/**
 * <rc/encoder_eqep.h>
 *
 * @brief      C interface for the Sitara eQEP encoder counter.
 *
 * Functions for reading the eQEP hardware-accelerated quadrature encoder
 * counters. This can be used for reading encoder channels 1-3 on the Robotics
 * Cape and BeagleBone Blue. Channel 4 is counted with the PRU, see
 * <rc/encoder_pru.h> to use the 4th channel.
 *
 *
 * @author     James Strawson
 * @date       1/31/2018
 *
 * @addtogroup Encoder_EQEP
 * @ingroup    Quadrature_Encoder
 * @{
 */

#ifndef RC_ENCODER_EQEP_H
#define RC_ENCODER_EQEP_H

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief      Initializes the eQEP encoder counters for channels 1-3
 *
 * This also resets the encoder position to 0 so the first position read is
 * consistent. Note this does NOT initialize the PRU-accelerated encoder counter
 * on channel 4. To use the 4th channel you must use <rc/encoder_pru.h>
 *
 * @return     0 on success or -1 on failure
 */
int rc_encoder_eqep_init(void);

/**
 * @brief      Stops the eQEP encoder counters and closes file descriptors. This
 * is not strictly necessary but is recommended that the user calls this
 * function at the end of their program.
 *
 * @return     0 on success or -1 on failure.
 */
int rc_encoder_eqep_cleanup(void);

/**
 * @brief      Reads the current position of an encoder channel.
 *
 * This is a signed 32-bit integer that wraps around if the position is allowed
 * to read +- 2^31
 *
 * @param[in]  ch    channel 1-3
 *
 * @return     The current position (signed 32-bit integer) or -1 and prints an
 * error message is there is a problem.
 */
int rc_encoder_eqep_read(int ch);

/**
 * @brief      Sets the current position of an eQEP encoder channel. Usually for
 * resetting a counter to 0 but can set an arbitrary position if desired.
 *
 * @param[in]  ch    channel 1-3
 * @param[in]  pos   The new position
 *
 * @return     0 on success, -1 on failure
 */
int rc_encoder_eqep_write(int ch, int pos);


#ifdef __cplusplus
}
#endif

#endif // RC_ENCODER_EQEP_H

/** @}  end group Encoder_EQEP*/