/**
 * <rc/encoder_pru.h>
 *
 * @brief      Functions for reading the PRU-accelerated quadrature encoder
 * counter.
 *
 * This can be used for reading encoder channel 4 on the Robotics Cape and
 * BeagleBone Blue. Channels 1-3 are instead counted with the eQEP hardware
 * encoder counters, see <rc/encoder_eqep.h> to use channels 1-3.
 *
 *
 * @author     James Strawson
 * @date       1/31/2018
 *
 * @addtogroup Encoder_PRU
 * @ingroup    Quadrature_Encoder
 * @{
 */

#ifndef RC_ENCODER_PRU_H
#define RC_ENCODER_PRU_H

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief      Initializes the pru encoder counter for channel 4
 *
 * This also resets the encoder position to 0 so the first position read is
 * consistent. Note this does NOT initialize the eqep-accelerated encoder
 * counters on channels 1-3. To use channels 1-3 you must use
 * <rc/encoder_eqep.h>
 *
 * @return     0 on success or -1 on failure
 */
int rc_encoder_pru_init(void);

/**
 * @brief      Stops the PRU encoder counter and closes file descriptors. This
 * is not strictly necessary but is recommended that the user calls this
 * function at the end of their program.
 */
void rc_encoder_pru_cleanup(void);

/**
 * @brief      Reads the current position of encoder channel 4.
 *
 * This is a signed 32-bit integer that wraps around if the position is allowed
 * to read +- 2^31
 *
 * @return     The current position (signed 32-bit integer) or -1 and prints an
 * error message is there is a problem.
 */
int rc_encoder_pru_read(void);

/**
 * @brief      Sets the current position of encoder channel 4. Usually for
 * resetting a counter to 0 but can set an arbitrary position if desired.
 *
 * @param[in]  pos   The new position
 *
 * @return     0 on success, -1 on failure
 */
int rc_encoder_pru_write(int pos);


#ifdef __cplusplus
}
#endif

#endif // RC_ENCODER_PRU_H

/** @} end group Encoder_PRU*/