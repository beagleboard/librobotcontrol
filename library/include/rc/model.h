/**
 * <rc/model.h>
 *
 * @brief      Determine the model of BeagleBone or cape in use.
 *
 *             Because we wish to support different beagleboard products with
 *             this same library, we must internally determine which board we
 *             are running on to decide which pins to use. We make these
 *             functions available to the user in case they wish to do the same.
 *             See the rc_model example for a demonstration.
 *
 * @author     James Strawson
 * @date       1/31/2018
 *
 * @addtogroup Model
 * @{
 */


#ifndef RC_MODEL_H
#define RC_MODEL_H

#ifdef __cplusplus
extern "C" {
#endif


typedef enum rc_model_t{
	UNKNOWN_MODEL,
	BB_BLACK,
	BB_BLACK_RC,
	BB_BLACK_W,
	BB_BLACK_W_RC,
	BB_GREEN,
	BB_GREEN_W,
	BB_BLUE
} rc_model_t;

/**
 * @brief      gets the current board model name
 *
 * @return     rc_model_t enum representation of model
 */
rc_model_t rc_model();

/**
 * @brief      prints to the screen the human-readable version of the model name
 */
void rc_model_print();


#ifdef __cplusplus
}
#endif

#endif // RC_MODEL_H

/** @}  end group Model*/