/**
 * <rc/model.h>
 *
 * @brief      Determine the model of board currently being used.
 *
 * The user may sometimes need to determine which board they are running on to
 * decide which pins or modules to use. These functions are also used internally
 * to the library for hardware specific configuration. See the rc_model example
 * for a demonstration.
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


/**
 * List of models detectable by the rc_model() function. This is not a complete
 * list of board this library will run on. Nor is it a list of boards that the
 * library is guaranteed to work on. Currently, MODEL_PC indicates personal
 * computer and will be selected whenever running on an x86 or x86_64
 * architecture machine until more specific boards are added to this list by
 * user request.
 */
typedef enum rc_model_t{
	MODEL_UNKNOWN,
	MODEL_BB_BLACK,
	MODEL_BB_BLACK_RC,
	MODEL_BB_BLACK_W,
	MODEL_BB_BLACK_W_RC,
	MODEL_BB_GREEN,
	MODEL_BB_GREEN_W,
	MODEL_BB_BLUE,
	MODEL_BB_POCKET,
	MODEL_RPI_B,
	MODEL_RPI_B_PLUS,
	MODEL_RPI2_B,
	MODEL_RPI3_B,
	MODEL_RPI3_B_PLUS,
	MODEL_RPI0,
	MODEL_RPI0_W,
	MODEL_RPI_CM,
	MODEL_RPI_CM3,
	MODEL_PC
} rc_model_t;


/**
 * This is a list of general categories of boards.
 */
typedef enum rc_model_category_t{
	CATEGORY_UNKNOWN,
	CATEGORY_BEAGLEBONE,
	CATEGORY_RPI,
	CATEGORY_PC
} rc_model_category_t;


/**
 * @brief      gets the current board model name
 *
 * @return     rc_model_t enum representation of model
 */
rc_model_t rc_model(void);


/**
 * @brief      gets the general category for the current board
 *
 * @return     rc_model_category_t enum representation of categoy
 */
rc_model_category_t rc_model_category(void);


/**
 * @brief      prints to the screen the human-readable version of the model name
 * with no trailing newline character.
 */
void rc_model_print(void);


/**
 * @brief      prints to the screen the human-readable version of the category
 * name with no trailing newline character.
 */
void rc_model_category_print(void);


#ifdef __cplusplus
}
#endif

#endif // RC_MODEL_H

/** @} end group Model*/