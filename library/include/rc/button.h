/**
 * <rc/button.h>
 *
 * @brief      Handle generic GPIO buttons.
 *
 *             Functions for assigning button callback functions. This is based
 *             on the GPIO character device driver instead of the gpio-keys
 *             driver which means it can be used with any GPIO pin.
 *
 * @author     James Strawson
 * @date       3/7/2018
 *
 * @addtogroup Button
 * @{
 */


#ifndef RC_BUTTON_H
#define RC_BUTTON_H

#ifdef __cplusplus
extern "C" {
#endif


#define RC_BTN_PIN_PAUSE		69	//gpio2.5 P8.9
#define RC_BTN_PIN_MODE			68	//gpio2.4 P8.10

#define RC_BTN_STATE_PRESSED		1
#define RC_BTN_STATE_RELEASED		0

#define RC_BTN_POLARITY_NORM_HIGH	1
#define RC_BTN_POLARITY_NORM_LOW	0

#define RC_BTN_DEBOUNCE_DEFAULT_US	2000

/**
 * @brief      Initializes a single button handler.
 *
 * @param[in]  pin          The gpio pin
 * @param[in]  polarity     RC_BTN_POLARITY_NORM_HIGH if using with a pullup
 *                          resistor, use this for the BeagleBone Blue and
 *                          Robotics Cape MODE and PAUSE buttons. Alternatively
 *                          use RC_BTN_POLARITY_NORM_LOW if you are using your
 *                          own button on another pin set up with a pulldown
 *                          resistor.
 * @param[in]  debounce_us  debounce interval in microseconds. Set to 0 for no
 *                          debounce. Usually should set to
 *                          RC_BTN_DEBOUNCE_DEFAULT_US.
 *
 * @return     0 on success, -1 on failure
 */
int rc_button_init(int pin, char polarity, int debounce_us);


/**
 * @brief      Closes all button handlers. Call at the end of your program
 *             before returning.
 */
void rc_button_cleanup();


/**
 * @brief      Sets the callback functions to be called when the button is
 *             pressed or released.
 *
 *             These functions should be short and return quickly. On every
 *             press and release a new thread is created to run your callback
 *             functions. If your callbacks take too long to return then
 *             multiple instances of them will run in parallel which may or may
 *             not be desirable.
 *
 * @param[in]  pin           The gpio pin
 * @param[in]  press_func    callback when button is pressed, set to NULL if no callback is desired.
 * @param[in]  release_func  callback when button is released, set to NULL if no callback is desired.
 *
 * @return     0 on success, -1 on failure.
 */
int rc_button_set_callbacks(int pin, void (*press_func)(void), void (*release_func)(void));


/**
 * @brief      used to query the position of a button.
 *
 * @param[in]  pin   The gpio pin
 *
 * @return     RC_BTN_STATE_PRESSED or RC_BTN_STATE_RELEASED on success, -1 on
 *             failure.
 */
int rc_button_get_state(int pin);


#ifdef __cplusplus
}
#endif

#endif // RC_BUTTON_H

/** @}  end group Button */