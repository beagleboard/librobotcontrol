/**
 * <rc/button.h>
 *
 * @brief      Handle generic GPIO buttons.
 *
 * Functions for assigning button callback functions. This is based on the GPIO
 * character device driver instead of the gpio-keys driver which means it can be
 * used with any GPIO pin.
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


#define RC_BTN_PIN_PAUSE		2,5	//gpio2.5 P8.9
#define RC_BTN_PIN_MODE			2,4	//gpio2.4 P8.10

#define RC_BTN_STATE_PRESSED		1
#define RC_BTN_STATE_RELEASED		0

#define RC_BTN_POLARITY_NORM_HIGH	1
#define RC_BTN_POLARITY_NORM_LOW	0

#define RC_BTN_DEBOUNCE_DEFAULT_US	2000

/**
 * @brief      Initializes a single button handler.
 *
 * @param[in]  chip         The gpio chip
 * @param[in]  pin          The gpio pin for that chip
 * @param[in]  polarity     RC_BTN_POLARITY_NORM_HIGH if using with a pullup
 * resistor, use this for the BeagleBone Blue and Robotics Cape MODE and PAUSE
 * buttons. Alternatively use RC_BTN_POLARITY_NORM_LOW if you are using your own
 * button on another pin set up with a pulldown resistor.
 * @param[in]  debounce_us  debounce interval in microseconds. Set to 0 for no
 * debounce. Usually should set to RC_BTN_DEBOUNCE_DEFAULT_US.
 *
 * @return     0 on success, -1 on failure
 */
int rc_button_init(int chip, int pin, char polarity, int debounce_us);


/**
 * @brief      Closes all button handlers. Call at the end of your program
 * before returning.
 */
void rc_button_cleanup(void);


/**
 * @brief      Sets the callback functions to be called when the button is
 * pressed or released.
 *
 * These functions should be short and return quickly. On every press and
 * release a new thread is created to run your callback functions. If your
 * callbacks take too long to return then multiple instances of them will run in
 * parallel which may or may not be desirable.
 *
 * @param[in]  chip          The gpio chip
 * @param[in]  pin           The gpio pin for that chip
 * @param[in]  press_func    callback when button is pressed, set to NULL if no
 * callback is desired.
 * @param[in]  release_func  callback when button is released, set to NULL if no
 * callback is desired.
 *
 * @return     0 on success, -1 on failure.
 */
int rc_button_set_callbacks(int chip, int pin, void (*press_func)(void), void (*release_func)(void));


/**
 * @brief      used to query the position of a button.
 *
 * @param[in]  chip  The gpio chip
 * @param[in]  pin   The gpio pin for that chip
 *
 * @return     RC_BTN_STATE_PRESSED or RC_BTN_STATE_RELEASED on success, -1 on
 * failure.
 */
int rc_button_get_state(int chip, int pin);


/**
 * @brief      blocking function call, returns when press or release happens
 *
 * @param[in]  chip              The gpio chip
 * @param[in]  pin               The gpio pin for that chip
 * @param[in]  press_or_release  RC_BTN_STATE_PRESSED or RC_BTN_STATE_RELEASED
 *
 * @return     0 on successful event, -1 on error
 */
int rc_button_wait_for_event(int chip, int pin, int press_or_release);


#ifdef __cplusplus
}
#endif

#endif // RC_BUTTON_H

/** @} end group Button */