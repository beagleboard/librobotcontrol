/**
 * <rc/gpio.h>
 *
 * @brief      C interface for the Linux GPIO driver
 *
 * Developed and tested on the BeagleBone Black but should work fine on any
 * Linux system with the new character-device gpio driver in kernel 4.8 and
 * newer
 *
 * @author     James Strawson
 * @date       1/19/2018
 *
 * @addtogroup GPIO
 * @ingroup    IO
 * @{
 */

#ifndef RC_GPIO_H
#define RC_GPIO_H

#ifdef  __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifndef _GPIO_H_
#define GPIOHANDLE_REQUEST_INPUT	(1UL << 0)
#define GPIOHANDLE_REQUEST_OUTPUT	(1UL << 1)
#define GPIOHANDLE_REQUEST_ACTIVE_LOW	(1UL << 2)
#define GPIOHANDLE_REQUEST_OPEN_DRAIN	(1UL << 3)
#define GPIOHANDLE_REQUEST_OPEN_SOURCE	(1UL << 4)
#endif


/**
 * @brief      Configures a gpio pin as input or output
 *
 * This configures the pin by making a gpio handle request to the character
 * device driver. It accepts the same gpio handle request flags as defined in
 * <linux/gpio.h>
 *
 * - GPIOHANDLE_REQUEST_INPUT
 * - GPIOHANDLE_REQUEST_OUTPUT
 * - GPIOHANDLE_REQUEST_ACTIVE_LOW
 * - GPIOHANDLE_REQUEST_OPEN_DRAIN
 * - GPIOHANDLE_REQUEST_OPEN_SOURCE
 *
 * Obviously the INPUT and OUTPUT flags cannot be used at the same time. If you
 * don't know what the other flags mean just stick with INPUT and OUTPUT modes,
 * that covers 99% of use cases.
 *
 * @param[in]  chip          The chip number, /dev/gpiochipX
 * @param[in]  pin           The pin ID
 * @param[in]  handle_flags  The handle flags
 *
 * @return     0 on success or -1 on failure.
 */
int rc_gpio_init(int chip, int pin, int handle_flags);


/**
 * @brief      Sets the value of a GPIO pin when in output mode
 *
 * must call rc_gpio_init with the OUTPUT flag first.
 *
 * @param[in]  chip   The chip number, /dev/gpiochipX
 * @param[in]  pin    The pin ID
 * @param[in]  value  0 for off (inactive), nonzero for on (active)
 *
 * @return     0 on success or -1 on failure
 */
int rc_gpio_set_value(int chip, int pin, int value);


/**
 * @brief      Reads the value of a GPIO pin when in input mode or output mode.
 *
 * Must call rc_gpio_init first.
 *
 * @param[in]  chip  The chip number, /dev/gpiochipX
 * @param[in]  pin   The pin ID
 *
 * @return     1 if pin is high, 0 if pin is low, -1 on error
 */
int rc_gpio_get_value(int chip, int pin);


/** possible edge request **/
#ifndef _GPIO_H_
#define GPIOEVENT_REQUEST_RISING_EDGE	(1UL << 0)
#define GPIOEVENT_REQUEST_FALLING_EDGE	(1UL << 1)
#define GPIOEVENT_REQUEST_BOTH_EDGES	((1UL << 0) | (1UL << 1))
#endif

/**
 * @brief      Initializes a pin for interrupt event polling and normal reading.
 *
 * Handle flags exists if the user wishes to configure the pic as active-low,
 * open-source, or open-drain. This is usually not necessary and can be left at
 * 0. This function returns the file descriptor used for polling in case the
 * user wants to use a polling method other than rc_gpio_poll.
 *
 * @param[in]  chip          The chip number, /dev/gpiochipX
 * @param[in]  pin           The pin ID
 * @param[in]  handle_flags  Additional pin configuration flags, this can
 * usually be left as 0
 * @param[in]  event_flags   The event flags, GPIOEVENT_REQUEST_RISING_EDGE,
 * GPIOEVENT_REQUEST_FALLING_EDGE, or GPIOEVENT_REQUEST_BOTH_EDGES
 *
 * @return     File descriptor for the GPIO event or -1 on failure
 */
int rc_gpio_init_event(int chip, int pin, int handle_flags, int event_flags);

/** possible return values for rc_gpio_poll **/
#define RC_GPIOEVENT_ERROR		-1
#define RC_GPIOEVENT_TIMEOUT		0
#define RC_GPIOEVENT_RISING_EDGE	1
#define RC_GPIOEVENT_FALLING_EDGE	2

/**
 * @brief      polls a pin when configured for interrupt event polling
 *
 * This polls for an event and then reads one event from the queue.
 *
 * @param[in]  chip           The chip number, /dev/gpiochipX
 * @param[in]  pin            The pin ID
 * @param[in]  timeout_ms     The timeout in milliseconds. Negative value causes
 * infinite timeout, a value of 0 makes the function return immediately after
 * reading an event in the queue.
 * @param[out] event_time_ns  pointer where the time of the gpio event occured.
 * Units are nanoseconds since epoch. Set this as NULL if you don't want to keep
 * the time.
 *
 * @return     returns RC_GPIO_EVENT_ERROR, RC_GPIO_EVENT_TIMEOUT,
 * RC_GPIO_EVENT_RISING_EDGE, or RC_GPIO_EVENT_FALLING_EDGE to indicate what
 * happened.
 */
int rc_gpio_poll(int chip, int pin, int timeout_ms, uint64_t* event_time_ns);


/**
 * @brief      closes the file descriptor for a pin
 *
 * Not strictly necessary to run at the end of your program since linux will
 * clean this up for you. However this is sometimes useful in the middle of a
 * program when a pin is no longer needed.
 *
 * @param[in]  chip  The chip number, /dev/gpiochipX
 * @param[in]  pin   The pin ID
 */
void rc_gpio_cleanup(int chip, int pin);




#ifdef __cplusplus
}
#endif

#endif // RC_GPIO_H

///@} end group GPIO