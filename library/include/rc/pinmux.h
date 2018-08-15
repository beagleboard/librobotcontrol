/**
 * <rc/pinmux.h>
 *
 * @brief      C interface for the Sitara pinmux helper driver
 *
 * On the Robotics Cape, we allow changing the pinmux on the SPI, GPS, and UART1
 * headers in case you wish to expose GPIO, CAN, or PWM functionality. We use
 * the GPIO number to identify the pins even though they may be used for things
 * other than GPIO as this provides consistency with the GPIO functions which
 * will likely be used. A list of defines are also given here to make your code
 * easier to read and to indicate which pins are available for pinmuxing.
 *
 * Not all pinmux modes are available on each pin. However, every pin can be
 * configured as a GPIO output, or input with either an internal pullup (PU) or
 * pulldown (PD) resistor.
 *
 * The GPS header pins 3 and 4 can be configured to PWM mode which breaks out
 * channels A and B of PWM subsystem 0 which are not used by the motor drivers
 * and so are free for the user to do with as they please. They default to UART
 * mode for communicating with GPS receivers and can also be used in any GPIO
 * mode.
 *
 * The UART1 header pins 3 and 4 also default to UART mode and can be used in
 * any GPIO mode. However, they also break out the CAN bus RX and TX lines.
 * However, to use CAN bus you also need to set up a CAN-PHY IC yourself.
 *
 * All SPI pins can be used for SPI or GPIO. If you intend to use these pins for
 * pure GPIO use then use the set_pinmux_mode() function described here. Note
 * that when configuring the SPI slave select lines for manual or automatic mode
 * as described in the SPI section of this manual, the rc_spi_init function uses
 * this pinmux mode in the backend to set up the pin for you.
 *
 * The beaglebone Blue additionally has 2 GPIO headers, GP0 & GP1, with 3.3v,
 * ground, and 4 GPIO signals broken out on each. All 4 signal pins on GP0 can
 * be configured as one of the 3 different GPIO modes, and so can pins 3 and 4
 * of GP1. Pins 5 and 6 of the GP1 header are fixed in output mode and are tied
 * to the Red and Green LED signals in case those signals wish to be extended to
 * lights outside of a robot's case.
 *
 *
 * @addtogroup Pinmux
 * @ingroup    IO
 * @{
 */

#ifndef RC_PINMUX_H
#define RC_PINMUX_H

#ifdef __cplusplus
extern "C" {
#endif


/** @name Configurable pins shared between Robotics Cape and BeagleBone Blue */
///@{
#define DSM_HEADER_PIN		30	///< P9.11, normally DSM UART4
#define GPS_HEADER_PIN_3	2	///< P9_22, normally GPS UART2 RX
#define GPS_HEADER_PIN_4	3	///< P9_21, normally GPS UART2 TX
#define UART1_HEADER_PIN_3	14	///< P9_26, normally UART1 RX
#define UART1_HEADER_PIN_4	15	///< P9_24, normally UART1 TX
#define SPI_HEADER_PIN_3	112	///< P9_30, normally SPI1 MOSI
#define SPI_HEADER_PIN_4	111	///< P9_29, normally SPI1 MISO
#define SPI_HEADER_PIN_5	110	///< P9_31, normally SPI1 SCLK
///@}

/** @name Configurable pins for Robotics Cape only */
///@{
#define CAPE_SPI_PIN_6_SS1	113	///< P9_28, normally SPI mode
#define CAPE_SPI_PIN_6_SS2	49	///< P9_23, normally GPIO mode
///@}


/** @name Configurable pins for BeagleBone Blue only */
///@{
#define BLUE_SPI_PIN_6_SS1	29	///< gpio 0_29 pin H18
#define BLUE_SPI_PIN_6_SS2	7	///< gpio 0_7  pin C18
#define BLUE_GP0_PIN_3		57	///< gpio 1_25 pin U16
#define BLUE_GP0_PIN_4		49	///< gpio 1_17 pin P9.23
#define BLUE_GP0_PIN_5		116	///< gpio 3_20 pin D13
#define BLUE_GP0_PIN_6		113	///< gpio 3_17 pin P9_28
#define BLUE_GP1_PIN_3		98	///< gpio 3_2  pin J15
#define BLUE_GP1_PIN_4		97	///< gpio 3_1  pin H17
///@}


/**
 * Gives options for pinmuxing. Not every mode if available on each pin. Refer
 * to the official BeagleBone pin table for which to use.
 */
typedef enum rc_pinmux_mode_t{
	PINMUX_GPIO,
	PINMUX_GPIO_PU,
	PINMUX_GPIO_PD,
	PINMUX_PWM,
	PINMUX_SPI,
	PINMUX_UART,
	PINMUX_CAN
} rc_pinmux_mode_t;

/**
 * @brief      sets once of the pins defined in this header to a particular mode
 *
 * @param[in]  pin   The pin
 * @param[in]  mode  The mode
 *
 * @return     0 on success, -1 on failure
 */
int rc_pinmux_set(int pin, rc_pinmux_mode_t mode);

/**
 * @brief      puts everything back to standard
 *
 * @return     0 on success, -1 on failure
 */
int rc_pinmux_set_default(void);


#ifdef __cplusplus
}
#endif

#endif // RC_PINMUX_H

/** @} end group Pinmux */