/**
 * <rc/pinmux.h>
 *
 * @brief C interface for the Sitara pinmux helper driver
 *
 * On the Robotics Cape, we allow changing the pinmux on the SPI, GPS, and UART1
 * headers in case you wish to expose GPIO, CAN, or PWM functionality. We use
 * the GPIO number to identify the pins even though they may be used for things
 * other than GPIO as this provides consistency with the GPIO functions which
 * will likely be used. A list of defines are also given here to make your code
 * easier to read and to indicate which pins are available for pinmuxing.
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

// Cape and Blue
#define DSM_HEADER_PIN		30	// P9.11, normally DSM UART4
#define GPS_HEADER_PIN_3	2	// P9_22, normally GPS UART2 RX
#define GPS_HEADER_PIN_4	3	// P9_21, normally GPS UART2 TX
#define UART1_HEADER_PIN_3	14	// P9_26, normally UART1 RX
#define UART1_HEADER_PIN_4	15	// P9_24, normally UART1 TX
#define SPI_HEADER_PIN_3	112	// P9_30, normally SPI1 MOSI
#define SPI_HEADER_PIN_4	111	// P9_29, normally SPI1 MISO
#define SPI_HEADER_PIN_5	110	// P9_31, normally SPI1 SCLK

// Cape Only
#define CAPE_SPI_PIN_6_SS1	113	// P9_28, normally SPI mode
#define CAPE_SPI_PIN_6_SS2	49	// P9_23, normally GPIO mode

// Blue Only
#define BLUE_SPI_PIN_6_SS1	29	// gpio 0_29 pin H18
#define BLUE_SPI_PIN_6_SS2	7	// gpio 0_7  pin C18
#define BLUE_GP0_PIN_3		57	// gpio 1_25 pin U16
#define BLUE_GP0_PIN_4		49	// gpio 1_17 pin P9.23
#define BLUE_GP0_PIN_5		116	// gpio 3_20 pin D13
#define BLUE_GP0_PIN_6		113	// gpio 3_17 pin P9_28
#define BLUE_GP1_PIN_3		98	// gpio 3_2  pin J15
#define BLUE_GP1_PIN_4		97	// gpio 3_1  pin H17

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
int rc_pinmux_set_default();


#ifdef __cplusplus
}
#endif

#endif // RC_PINMUX_H

/** @}  end group Pinmux */