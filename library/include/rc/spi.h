/**
 * <rc/spi.h>
 *
 * @brief      General purpose C interface to the Linux SPI driver.
 *
 * It allows use of both dedicated hardware slave-select pins as well as GPIO
 * pins for slave select. While this was developed on the BeagleBone platform,
 * it should also work on Raspberry Pi and other embedded Linux systems.
 *
 * For the Robotics Cape and BeagleBone blue, the SPI bus 1 is broken out on two
 * JST SH 6-pin sockets labeled SPI1.1 and SPI1.2 These share clock and serial
 * IO signals, but have independent slave select lines so two devices can share
 * the same bus. Note that these ports labeled for slaves 1 and 2 correspond to
 * slaves 0 and 1 in software. To make source code more clear, the macros
 * RC_BB_SPI1_SS1 and RC_BB_SPI1_SS2 are provided in this header which are
 * defined as 1,0 and 1,1. the GPIO channels corresponding to these slave select
 * pins are also provided as macros in this header.For example:
 *
 * ```C
 * rc_spi_init_manual_slave(RC_BB_SPI1_SS1, SPI_MODE_0, \
 *                          RC_SPI_MAX_SPEED, RC_BLUE_SS1_GPIO);
 * ```
 *
 * pinout on Robotics Cape and BeagleBone Blue:
 * 1. GND
 * 2. 3.3V
 * 3. MOSI (P9_30)
 * 4. MISO (P9_29)
 * 5. SCK  (P9_31)
 * 6. Slave Select
 *
 *
 * The slaves can be selected automatically by the SPI Linux driver or manually
 * with rc_spi_select() function. On the Robotics Cape, slave 1 can be used in
 * either mode, but slave 2 must be selected manually. On the BB Blue either
 * slave can be used in manual or automatic modes. Only initialize once with
 * either mode.
 *
 * @author     James Strawson
 * @date       1/19/2018
 *
 * @addtogroup SPI
 * @ingroup    IO
 * @{
 */


#ifndef RC_SPI_H
#define RC_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <linux/spi/spidev.h> // for xfer and ioctl calls


#define RC_SPI_MAX_SPEED	24000000	///< 24mhz
#define RC_SPI_MIN_SPEED	1000		///< 1khz
#define RC_SPI_BITS_PER_WORD	8		///< only allow for 8-bit words


#define RC_BB_SPI1_SS1		1,0 ///< bus and slave to use for Robotics Cape and BeagleBone Blue SPI1.1 port
#define RC_BB_SPI1_SS2		1,1 ///< bus and slave to use for Robotics Cape and BeagleBone Blue SPI1.2 port

#define RC_CAPE_SS1_GPIO	3,17 ///< Robotics Cape SPI1 SS1 gpio P9_28, normally AUTO mode
#define RC_CAPE_SS2_GPIO	1,17 ///< Robotics Cape SPI1 SS2 gpio P9_23, normally MANUAL GPIO mode

#define RC_BLUE_SS1_GPIO	0,29 ///< BeagleBone Blue SPI1 SS1 gpio 0_29 pin H18
#define RC_BLUE_SS2_GPIO	0,7  ///< BeagleBone Blue SPI1 SS2 gpio 0_7 pin H18


/**
 * @brief      Initializes an SPI bus
 *
 * For description of the 4 SPI modes, see
 * <https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus#Mode_numbers>
 *
 * @param[in]  bus       The bus
 * @param[in]  slave     The slave
 * @param[in]  bus_mode  SPI_MODE_0, SPI_MODE_1, SPI_MODE_2, or SPI_MODE_3
 * @param[in]  speed_hz  The speed hz
 *
 * @return     0 on succcess or -1 on failure
 */
int rc_spi_init_auto_slave(int bus, int slave, int bus_mode, int speed_hz);


/**
 * @brief      Initializes an SPI bus and GPIO pin for use as a manual SPI slave
 * select pin.
 *
 * The provided gpio chip/pin will then be remembered and tied to the provided
 * slave number and bus. Note that on the BeagleBone and probably other
 * platforms, there are only two files provided by the driver for interfacing to
 * the bus, /dev/spi1.0 and /dev/spi1.1. When using a slave in manual mode, the
 * first interface (slave 0 in software) will be used to talk to the manual
 * slaves. Therefore this slave can not be used as an automatic slave.
 *
 * For the BeagleBone Blue and RoboticsCape, this will also ensure that the
 * pinmux is set correctly for that pin. The available manual slave select pins
 * for these two boards are defined in this header for convenience. If using
 * other boards it's up to the user to make sure the pin they are using is set
 * up correctly in the device tree.
 *
 * @param[in]  bus       The spi bus
 * @param[in]  slave     The slave identifier (up to 16)
 * @param[in]  bus_mode  The bus mode
 * @param[in]  speed_hz  The speed hz
 * @param[in]  chip      The gpio chip
 * @param[in]  pin       The gpio pin
 *
 * @return     0 on succcess or -1 on failure
 */
int rc_spi_init_manual_slave(int bus, int slave, int bus_mode, int speed_hz, int chip, int pin);


/**
 * @brief      fetches the file descriptor for a specified slave so the user can
 * do more advanced IO operations than what's presented here
 *
 * @param[in]  bus    The bus
 * @param[in]  slave  0 or 1
 *
 * @return     fd or -1 on failure
 */
int rc_spi_get_fd(int bus, int slave);


/**
 * @brief      Closes and cleans up the bus for specified slave
 *
 * @param[in]  bus   SPI bus to close
 *
 * @return     0 on succcess or -1 on failure
 */
int rc_spi_close(int bus);


/**
 * @brief      Manually selects or deselects a slave
 *
 * Only works if slave was initialized with SPI_SLAVE_MODE_MANUAL. If
 * SPI_SLAVE_MODE_AUTO was selected then the SPI driver will handle this
 * automatically when reading or writing.
 *
 * @param[in]  bus     SPI bus to use
 * @param[in]  slave   slave id
 * @param[in]  select  0 to deselect, otherwise selects
 *
 * @return     0 on succcess or -1 on failure
 */
int rc_spi_manual_select(int bus, int slave, int select);


/**
 * @brief      Send any sequence of bytes and read the response.
 *
 * This is a wrapper for the ioctl spi transfer function and is generally what
 * you will use for reading/writing device registers.
 *
 * @param[in]  bus       SPI bus to use
 * @param[in]  slave     slave id
 * @param[in]  tx_data   pointer to data to send
 * @param[in]  tx_bytes  number of bytes to send
 * @param      rx_data   pointer to put response data
 *
 * @return     number of bytes received or -1 on failure
 */
int rc_spi_transfer(int bus, int slave, uint8_t* tx_data, size_t tx_bytes, uint8_t* rx_data);


/**
 * @brief      Writes data to specified slave
 *
 * @param[in]  bus    SPI bus to use
 * @param[in]  slave  slave id
 * @param      data   data pointer
 * @param[in]  bytes  number of bytes to send
 *
 * @return     returns number of bytes written or -1 on failure
 */
int rc_spi_write(int bus, int slave, uint8_t* data, size_t bytes);


/**
 * @brief      Reads data from a specified slave
 *
 * @param[in]  bus    SPI bus to use
 * @param[in]  slave  slave id
 * @param      data   data poitner
 * @param[in]  bytes  number of bytes to read
 *
 * @return     number of bytes read or -1 on failure
 */
int rc_spi_read(int bus, int slave, uint8_t* data, size_t bytes);


#ifdef __cplusplus
}
#endif

#endif // RC_SPI_H

///@} end group SPI

