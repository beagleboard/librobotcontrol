/**
 * <rc/spi.h>
 *
 * @brief      C interface for the Linux SPI driver
 *
 *             The Sitara's SPI bus is broken out on two JST SH 6-pin sockets
 *             labeled SPI1.1 and SPI1.2 These share clock and serial IO
 *             signals, but have independent slave select lines.
 *
 *             The slaves can be selected automatically by the SPI linux driver
 *             or manually with rc_spi_select() function. On the Robotics Cape,
 *             slave 1 can be used in either mode, but slave 2 must be selected
 *             manually. On the BB Blue either slave can be used in manual or
 *             automatic modes.
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

#define SPI_SLAVE_MODE_AUTO	0
#define SPI_SLAVE_MODE_MANUAL	1

/**
 * @brief      Initializes a particular slave on SPI bus 1
 *
 *             see
 *             https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus#Mode_numbers
 *             for SPI modes
 *
 * @param[in]  slave       The slave 1 or 2
 * @param[in]  slave_mode  SPI_SLAVE_MODE_AUTO or SPI_SLAVE_MODE_MANUAL
 * @param[in]  bus_mode    SPI_MODE_0, SPI_MODE_1, SPI_MODE_2, or SPI_MODE_3
 * @param[in]  speed_hz    The speed hz
 *
 * @return     0 on succcess or -1 on failure
 */
int rc_spi_init(int slave, int slave_mode, int bus_mode, int speed_hz);

/**
 * @brief      fetches the file descriptor for a specified slave so the user can
 *             do more advanced IO operations than what's presented here
 *
 * @param[in]  slave slave 0 or 1
 *
 * @return     fd or -1 on failure
 */
int rc_spi_fd(int slave);

/**
 * @brief      Closes and cleans up the bus for specified slave
 *
 * @param[in]  slave  slave 0 or 1
 *
 * @return     0 on succcess or -1 on failure
 */
int rc_spi_close(int slave);

/**
 * @brief      Manually selects or deselects a slave
 *
 *             Only works if slave was initialized with SPI_SLAVE_MODE_MANUAL.
 *             If SPI_SLAVE_MODE_AUTO was selected then the SPI driver will
 *             handle this automatically when reading or writing.
 *
 * @param[in]  slave   slave 0 or 1
 * @param[in]  select  0 to deselect, otherwise selects
 *
 * @return     0 on succcess or -1 on failure
 */
int rc_spi_select(int slave, int select);

/**
 * @brief      Send any sequence of bytes and read the response.
 *
 *             This is a wrapper for the ioctl spi transfer function and is
 *             generally what you will use for reading/writing device registers.
 *
 * @param      tx_data   pointer to data to send
 * @param[in]  tx_bytes  number of bytes to send
 * @param      rx_data   pointer to put response data
 * @param[in]  slave     slave 1 or 2
 *
 * @return     number of bytes received or -1 on failure
 */
int rc_spi_transfer(int slave, uint8_t* tx_data, int tx_bytes, uint8_t* rx_data);

/**
 * @brief      Writes data to specified slave
 *
 * @param      data   data pointer
 * @param[in]  bytes  number of bytes to send
 * @param[in]  slave  slave 1 or 2
 *
 * @return     returns number of bytes written or -1 on failure
 */
int rc_spi_write(int slave, uint8_t* data, int bytes);

/**
 * @brief      Reads data from a specified slave
 *
 * @param      data   data poitner
 * @param[in]  bytes  number of bytes to read
 * @param[in]  slave  slave 1 or 2
 *
 * @return     number of bytes read or -1 on failure
 */
int rc_spi_read(int slave, uint8_t* data, int bytes);


#ifdef __cplusplus
}
#endif

#endif // RC_SPI_H

///@} end group SPI

