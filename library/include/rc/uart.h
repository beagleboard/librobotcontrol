/**
 * <rc/uart.h>
 *
 * @brief      C interface for the Linux UART driver
 *
 * This is a general-purpose C interface to the linux UART driver device
 * (/dev/ttyO*). This is developed and tested on the BeagleBone platform but
 * should work on other linux systems too.
 *
 * @author     James Strawson
 * @date       3/6/2018
 *
 * @addtogroup UART
 * @ingroup    IO
 * @{
 */

#ifndef RC_UART_H
#define RC_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief      Initializes a UART bus /dev/ttyO{bus} at specified baudrate and
 * timeout.
 *
 * This is a very generalized function that configures the bus for 8-bit
 * characters and ignores the modem status lines.
 *
 * If you need a configuration other than whats presented here then you are
 * probably doing something fancy with the bus and you will probably want to do
 * your own reading/writing with standard linux methods.
 *
 * @param[in]  bus           The bus number /dev/ttyO{bus}
 * @param[in]  baudrate      must be one of the standard speeds in the UART
 * spec. 115200 and 57600 are most common.
 * @param[in]  timeout       timeout is in seconds and must be >=0.1
 * @param[in]  canonical_en  0 for non-canonical mode (raw data), non-zero for
 * canonical mode where only one line ending in '\n' is read at a time.
 * @param[in]  stop_bits     number of stop bits, 1 or 2, usually 1 for most
 * sensors
 * @param[in]  parity_en     0 to disable parity, nonzero to enable. usually
 * disabled for most sensors.
 *
 * @return     0 on success, -1 on failure
 */
int rc_uart_init(int bus, int baudrate, float timeout, int canonical_en, int stop_bits, int parity_en);


/**
 * @brief      closes a UART bus
 *
 * @param[in]  bus   The bus number /dev/ttyO{bus}
 *
 * @return     returns 0 on success, -1 on error.
 */
int rc_uart_close(int bus);

/**
 * @brief      Fetches the file descriptor to a uart bus after the bus has been
 * initialized.
 *
 * This is so the user can optionally do additional configuration or IO
 * operations on the bus.
 *
 * @param[in]  bus   The bus number /dev/ttyO{bus}
 *
 * @return     the file descriptor to /dev/ttyO{bus} or -1 on error
 */
int rc_uart_get_fd(int bus);

/**
 * @brief      flushes (discards) any data received but not read, or data
 * written but not sent.
 *
 * @param[in]  bus   The bus number /dev/ttyO{bus}
 *
 * @return     0 on success or -1 on failure
 */
int rc_uart_flush(int bus);

/**
 * @brief      Sends data out the uart port
 *
 * @param[in]  bus    The bus number /dev/ttyO{bus}
 * @param[in]  data   data pointer
 * @param[in]  bytes  number of bytes to send
 *
 * @return     returns number of bytes sent or -1 on error
 */
int rc_uart_write(int bus, uint8_t* data, size_t bytes);

/**
 * @brief      reads bytes from the UART bus
 *
 * This is a blocking function call. It will only return once the desired number
 * of bytes has been read from the buffer or the shutdown flag is set. due to
 * the Sitara's UART FIFO buffer, MAX_READ_LEN (128bytes) is the largest packet
 * that can be read with a single call to read(). For reads larger than
 * 128bytes, we run a loop instead.
 *
 * @param[in]  bus    The bus number /dev/ttyO{bus}
 * @param[out] buf    data pointer
 * @param[in]  bytes  number of bytes to read
 *
 * @return     Returns number of bytes actually read or -1 on error.
 */
int rc_uart_read_bytes(int bus, uint8_t* buf, size_t bytes);

/**
 * @brief      reads a line of characters ending in newline '\n'
 *
 * This is a blocking function call. It will only return on these conditions:
 *
 * - a newline character was read, this is discarded.
 * - max_bytes were read, this prevents overflowing a user buffer.
 * - timeout declared in rc_uart_init() is reached
 * - shutdown flag is set by rc_uart_close
 *
 * @param[in]  bus        The bus number /dev/ttyO{bus}
 * @param[out] buf        data pointer
 * @param[in]  max_bytes  max bytes to read in case newline character not found
 *
 * @return     Returns number of bytes actually read or -1 on error.
 */
int rc_uart_read_line(int bus, uint8_t* buf, size_t max_bytes);

/**
 * @brief      Fetches the number of bytes ready to be read from a bus
 *
 * @param[in]  bus   The bus
 *
 * @return     Returns number of bytes ready to be read or -1 on error.
 */
int rc_uart_bytes_available(int bus);


#ifdef __cplusplus
}
#endif

#endif // RC_UART_H

///@} end group IO