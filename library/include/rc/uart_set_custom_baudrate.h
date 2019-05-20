/**
 * <rc/uart_set_custom_baudrate.h>
 *
 * @brief      C interface for the Linux UART driver to set custom baudrates
 *
 * This is a general-purpose C interface to the linux UART driver device
 * (/dev/ttyO*). This is developed and tested on the BeagleBone platform but
 * should work on other linux systems too.
 *
 * @author     Per Dalgas Jakobsen
 * @date       3/4/2019
 *
 * @addtogroup UART
 * @ingroup    IO
 * @{
 */

#ifndef RC_UART_SET_CUSTOM_BAUDRATE_H
#define RC_UART_SET_CUSTOM_BAUDRATE_H

#ifdef __cplusplus
extern "C" {
#endif

int rc_uart_set_custom_baudrate (int fd, int baudrate);

#ifdef __cplusplus
}
#endif

#endif // RC_UART_SET_CUSTOM_BAUDRATE_H

///@} end group IO
