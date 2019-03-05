/*
 * rc_uart_set_custom_baudrate is placed in separate compilation file
 * because definitions in asm/ioctls.h and/or asm/termbits.h conflicts
 * with definitions in sys/ioctl.h
 */

#include <sys/ioctl.h>
#include <asm/termbits.h>
#include <asm/ioctls.h>

#include <rc/uart_set_custom_baudrate.h>

int rc_uart_set_custom_baudrate (int fd, int baudrate)
{
	struct termios2 ntio;
	int res;

	res = ioctl(fd, TCGETS2, &ntio);
	if (res != 0) return res;

	ntio.c_cflag &= ~CBAUD;
	ntio.c_cflag |= BOTHER | CREAD;
	ntio.c_ispeed = baudrate;
	ntio.c_ospeed = baudrate;

	res = ioctl(fd, TCSETS2, &ntio);
	return res;
 }
