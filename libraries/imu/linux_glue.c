////////////////////////////////////////////////////////////////////////////
//
//  This file is part of linux-mpu9150
//
//  Copyright (c) 2013 Pansenti, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <time.h>
#include <sys/time.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include "linux_glue.h"

#define MAX_WRITE_LEN 511

// default is the RPi
int i2c_bus = 1;

int i2c_fd;
int current_slave;
unsigned char txBuff[MAX_WRITE_LEN + 1];


void __no_operation(void) { }

int i2c_open()
{
	char buff[32];

	if (!i2c_fd) {
		sprintf(buff, "/dev/i2c-%d", i2c_bus);

#ifdef I2C_DEBUG
		printf("\t\t\ti2c_open() : %s\n", buff);
#endif

		i2c_fd = open(buff, O_RDWR);

		if (i2c_fd < 0) {
			perror("open(i2c_bus)");
			i2c_fd = 0;
			return -1;
		}
	}

	return 0;
}

void i2c_close()
{
	if (i2c_fd) {
		close(i2c_fd);
		i2c_fd = 0;
		current_slave = 0;
	}
}

int i2c_select_slave(unsigned char slave_addr)
{
	if (current_slave == slave_addr)
		return 0;

	if (i2c_open())
		return -1;

#ifdef I2C_DEBUG
	printf("\t\ti2c_select_slave(%02X)\n", slave_addr);
#endif

	if (ioctl(i2c_fd, I2C_SLAVE, slave_addr) < 0) {
		perror("ioctl(I2C_SLAVE)");
		return -1;
	}

	current_slave = slave_addr;

	return 0;
}

void linux_set_i2c_bus(int bus)
{
	if (i2c_fd)
		i2c_close();

	i2c_bus = bus;
}

int linux_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
       unsigned char length, unsigned char const *data)
{
	int result, i;

	if (length > MAX_WRITE_LEN) {
		printf("Max write length exceeded in linux_i2c_write()\n");
		return -1;
	}

#ifdef I2C_DEBUG
	printf("\tlinux_i2c_write(%02X, %02X, %u, [", slave_addr, reg_addr, length);

	if (length == 0) {
		printf(" NULL ] )\n");
	}
	else {
		for (i = 0; i < length; i++)
			printf(" %02X", data[i]); 

		printf(" ] )\n");
	}
#endif

	if (i2c_select_slave(slave_addr))
		return -1;

	if (length == 0) {
		result = write(i2c_fd, &reg_addr, 1);

		if (result < 0) {
			perror("write:1");
			return result;
		}
		else if (result != 1) {
			printf("Write fail:1 Tried 1 Wrote 0\n");
			return -1;
		}
	}
	else {
		txBuff[0] = reg_addr;

		for (i = 0; i < length; i++)
			txBuff[i+1] = data[i];

		result = write(i2c_fd, txBuff, length + 1);

		if (result < 0) {
			perror("write:2");
			return result;
		}
		else if (result < (int)length) {
			printf("Write fail:2 Tried %u Wrote %d\n", length, result); 
			return -1;
		}
	}

	return 0;
}

int linux_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
       unsigned char length, unsigned char *data)
{
	int tries, result, total;

#ifdef I2C_DEBUG
	int i;

	printf("\tlinux_i2c_read(%02X, %02X, %u, ...)\n", slave_addr, reg_addr, length);
#endif

	if (linux_i2c_write(slave_addr, reg_addr, 0, NULL))
		return -1;

	total = 0;
	tries = 0;

	while (total < length && tries < 5) {
		result = read(i2c_fd, data + total, length - total);

		if (result < 0) {
			perror("read");
			break;
		}

		total += result;

		if (total == length)
			break;

		tries++;		
		linux_delay_ms(10);
	}

	if (total < length)
		return -1;

#ifdef I2C_DEBUG
	printf("\tLeaving linux_i2c_read(), read %d bytes: ", total);

	for (i = 0; i < total; i++)
		printf("%02X ", data[i]); 

	printf("\n");
#endif

	return 0;
}

int linux_delay_ms(unsigned long num_ms)
{
	struct timespec ts;

	ts.tv_sec = num_ms / 1000;
	ts.tv_nsec = (num_ms % 1000) * 1000000;

	return nanosleep(&ts, NULL);
}

int linux_get_ms(unsigned long *count)
{
	struct timeval t;

	if (!count)
		return -1;

	if (gettimeofday(&t, NULL) < 0) {
		perror("gettimeofday");
		return -1;
	}

	*count = (t.tv_sec * 1000) + (t.tv_usec / 1000);	

	return 0;
}

