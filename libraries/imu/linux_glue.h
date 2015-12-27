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

#ifndef LINUX_GLUE_H
#define LINUX_GLUE_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "inv_mpu.h"

#define MIN_I2C_BUS 0
#define MAX_I2C_BUS 7

static inline int reg_int_cb(struct int_param_s *int_param)
{
	return 0;
}

#define i2c_write	linux_i2c_write
#define i2c_read	linux_i2c_read
#define delay_ms	linux_delay_ms
#define get_ms		linux_get_ms
#define log_i		printf
#define log_e		printf
#define min(a, b) 	((a < b) ? a : b)

void __no_operation(void);

void linux_set_i2c_bus(int bus);

int linux_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
       unsigned char length, unsigned char const *data);

int linux_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
       unsigned char length, unsigned char *data);
 
int linux_delay_ms(unsigned long num_ms);
int linux_get_ms(unsigned long *count);

#endif /* ifndef LINUX_GLUE_H */

