// Modified by James Strawson 2014
// changed min & max_sample_rate to 200hz

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

#ifndef MPU9150_H
#define MPU9150_H

#include "quaternion.h"
#include "linux_glue.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu9150.h"

#define MAG_SENSOR_RANGE 	4096
#define ACCEL_SENSOR_RANGE 	32000

// Somewhat arbitrary limits here. The values are samples per second.
// The MIN comes from the way we are timing our loop in imu and imucal.
// That's easily worked around, but no one probably cares.
// The MAX comes from the compass. This could be avoided with separate
// sample rates for the compass and the accel/gyros which can handle 
// faster sampling rates. This is a TODO item to see if it's useful. 
// There are some practical limits on the speed that come from a 'userland'
// implementation like this as opposed to a kernel or 'bare-metal' driver.
#define MIN_SAMPLE_RATE 5
#define MAX_SAMPLE_RATE 200

typedef struct {
	short offset[3];
	short range[3];
} caldata_t;

typedef struct {
	short rawGyro[3];
	short rawAccel[3];
	long rawQuat[4];
	unsigned long dmpTimestamp;

	short rawMag[3];
	unsigned long magTimestamp;

	short calibratedAccel[3];
	short calibratedMag[3];

	quaternion_t fusedQuat;
	vector3d_t fusedEuler;

	float lastDMPYaw;
	float lastYaw;
} mpudata_t;


void mpu9150_set_debug(int on);
int mpu9150_init(int i2c_bus, int sample_rate, int yaw_mixing_factor);
void mpu9150_exit();
int mpu9150_read(mpudata_t *mpu);
int mpu9150_read_dmp(mpudata_t *mpu);
int mpu9150_read_mag(mpudata_t *mpu);
void mpu9150_set_accel_cal(caldata_t *cal);
void mpu9150_set_mag_cal(caldata_t *cal);

int data_ready();
void calibrate_data(mpudata_t *mpu);
void tilt_compensate(quaternion_t magQ, quaternion_t unfusedQ);
int data_fusion(mpudata_t *mpu);
unsigned short inv_row_2_scale(const signed char *row);
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);


#endif /* MPU9150_H */

