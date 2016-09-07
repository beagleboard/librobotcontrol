/*******************************************************************************
The functions below are from the Pansenti mpu-9150 repository and modified
slightly to integrate into the robotics cape library. At the request of the
original author here is the original license:


Copyright (c) 2013 Pansenti, LLC

Permission is hereby granted, free of charge, to any person obtaining a copy of 
this software and associated documentation files (the "Software"), to deal in 
the Software without restriction, including without limitation the rights to 
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies 
of the Software, and to permit persons to whom the Software is furnished to do 
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all 
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR 
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER 
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*******************************************************************************/

#include "../robotics_cape.h"
#include <math.h>

#define VEC3_X		0
#define VEC3_Y		1
#define VEC3_Z		2

float vector3DotProduct(float a[3], float b[3]){
	return a[VEC3_X] * b[VEC3_X] + a[VEC3_Y] * b[VEC3_Y] + a[VEC3_Z] * b[VEC3_Z];  
}

void vector3CrossProduct(float a[3], float b[3], float d[3]){
	d[VEC3_X] = a[VEC3_Y] * b[VEC3_Z] - a[VEC3_Z] * b[VEC3_Y];
	d[VEC3_Y] = a[VEC3_Z] * b[VEC3_X] - a[VEC3_X] * b[VEC3_Z];
	d[VEC3_Z] = a[VEC3_X] * b[VEC3_Y] - a[VEC3_Y] * b[VEC3_X];
}


float quaternionNorm(float q[4]){
	return sqrtf(q[QUAT_W] * q[QUAT_W] + q[QUAT_X] * q[QUAT_X] +  
				q[QUAT_Y] * q[QUAT_Y] + q[QUAT_Z] * q[QUAT_Z]);
}

void normalizeQuaternion(float q[4]){
	float length = quaternionNorm(q);

	if (length == 0)
		return;

	q[QUAT_W] /= length;
	q[QUAT_X] /= length;
	q[QUAT_Y] /= length;
	q[QUAT_Z] /= length;
}

void quaternionToTaitBryan(float q[4], float v[3]){
	// fix roll near poles with this tolerance
	float pole = (float)M_PI / 2.0f - 0.05f;

	v[VEC3_Y] = asinf(2.0f * (q[QUAT_W] * q[QUAT_Y] - q[QUAT_X] * q[QUAT_Z]));

	if ((v[VEC3_Y] < pole) && (v[VEC3_Y] > -pole)) {
		v[VEC3_X] = atan2f(2.0f * (q[QUAT_Y] * q[QUAT_Z] + q[QUAT_W] * q[QUAT_X]),
					1.0f - 2.0f * (q[QUAT_X] * q[QUAT_X] + q[QUAT_Y] * q[QUAT_Y]));
	}

	v[VEC3_Z] = atan2f(2.0f * (q[QUAT_X] * q[QUAT_Y] + q[QUAT_W] * q[QUAT_Z]),
					1.0f - 2.0f * (q[QUAT_Y] * q[QUAT_Y] + q[QUAT_Z] * q[QUAT_Z]));
}

void TaitBryanToQuaternion(float v[3], float q[4]){
	float cosX2 = cosf(v[VEC3_X] / 2.0f);
	float sinX2 = sinf(v[VEC3_X] / 2.0f);
	float cosY2 = cosf(v[VEC3_Y] / 2.0f);
	float sinY2 = sinf(v[VEC3_Y] / 2.0f);
	float cosZ2 = cosf(v[VEC3_Z] / 2.0f);
	float sinZ2 = sinf(v[VEC3_Z] / 2.0f);

	q[QUAT_W] = cosX2 * cosY2 * cosZ2 + sinX2 * sinY2 * sinZ2;
	q[QUAT_X] = sinX2 * cosY2 * cosZ2 - cosX2 * sinY2 * sinZ2;
	q[QUAT_Y] = cosX2 * sinY2 * cosZ2 + sinX2 * cosY2 * sinZ2;
	q[QUAT_Z] = cosX2 * cosY2 * sinZ2 - sinX2 * sinY2 * cosZ2;

	normalizeQuaternion(q);
}

void tilt_compensate(float in[4], float tilt[4], float out[4]){
	float tiltConjugate[4];
	float tempQ[4];

	quaternionConjugate(tilt, tiltConjugate);
	quaternionMultiply(in, tiltConjugate, tempQ);
	quaternionMultiply(tilt, tempQ, out);
}

void quaternionConjugate(float in[4], float out[4]){
	out[QUAT_W] = in[QUAT_W];
	out[QUAT_X] = -in[QUAT_X];
	out[QUAT_Y] = -in[QUAT_Y];
	out[QUAT_Z] = -in[QUAT_Z];
}
	
void quaternionMultiply(float a[4], float b[4], float out[4]){
	float va[3], vb[3], crossAB[3];
	float dotAB;
	
	va[VEC3_X] = a[QUAT_X];
	va[VEC3_Y] = a[QUAT_Y];
	va[VEC3_Z] = a[QUAT_Z];

	vb[VEC3_X] = b[QUAT_X];
	vb[VEC3_Y] = b[QUAT_Y];
	vb[VEC3_Z] = b[QUAT_Z];

	dotAB = vector3DotProduct(va, vb);
	vector3CrossProduct(va, vb, crossAB);
	
	out[QUAT_W] = a[QUAT_W]*b[QUAT_W]-dotAB;
	out[QUAT_X] = a[QUAT_W]*vb[VEC3_X]+b[QUAT_W]*va[VEC3_X]+crossAB[VEC3_X];
	out[QUAT_Y] = a[QUAT_W]*vb[VEC3_Y]+b[QUAT_W]*va[VEC3_Y]+crossAB[VEC3_Y];
	out[QUAT_Z] = a[QUAT_W]*vb[VEC3_Z]+b[QUAT_W]*va[VEC3_Z]+crossAB[VEC3_Z];
}
