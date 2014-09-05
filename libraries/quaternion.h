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

#ifndef MPUQUATERNION_H
#define MPUQUATERNION_H

#include "vector3d.h"

#define QUAT_W		0
#define QUAT_X		1
#define QUAT_Y		2
#define QUAT_Z		3

typedef float quaternion_t[4]; 

void quaternionNormalize(quaternion_t q);
void quaternionToEuler(quaternion_t q, vector3d_t v);
void eulerToQuaternion(vector3d_t v, quaternion_t q);
void quaternionConjugate(quaternion_t s, quaternion_t d);
void quaternionMultiply(quaternion_t qa, quaternion_t qb, quaternion_t qd);


#endif /* MPUQUATERNION_H */

