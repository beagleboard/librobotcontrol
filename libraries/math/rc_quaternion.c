/*******************************************************************************
* rc_quaternion.c
*
* A collectino of functions for manipulating quaternions and tait-bryan angles.
*******************************************************************************/

#include "../roboticscape.h"
#include "../preprocessor_macros.h"
#include <math.h>
#include <stdio.h>

/*******************************************************************************
* float rc_quaternion_norm(rc_vector_t q)
*
* Returns the length of a quaternion vector by finding its 2-norm.
* Prints an error message and returns -1.0f on error.
*******************************************************************************/
float rc_quaternion_norm(rc_vector_t q){
	if(unlikely(q.len!=4)){
		fprintf(stderr, "ERROR in rc_quaternion_norm, expected vector of length 4\n");
		return -1.0f;
	}
	return rc_vector_norm(q,2);
}

/*******************************************************************************
* float rc_quaternion_norm_array(float q[4])
*
* Returns the length of a quaternion vector by finding its 2-norm.
* Prints an error message and returns -1.0f on error.
*******************************************************************************/
float rc_quaternion_norm_array(float q[4]){
	double sum = 0.0;
	int i;
	if(unlikely(q==NULL)){
		fprintf(stderr, "ERROR in rc_quaternion_norm_array, received NULL pointer\n");
		return -1.0f;
	}
	for(i=0;i<4;i++) sum+=q[i]*q[i];
	return sqrt(sum);
}

/*******************************************************************************
* int rc_normalize_quaternion(rc_vector_t* q)
*
* Normalizes a quaternion in-place to have length 1.0. Returns 0 on success.
* Returns -1 if the quaternion is uninitialized or has 0 length.
*******************************************************************************/
int rc_normalize_quaternion(rc_vector_t* q){
	int i;
	float len;
	// sanity checks
	if(unlikely(q->len!=4)){
		fprintf(stderr, "ERROR in rc_normalize_quaternion, expected vector of length 4\n");
		return -1;
	}
	len = rc_vector_norm(*q,2);
	if(unlikely(len<=0.0f)){
		fprintf(stderr, "ERROR in rc_normalize_quaternion, unable to calculate norm\n");
		return -1;
	}
	for(i=0;i<3;i++) q->d[i]/=len;
	return 0;
}

/*******************************************************************************
* int rc_normalize_quaternion_array(float q[4])
*
* Same as normalize_quaternion but performs the action on an array instead of
* a rc_vector_t type. 
*******************************************************************************/
int rc_normalize_quaternion_array(float q[4]){
	int i;
	float len;
	float sum=0.0f;
	for(i=0;i<3;i++) sum+=q[i]*q[i];
	len = sqrtf(sum);

	// can't check if length is below a constant value as q may be filled
	// with extremely small but valid floats
	if(unlikely(len==0.0f)){
		fprintf(stderr, "ERROR in quaternion has 0 length\n");
		return -1;
	}
	for(i=0;i<3;i++) q[i]=q[i]/len;
	return 0;
}

/*******************************************************************************
* int rc_quaternion_to_tb(rc_vector_t q, rc_vector_t* tb)
*
* Populates vector tb with 321 Tait Bryan angles in array order XYZ with
* operation order 321(yaw-Z, pitch-Y, roll-x). If tb is already allocated and of
* length 3 then the new values are written in place, otherwise any existing 
* memory is freed and a new vector of length 3 is allocated for tb. 
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_quaternion_to_tb(rc_vector_t q, rc_vector_t* tb){
	if(unlikely(!q.initialized)){
		fprintf(stderr, "ERROR in rc_quaternion_to_tb, vector uninitialized\n");
		return -1;
	}
	if(unlikely(q.len!=4)){
		fprintf(stderr, "ERROR in rc_quaternion_to_tb, expected vector of length 4\n");
		return -1;
	}
	if(unlikely(rc_alloc_vector(tb,3))){
		fprintf(stderr, "ERROR in rc_quaternion_to_tb, failed to alloc array\n");
		return -1;
	}
	rc_quaternion_to_tb_array(q.d,tb->d);
	return 0;
}

/*******************************************************************************
* void rc_quaternion_to_tb_array(float q[4], float tb[3])
*
* Same as rc_quaternion_to_tb but takes arrays instead.
*******************************************************************************/
void rc_quaternion_to_tb_array(float q[4], float tb[3]){
	// these functions are done with double precision since they cannot be
	// accelerated by the NEON unit and the VFP computes doubles at the same
	// speed as single-precision floats
	tb[1] = asin(2.0*(q[0]*q[2] - q[1]*q[3]));
	tb[0] = atan2(2.0*(q[2]*q[3] + q[0]*q[1]),
										1.0 - 2.0*(q[1]*q[1] + q[2]*q[2]));
	tb[2] = atan2(2.0*(q[1]*q[2] + q[0]*q[3]),
										1.0 - 2.0*(q[2]*q[2] + q[3]*q[3]));
	return;
}

/*******************************************************************************
* int rc_tb_to_quaternion(rc_vector_t tb, rc_vector_t* q)
*
* Populates quaternion vector q with the quaternion corresponding to the 
* tait-bryan pitch-roll-yaw values in vector tb. If q is already of length 4
* then old contents are simply overwritten. Otherwise q'd existing memory is
* freed and new memory is allocated to avoid memory leaks.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_tb_to_quaternion(rc_vector_t tb, rc_vector_t* q){
	if(unlikely(!tb.initialized)){
		fprintf(stderr, "ERROR in rc_tb_to_quaternion, vector uninitialized\n");
		return -1;
	}
	if(unlikely(tb.len!=3)){
		fprintf(stderr, "ERROR in rc_tb_to_quaternion, expected vector of length 3\n");
		return -1;
	}
	if(unlikely(rc_alloc_vector(q,4))){
		fprintf(stderr, "ERROR in rc_tb_to_quaternion, failed to alloc array\n");
		return -1;
	}
	rc_tb_to_quaternion_array(tb.d,q->d);
	return 0;
}

/*******************************************************************************
* void rc_tb_to_quaternion_array(float tb[3], float q[4])
*
* Like rc_tb_to_quaternion but takes arrays as arguments.
*******************************************************************************/
void rc_tb_to_quaternion_array(float tb[3], float q[4]){
	double tbt[3];
	tbt[0]=tb[0]/2.0;
	tbt[1]=tb[1]/2.0;
	tbt[2]=tb[2]/2.0;
	double cosX2 = cos(tbt[0]);
	double sinX2 = sin(tbt[0]);
	double cosY2 = cos(tbt[1]);
	double sinY2 = sin(tbt[1]);
	double cosZ2 = cos(tbt[2]);
	double sinZ2 = sin(tbt[2]);
	q[0] = cosX2*cosY2*cosZ2 + sinX2*sinY2*sinZ2;
	q[1] = sinX2*cosY2*cosZ2 - cosX2*sinY2*sinZ2;
	q[2] = cosX2*sinY2*cosZ2 + sinX2*cosY2*sinZ2;
	q[3] = cosX2*cosY2*sinZ2 - sinX2*sinY2*cosZ2;
	rc_normalize_quaternion_array(q);
	return;
}

/*******************************************************************************
* int rc_quaternion_conjugate(rc_vector_t q, rc_vector_t* c)
*
* Populates quaternion vector c with the conjugate of quaternion q where the 3 
* imaginary parts ijk are multiplied by -1. If c is already of length 4 then the
* old values are overwritten. Otherwise the old memory in c is freed and new
* memory is allocated to help prevent memory leaks. 
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_quaternion_conjugate(rc_vector_t q, rc_vector_t* c){
	// sanity checks
	if(unlikely(!q.initialized)){
		fprintf(stderr, "ERROR in rc_quaternion_conjugate, vector uninitialized\n");
		return -1;
	}
	if(unlikely(q.len!=4)){
		fprintf(stderr, "ERROR in rc_quaternion_conjugate, expected vector of length 4\n");
		return -1;
	}
	if(unlikely(rc_alloc_vector(c,4))){
		fprintf(stderr, "ERROR in rc_quaternion_conjugate, failed to alloc array\n");
		return -1;
	}
	// populate conjugate
	c->d[0] =  q.d[0];
	c->d[1] = -q.d[1];
	c->d[2] = -q.d[2];
	c->d[3] = -q.d[3];
	return 0;
}

/*******************************************************************************
* int rc_quaternion_conjugate_inplace(rc_vector_t* q)
*
* Conjugates quaternion q by multiplying the 3 imaginary parts ijk by -1.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_quaternion_conjugate_inplace(rc_vector_t* q){
	// sanity checks
	if(unlikely(!q->initialized)){
		fprintf(stderr, "ERROR in rc_quaternion_conjugate, vector uninitialized\n");
		return -1;
	}
	if(unlikely(q->len!=4)){
		fprintf(stderr, "ERROR in rc_quaternion_conjugate, expected vector of length 4\n");
		return -1;
	}
	// populate conjugate
	q->d[1] = -q->d[1];
	q->d[2] = -q->d[2];
	q->d[3] = -q->d[3];
	return 0;
}

/*******************************************************************************
* void rc_quaternion_conjugate_array(float q[4], float c[4])
*
* Populates quaternion vector c with the conjugate of quaternion q where the 3 
* imaginary parts ijk are multiplied by -1.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
void rc_quaternion_conjugate_array(float q[4], float c[4]){
	c[0] =  q[0];
	c[1] = -q[1];
	c[2] = -q[2];
	c[3] = -q[3];
	return;
}

/*******************************************************************************
* void rc_quaternion_conjugate_array_inplace(float q[4])
*
* Conjugates quaternion q by multiplying the 3 imaginary parts ijk by -1.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
void rc_quaternion_conjugate_array_inplace(float q[4]){
	q[1] = -q[1];
	q[2] = -q[2];
	q[3] = -q[3];
	return;
}

/*******************************************************************************
* int rc_quaternion_imaginary_part(rc_vector_t q, rc_vector_t* img)
*
* Populates vector i with the imaginary components ijk of of quaternion vector
* q. If img is already of length 3 then its original contents are overwritten.
* Otherwise the original allocated memory is freed and new memory is allocated.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_quaternion_imaginary_part(rc_vector_t q, rc_vector_t* img){
	int i;
	// sanity checks
	if(unlikely(!q.initialized)){
		fprintf(stderr, "ERROR in rc_quaternion_imaginary_part, vector uninitialized\n");
		return -1;
	}
	if(unlikely(q.len!=4)){
		fprintf(stderr, "ERROR in rc_quaternion_imaginary_part, expected vector of length 4\n");
		return -1;
	}
	if(unlikely(rc_alloc_vector(img,3))){
		fprintf(stderr, "ERROR in rc_quaternion_imaginary_part, failed to alloc array\n");
		return -1;
	}
	for(i=0;i<3;i++) img->d[i]=q.d[i+1];
	return 0;
}

/*******************************************************************************
* int rc_quaternion_multiply(rc_vector_t a, rc_vector_t b, rc_vector_t* c)
*
* Calculates the quaternion Hamilton product ab=c and places the result in
* vector argument c. If c is already of length 4 then the old values are 
* overwritten. Otherwise the old memory in c is freed and new memory is
* allocated to help prevent memory leaks. 
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_quaternion_multiply(rc_vector_t a, rc_vector_t b, rc_vector_t* c){
	rc_matrix_t tmp = rc_empty_matrix();
	// sanity checks
	if(unlikely(!a.initialized || !b.initialized)){
		fprintf(stderr, "ERROR in rc_quaternion_multiply, vector uninitialized\n");
		return -1;
	}
	if(unlikely(a.len!=4 || b.len!=4)){
		fprintf(stderr, "ERROR in rc_quaternion_multiply, expected vector of length 4\n");
		return -1;
	}
	if(unlikely(rc_alloc_matrix(&tmp,4,4))){
		fprintf(stderr, "ERROR in rc_quaternion_multiply, failed to alloc matrix\n");
		return -1;
	}
	// construct tmp matrix
	tmp.d[0][0] =  a.d[0];
	tmp.d[0][1] = -a.d[1];
	tmp.d[0][2] = -a.d[2];
	tmp.d[0][3] = -a.d[3];
	tmp.d[1][0] =  a.d[1];
	tmp.d[1][1] =  a.d[0];
	tmp.d[1][2] = -a.d[3];
	tmp.d[1][3] =  a.d[2];
	tmp.d[2][0] =  a.d[2];
	tmp.d[2][1] =  a.d[3];
	tmp.d[2][2] =  a.d[0];
	tmp.d[2][3] = -a.d[1];
	tmp.d[3][0] =  a.d[3];
	tmp.d[3][1] = -a.d[2];
	tmp.d[3][2] =  a.d[1];
	tmp.d[3][3] =  a.d[0];
	// multiply
	if(unlikely(rc_matrix_times_col_vec(tmp,b,c))){
		fprintf(stderr, "ERROR in rc_quaternion_multiply, failed to multiply\n");
		rc_free_matrix(&tmp);
		return -1;
	}
	rc_free_matrix(&tmp);
	return 0;
}

/*******************************************************************************
* void rc_quaternion_multiply_array(float a[4], float b[4], float c[4])
*
* Calculates the quaternion Hamilton product ab=c and places the result in c
*******************************************************************************/
void rc_quaternion_multiply_array(float a[4], float b[4], float c[4]){
	int i,j;
	float tmp[4][4];
	// construct tmp matrix
	tmp[0][0] =  a[0];
	tmp[0][1] = -a[1];
	tmp[0][2] = -a[2];
	tmp[0][3] = -a[3];
	tmp[1][0] =  a[1];
	tmp[1][1] =  a[0];
	tmp[1][2] = -a[3];
	tmp[1][3] =  a[2];
	tmp[2][0] =  a[2];
	tmp[2][1] =  a[3];
	tmp[2][2] =  a[0];
	tmp[2][3] = -a[1];
	tmp[3][0] =  a[3];
	tmp[3][1] = -a[2];
	tmp[3][2] =  a[1];
	tmp[3][3] =  a[0];
	// multiply
	for(i=0;i<3;i++){
		c[i]=0.0f;
		for(j=0;j<3;j++) c[i]+=tmp[i][j]*b[j];
	}
	return;
}

/*******************************************************************************
* int rc_rotate_quaternion(rc_vector_t* p, rc_vector_t q)
*
* Rotates the quaternion p by quaternion q with the operation p'=qpq* 
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_rotate_quaternion(rc_vector_t* p, rc_vector_t q){
	rc_vector_t conj = rc_empty_vector();
	rc_vector_t tmp  = rc_empty_vector();
	// sanity checks
	if(unlikely(!q.initialized || !p->initialized)){
		fprintf(stderr, "ERROR in rc_quaternion_rotate_inplace, vector uninitialized\n");
		return -1;
	}
	if(unlikely(q.len!=4 || p->len!=4)){
		fprintf(stderr, "ERROR in rc_quaternion_rotate_inplace, expected vector of length 4\n");
		return -1;
	}
	// compute p'=qpq*
	if(unlikely(rc_quaternion_conjugate(q, &conj))){
		fprintf(stderr, "ERROR in rc_quaternion_rotate_inplace, failed to conjugate\n");
		return -1;
	}
	if(unlikely(rc_quaternion_multiply(*p,conj,&tmp))){
		fprintf(stderr, "ERROR in rc_quaternion_rotate_inplace, failed to multiply\n");
		rc_free_vector(&conj);
		return -1;
	}
	if(unlikely(rc_quaternion_multiply(q,tmp,p))){
		fprintf(stderr, "ERROR in rc_quaternion_rotate_inplace, failed to multiply\n");
		rc_free_vector(&conj);
		rc_free_vector(&tmp);
		return -1;
	}
	// free memory
	rc_free_vector(&conj);
	rc_free_vector(&tmp);
	return 0;
}

/*******************************************************************************
* void rc_rotate_quaternion_array(float p[4], float q[4])
*
* Rotates the quaternion p by quaternion q with the operation p'=qpq* 
*******************************************************************************/
void rc_rotate_quaternion_array(float p[4], float q[4]){
	float conj[4], tmp[4];
	// make a conjugate of q
	conj[0]= q[0];
	conj[1]=-q[1];
	conj[2]=-q[2];
	conj[3]=-q[3];
	// multiply tmp=pq*
	rc_quaternion_multiply_array(p,conj,tmp);
	// multiply p'=q*tmp
	rc_quaternion_multiply_array(q,tmp,p);
	return;
}

/*******************************************************************************
* int rc_quaternion_rotate_vector(rc_vector_t* v, rc_vector_t q)
*
* Rotate a 3D vector v in-place about the origin by quaternion q by converting
* v to a quaternion and performing the operation p'=qpq* 
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_quaternion_rotate_vector(rc_vector_t* v, rc_vector_t q){
	rc_vector_t vq = rc_empty_vector();
	// sanity checks
	if(unlikely(!q.initialized || !v->initialized)){
		fprintf(stderr, "ERROR in rc_quaternion_rotate_vector, vector uninitialized\n");
		return -1;
	}
	if(unlikely(q.len!=4 || v->len!=3)){
		fprintf(stderr, "ERROR in rc_quaternion_rotate_vector, incorrect length\n");
		return -1;
	}
	// duplicate v into a quaternion with 0 real part
	if(unlikely(rc_alloc_vector(&vq,4))){
		fprintf(stderr, "ERROR in rc_quaternion_rotate_vector, failed to alloc vector\n");
		return -1;
	}
	vq.d[0]=0.0f;
	vq.d[1]=v->d[0];
	vq.d[2]=v->d[1];
	vq.d[3]=v->d[2];
	// rotate quaternion vector
	if(unlikely(rc_rotate_quaternion(&vq, q))){
		fprintf(stderr, "ERROR in rc_quaternion_rotate_vector, failed to rotate\n");
		rc_free_vector(&vq);
		return -1;
	}
	// populate v with result
	v->d[0]=vq.d[1];
	v->d[1]=vq.d[2];
	v->d[2]=vq.d[3];
	// free memory
	rc_free_vector(&vq);
	return 0;
}

/*******************************************************************************
* void rc_quaternion_rotate_vector_array(float v[3], float q[4])
*
* Rotate a 3D vector v in-place about the origin by quaternion q by converting
* v to a quaternion and performing the operation p'=qpq* 
*******************************************************************************/
void rc_quaternion_rotate_vector_array(float v[3], float q[4]){
	float vq[4];
	// duplicate v into a quaternion with 0 real part
	vq[0]=0.0f;
	vq[1]=v[0];
	vq[2]=v[1];
	vq[3]=v[2];
	// rotate quaternion vector
	rc_rotate_quaternion_array(vq, q);
	// populate v with result
	v[0]=vq[1];
	v[1]=vq[2];
	v[2]=vq[3];
	return;
}


/*******************************************************************************
* int rc_quaternion_to_rotation_matrix(rc_vector_t q, rc_matrix_t* m)
*
* Populates m with a 3x3 rotation matrix which would perform the equivalent
* rotation as quaternion q when multiplied by a 3D vector. If m is already
* 3x3 then its contents are overwritten, otherwise its existing memory is freed
* and new memory is allocated.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_quaternion_to_rotation_matrix(rc_vector_t q, rc_matrix_t* m){
	float q0s, q1s, q2s, q3s;
	// sanity checks
	if(unlikely(!q.initialized)){
		fprintf(stderr, "ERROR in rc_quaternion_to_rotation_matrix, vector uninitialized\n");
		return -1;
	}
	if(unlikely(q.len!=4)){
		fprintf(stderr, "ERROR in rc_quaternion_to_rotation_matrix, expected vector of length 4\n");
		return -1;
	}
	if(unlikely(rc_alloc_matrix(m,3,3))){
		fprintf(stderr, "ERROR in rc_quaternion_to_rotation_matrix, failed to alloc matrix\n");
		return -1;
	}
	// compute squares which will be used multiple times
	q0s = q.d[0]*q.d[0];
	q1s = q.d[1]*q.d[1];
	q2s = q.d[2]*q.d[2];
	q3s = q.d[3]*q.d[3];
	// compute diagonal entries
	m->d[0][0] = q0s+q1s-q2s-q3s;
	m->d[1][1] = q0s-q1s+q2s-q3s;
	m->d[2][2] = q0s-q1s-q2s+q3s;
	// compute upper triangle
	m->d[0][1] = 2.0f * (q.d[1]*q.d[2] - q.d[0]*q.d[3]);
	m->d[0][2] = 2.0f * (q.d[1]*q.d[3] + q.d[0]*q.d[2]);
	m->d[1][2] = 2.0f * (q.d[2]*q.d[3] - q.d[0]*q.d[1]);
	// mirror lower triangle
	m->d[1][0] = m->d[0][1];
	m->d[2][0] = m->d[0][2];
	m->d[2][1] = m->d[1][2];
	return 0;
}
