/**
 * @file quaternion.c <rc/math/quaternion.h>
 *
 * @brief      Collection of quaternion manipulation functions.
 *
 * Arrays are assumed to contain the quaternion components in the order [Wijk]
 *
 * @author     James Strawson
 * @date       2016
 */

#include <stdio.h>
#include <math.h>

#include <rc/math/quaternion.h>
#include "algebra_common.h"

double rc_quaternion_norm(rc_vector_t q)
{
	if(unlikely(q.len!=4)){
		fprintf(stderr, "ERROR in rc_quaternion_norm, expected vector of length 4\n");
		return -1.0;
	}
	return rc_vector_norm(q,2);
}


double rc_quaternion_norm_array(double q[4])
{
	double sum = 0.0;
	int i;
	if(unlikely(q==NULL)){
		fprintf(stderr, "ERROR in rc_quaternion_norm_array, received NULL pointer\n");
		return -1.0;
	}
	for(i=0;i<4;i++) sum+=q[i]*q[i];
	return sqrt(sum);
}


int rc_normalize_quaternion(rc_vector_t* q)
{
	int i;
	double len;
	// sanity checks
	if(unlikely(q->len!=4)){
		fprintf(stderr, "ERROR in rc_normalize_quaternion, expected vector of length 4\n");
		return -1;
	}
	len = rc_vector_norm(*q,2);
	if(unlikely(len<=0.0)){
		fprintf(stderr, "ERROR in rc_normalize_quaternion, unable to calculate norm\n");
		return -1;
	}
	for(i=0;i<4;i++) q->d[i]/=len;
	return 0;
}


int rc_normalize_quaternion_array(double q[4])
{
	int i;
	double len;
	double sum=0.0;
	for(i=0;i<4;i++) sum+=q[i]*q[i];
	len = sqrtf(sum);

	// can't check if length is below a constant value as q may be filled
	// with extremely small but valid doubles
	if(unlikely(fabs(len) < zero_tolerance)){
		fprintf(stderr, "ERROR in quaternion has 0 length\n");
		return -1;
	}
	for(i=0;i<4;i++) q[i]=q[i]/len;
	return 0;
}


int rc_quaternion_to_tb(rc_vector_t q, rc_vector_t* tb)
{
	if(unlikely(!q.initialized)){
		fprintf(stderr, "ERROR in rc_quaternion_to_tb, vector uninitialized\n");
		return -1;
	}
	if(unlikely(q.len!=4)){
		fprintf(stderr, "ERROR in rc_quaternion_to_tb, expected vector of length 4\n");
		return -1;
	}
	if(unlikely(rc_vector_alloc(tb,3))){
		fprintf(stderr, "ERROR in rc_quaternion_to_tb, failed to alloc array\n");
		return -1;
	}
	rc_quaternion_to_tb_array(q.d,tb->d);
	return 0;
}


int rc_quaternion_to_tb_array(double q[4], double tb[3])
{
	if(unlikely(q==NULL||tb==NULL)){
		fprintf(stderr,"ERROR: in rc_quaternion_to_tb_array, received NULL pointer\n");
		return -1;
	}
	// these functions are done with double precision since they cannot be
	// accelerated by the NEON unit and the VFP computes doubles at the same
	// speed as single-precision doubles
	tb[1] = asin(2.0*(q[0]*q[2] - q[1]*q[3]));
	tb[0] = atan2(2.0*(q[2]*q[3] + q[0]*q[1]),
		1.0 - 2.0*(q[1]*q[1] + q[2]*q[2]));
	tb[2] = atan2(2.0*(q[1]*q[2] + q[0]*q[3]),
		1.0 - 2.0*(q[2]*q[2] + q[3]*q[3]));
	return 0;
}


int rc_quaternion_from_tb(rc_vector_t tb, rc_vector_t* q)
{
	if(unlikely(!tb.initialized)){
		fprintf(stderr, "ERROR in rc_quaternion_from_tb, vector uninitialized\n");
		return -1;
	}
	if(unlikely(tb.len!=3)){
		fprintf(stderr, "ERROR in rc_quaternion_from_tb, expected vector of length 3\n");
		return -1;
	}
	if(unlikely(rc_vector_alloc(q,4))){
		fprintf(stderr, "ERROR in rc_quaternion_from_tb, failed to alloc array\n");
		return -1;
	}
	rc_quaternion_from_tb_array(tb.d,q->d);
	return 0;
}


int rc_quaternion_from_tb_array(double tb[3], double q[4])
{
	if(unlikely(q==NULL||q==NULL)){
		fprintf(stderr,"ERROR: in rc_quaternion_from_tb_array, received NULL pointer\n");
		return -1;
	}

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
	return 0;
}


int rc_quaternion_conjugate(rc_vector_t q, rc_vector_t* c)
{
	// sanity checks
	if(unlikely(!q.initialized)){
		fprintf(stderr, "ERROR in rc_quaternion_conjugate, vector uninitialized\n");
		return -1;
	}
	if(unlikely(q.len!=4)){
		fprintf(stderr, "ERROR in rc_quaternion_conjugate, expected vector of length 4\n");
		return -1;
	}
	if(unlikely(rc_vector_alloc(c,4))){
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


int rc_quaternion_conjugate_inplace(rc_vector_t* q)
{
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


int rc_quaternion_conjugate_array(double q[4], double c[4])
{
	if(unlikely(q==NULL||c==NULL)){
		fprintf(stderr,"ERROR: in rc_quaternion_conjugate_array, received NULL pointer\n");
		return -1;
	}
	c[0] =  q[0];
	c[1] = -q[1];
	c[2] = -q[2];
	c[3] = -q[3];
	return 0;
}


int rc_quaternion_conjugate_array_inplace(double q[4])
{
	if(unlikely(q==NULL)){
		fprintf(stderr,"ERROR: in rc_quaternion_conjugate_array_inplace, received NULL pointer\n");
		return -1;
	}
	q[1] = -q[1];
	q[2] = -q[2];
	q[3] = -q[3];
	return 0;
}


int rc_quaternion_imaginary_part(rc_vector_t q, rc_vector_t* img)
{
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
	if(unlikely(rc_vector_alloc(img,3))){
		fprintf(stderr, "ERROR in rc_quaternion_imaginary_part, failed to alloc array\n");
		return -1;
	}
	for(i=0;i<3;i++) img->d[i]=q.d[i+1];
	return 0;
}


int rc_quaternion_multiply(rc_vector_t a, rc_vector_t b, rc_vector_t* c)
{
	rc_matrix_t tmp = RC_MATRIX_INITIALIZER;
	// sanity checks
	if(unlikely(!a.initialized || !b.initialized)){
		fprintf(stderr, "ERROR in rc_quaternion_multiply, vector uninitialized\n");
		return -1;
	}
	if(unlikely(a.len!=4 || b.len!=4)){
		fprintf(stderr, "ERROR in rc_quaternion_multiply, expected vector of length 4\n");
		return -1;
	}
	if(unlikely(rc_matrix_alloc(&tmp,4,4))){
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
		rc_matrix_free(&tmp);
		return -1;
	}
	rc_matrix_free(&tmp);
	return 0;
}


int rc_quaternion_multiply_array(double a[4], double b[4], double c[4])
{
	if(unlikely(a==NULL||b==NULL||c==NULL)){
		fprintf(stderr,"ERROR: in rc_quaternion_multiply_array, received NULL pointer\n");
		return -1;
	}

	int i,j;
	double tmp[4][4];
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
	for(i=0;i<4;i++){
		c[i]=0.0;
		for(j=0;j<4;j++) c[i]+=tmp[i][j]*b[j];
	}
	return 0;
}


int rc_quaternion_rotate(rc_vector_t* p, rc_vector_t q)
{
	rc_vector_t conj = RC_VECTOR_INITIALIZER;
	rc_vector_t tmp  = RC_VECTOR_INITIALIZER;
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
		rc_vector_free(&conj);
		return -1;
	}
	if(unlikely(rc_quaternion_multiply(q,tmp,p))){
		fprintf(stderr, "ERROR in rc_quaternion_rotate_inplace, failed to multiply\n");
		rc_vector_free(&conj);
		rc_vector_free(&tmp);
		return -1;
	}
	// free memory
	rc_vector_free(&conj);
	rc_vector_free(&tmp);
	return 0;
}


int rc_quaternion_rotate_array(double p[4], double q[4])
{
	double conj[4], tmp[4];
	if(unlikely(p==NULL||q==NULL)){
		fprintf(stderr,"ERROR: in rc_quaternion_rotate_array, received NULL pointer\n");
		return -1;
	}
	// make a conjugate of q
	conj[0]= q[0];
	conj[1]=-q[1];
	conj[2]=-q[2];
	conj[3]=-q[3];
	// multiply tmp=pq*
	rc_quaternion_multiply_array(p,conj,tmp);
	// multiply p'=q*tmp
	rc_quaternion_multiply_array(q,tmp,p);
	return 0;
}


int rc_quaternion_rotate_vector(rc_vector_t* v, rc_vector_t q)
{
	rc_vector_t vq = RC_VECTOR_INITIALIZER;
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
	if(unlikely(rc_vector_alloc(&vq,4))){
		fprintf(stderr, "ERROR in rc_quaternion_rotate_vector, failed to alloc vector\n");
		return -1;
	}
	vq.d[0]=0.0;
	vq.d[1]=v->d[0];
	vq.d[2]=v->d[1];
	vq.d[3]=v->d[2];
	// rotate quaternion vector
	if(unlikely(rc_quaternion_rotate(&vq, q))){
		fprintf(stderr, "ERROR in rc_quaternion_rotate_vector, failed to rotate\n");
		rc_vector_free(&vq);
		return -1;
	}
	// populate v with result
	v->d[0]=vq.d[1];
	v->d[1]=vq.d[2];
	v->d[2]=vq.d[3];
	// free memory
	rc_vector_free(&vq);
	return 0;
}


int rc_quaternion_rotate_vector_array(double v[3], double q[4])
{
	double vq[4];
	if(unlikely(v==NULL||q==NULL)){
		fprintf(stderr,"ERROR: in rc_quaternion_rotate_vector_array, received NULL pointer\n");
		return -1;
	}
	// duplicate v into a quaternion with 0 real part
	vq[0]=0.0;
	vq[1]=v[0];
	vq[2]=v[1];
	vq[3]=v[2];
	// rotate quaternion vector
	rc_quaternion_rotate_array(vq, q);
	// populate v with result
	v[0]=vq[1];
	v[1]=vq[2];
	v[2]=vq[3];
	return 0;
}



int rc_quaternion_to_rotation_matrix(rc_vector_t q, rc_matrix_t* m)
{
	double q0s, q1s, q2s, q3s;
	// sanity checks
	if(unlikely(!q.initialized)){
		fprintf(stderr, "ERROR in rc_quaternion_to_rotation_matrix, vector uninitialized\n");
		return -1;
	}
	if(unlikely(q.len!=4)){
		fprintf(stderr, "ERROR in rc_quaternion_to_rotation_matrix, expected vector of length 4\n");
		return -1;
	}
	if(unlikely(rc_matrix_alloc(m,3,3))){
		fprintf(stderr, "ERROR in rc_quaternion_to_rotation_matrix, failed to alloc matrix\n");
		return -1;
	}
	// compute squares which will be used multiple times
	q0s = q.d[0]*q.d[0];
	q1s = q.d[1]*q.d[1];
	q2s = q.d[2]*q.d[2];
	q3s = q.d[3]*q.d[3];
	// diagonal entries
	m->d[0][0] = q0s+q1s-q2s-q3s;
	m->d[1][1] = q0s-q1s+q2s-q3s;
	m->d[2][2] = q0s-q1s-q2s+q3s;
	// upper triangle
	m->d[0][1] = 2.0 * (q.d[1]*q.d[2] - q.d[0]*q.d[3]);
	m->d[0][2] = 2.0 * (q.d[1]*q.d[3] + q.d[0]*q.d[2]);
	m->d[1][2] = 2.0 * (q.d[2]*q.d[3] - q.d[0]*q.d[1]);
	// lower triangle
	m->d[1][0] = 2.0 * (q.d[1]*q.d[2] + q.d[0]*q.d[3]);
	m->d[2][0] = 2.0 * (q.d[1]*q.d[3] - q.d[0]*q.d[2]);
	m->d[2][1] = 2.0 * (q.d[2]*q.d[3] + q.d[0]*q.d[1]);
	return 0;
}
