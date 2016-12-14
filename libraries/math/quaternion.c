#include "../roboticscape.h"
#include <math.h>
#include <stdio.h>

/*******************************************************************************
* double quaternion_norm(vector_t quat)
*
* This gives the length of the quaternion vector which is really just a 2-norm
* quick sanity check for length
*******************************************************************************/
double quaternion_norm(vector_t q){
	if (q.len != 4){
		printf("ERROR: quaternion must be a vector of length 4\n");
		return -1;
	}
	return vector_norm(q,2);
}

/*******************************************************************************
* int normalize_quaternion(vector_t* q)
*
* normalizes a quaternion in-place to have length 1. Returns 0 on success.
* Returns -1 if the quaternion is uninitialized or has 0 length.
*******************************************************************************/
int normalize_quaternion(vector_t* q){
	int i;
	double len;

	if(!q->initialized){
		printf("ERROR: quaternion vector not initialized yet\n");
		return -1;
	}
	if(q->len != 4){
		printf("ERROR: quaternion must be a vector of length 4\n");
		return -1;
	}
	len = quaternion_norm(*q);
	if(len == 0.0){
		printf("ERROR: quaternion has 0 length\n");
		return -1;
	}
	for(i=0;i<3;i++){
		q->data[i]/=len;
	}
	return 0;
}

/*******************************************************************************
* int normalize_quaternion_array(double q[4])
*
* same as normalize_quaternion but performs the action on an array instead of
* a vector_t type. 
*******************************************************************************/
int normalize_quaternion_array(double q[4]){
	double len = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
	if(len==0.0){
		printf("ERROR: quaternion has 0 length\n");
		return -1;
	}
	q[0]/=len;
	q[1]/=len;
	q[2]/=len;
	q[3]/=len;
	return 0;
}


/*******************************************************************************
* vector_r quaternion_to_tb(vector_t q)
*
* returns a vector of length 3 with 321 Tait Bryan angles 
* (XYZ, pitch roll yaw) from quaternion q.
*******************************************************************************/
vector_t quaternion_to_tb(vector_t q){
	vector_t out;
	if(!q.initialized){
		printf("ERROR: quaternion vector not initialized yet\n");
		return empty_vector();
	}
	if(q.len != 4){
		printf("ERROR: quaternion must be a vector of length 4\n");
		return empty_vector();
	}

	out = create_vector(3);
	printf("before tb array\n");
	quaternion_to_tb_array(q.data,out.data);
	printf("after tb_array\n");
	return out;
}


/*******************************************************************************
* void quaternion_to_tb(double q[4], double tb[3])
*
* populates array v with 321 Tait Bryan angles in array order XYZ with
* operation order 321(yaw-Z, pitch-Y, roll-x) about the IMU's indicated axis
* Note that the IMU's axis do not necessarily line up with NED coordinates
* quaternion q. same as quaternion_to_tb but takes an array instead
*******************************************************************************/
void quaternion_to_tb_array(double q[4], double tb[3]){
	tb[1] = asin(2.0*(q[0]*q[2] - q[1]*q[3]));
	tb[0] = atan2(2.0*(q[2]*q[3] + q[0]*q[1]),
										1.0 - 2.0*(q[1]*q[1] + q[2]*q[2]));
	tb[2] = atan2(2.0*(q[1]*q[2] + q[0]*q[3]),
										1.0 - 2.0*(q[2]*q[2] + q[3]*q[3]));

	// double q0s, q1s, q2s, q3s;
	// // compute squares which will be used multiple times
	// q0s = q[0]*q[0];
	// q1s = q[1]*q[1];
	// q2s = q[2]*q[2];
	// q3s = q[3]*q[3];

	// // alpha yaw about z
	// tb[2]=atan2(2.0*q[1]*q[2] - 2.0*q[0]*q[3], q0s+q1s-q2s-q3s);
	// // beta pitch about Y
	// tb[1]=-asin(2*q[1]*q[3] + 2*q[0]*q[2]);
	// // gamma roll about x
	// tb[0]=atan2(2.0*q[2]*q[3] - 2.0*q[0]*q[1], q0s-q1s-q2s+q3s);

	return;
}


/*******************************************************************************
* vector_t tb_to_quaternion(vector_t tb)
*
* returns a vector of length 4 containing the quaternion corresponding
* to the tait-bryan pitch-roll-yaw values in vector v
*******************************************************************************/
vector_t tb_to_quaternion(vector_t tb){
	vector_t out;
	if(!tb.initialized){
		printf("ERROR: tait-bryan vector not initialized yet\n");
		return empty_vector();
	}
	if(tb.len != 3){
		printf("ERROR: tait-bryan vector must have length 3\n");
		return empty_vector();
	}

	tb_to_quaternion_array(tb.data,out.data);
	return out;
}

/*******************************************************************************
* void tb_to_quaternion_array(double tb[3], double q[4])
*
* populates q with the normalized quaternion corresponding to the tait-bryan 
* angles in array tb
*******************************************************************************/
void tb_to_quaternion_array(double tb[3], double q[4]){
	double cosX2 = cos(tb[0]/2.0);
	double sinX2 = sin(tb[0]/2.0);
	double cosY2 = cos(tb[1]/2.0);
	double sinY2 = sin(tb[1]/2.0);
	double cosZ2 = cos(tb[2]/2.0);
	double sinZ2 = sin(tb[2]/2.0);

	q[0] = cosX2*cosY2*cosZ2 + sinX2*sinY2*sinZ2;
	q[1] = sinX2*cosY2*cosZ2 - cosX2*sinY2*sinZ2;
	q[2] = cosX2*sinY2*cosZ2 + sinX2*cosY2*sinZ2;
	q[3] = cosX2*cosY2*sinZ2 - sinX2*sinY2*cosZ2;

	normalize_quaternion_array(q);
	return;
}

/*******************************************************************************
* vector_t quaternion_conjugate(vector_t q)
*
* returns a vector of length 4 containing the conjugate of quaternion q
* where the 3 imaginary parts ijk are multiplied by -1
*******************************************************************************/
vector_t quaternion_conjugate(vector_t q){
	vector_t out;
	printf("before create vector in quaternion conjugate\n");
	out = create_vector(4);
	printf("after create vector in quaternion conjugate\n");
	out.data[0] =  q.data[0];
	out.data[1] = -q.data[1];
	out.data[2] = -q.data[2];
	out.data[3] = -q.data[3];
	return out;
}

/*******************************************************************************
* void quaternion_conjugate_array(double q[4], double conj[4])
*
* returns a vector of length 4 containing the conjugate of quaternion q
* where the 3 imaginary parts ijk are multiplied by -1
*******************************************************************************/
void quaternion_conjugate_array(double q[4], double conj[4]){
	conj[0] =  q[0];
	conj[1] = -q[1];
	conj[2] = -q[2];
	conj[3] = -q[3];
	return;
}

/*******************************************************************************
* vector_t quaternion_imaginary_part(vector_t q)
*
* returns a vector of length 3 containing the imaginary part of quaternion q
*******************************************************************************/
vector_t quaternion_imaginary_part(vector_t q){
	if(!q.initialized){
		printf("ERROR: quaternion vector not initialized yet\n");
		return empty_vector();
	}
	if(q.len != 4){
		printf("ERROR: quaternion must be a vector of length 4\n");
		return empty_vector();
	}

	vector_t out = create_vector(3);
	int i;

	for(i=0;i<3;i++){
		out.data[i]=q.data[i+1];
	}

	return out;
}

/*******************************************************************************
* vector_t quaternion_multiply(vector_t a, vector_t b)
*
* returns a vector of length 4 containing the quaternion Hamilton product q=ab
*******************************************************************************/
vector_t quaternion_multiply(vector_t a, vector_t b){
	vector_t out;
	matrix_t tmp;

	// sanity checks
	if(!a.initialized || !b.initialized){
		printf("ERROR: quaternion vector not initialized yet\n");
		return empty_vector();
	}
	if(a.len!=4 || b.len!=4){
		printf("ERROR: quaternion must be a vector of length 4\n");
		return empty_vector();
	}
	// construct tmp matrix
	tmp = create_matrix(4,4);
	tmp.data[0][0] = a.data[0];
	tmp.data[0][1] = -a.data[1];
	tmp.data[0][2] = -a.data[2];
	tmp.data[0][3] = -a.data[3];
	tmp.data[1][0] = a.data[1];
	tmp.data[1][1] = a.data[0];
	tmp.data[1][2] = -a.data[3];
	tmp.data[1][3] = a.data[2];
	tmp.data[2][0] = a.data[2];
	tmp.data[2][1] = a.data[3];
	tmp.data[2][2] = a.data[0];
	tmp.data[2][3] = -a.data[1];
	tmp.data[3][0] = a.data[3];
	tmp.data[3][1] = -a.data[2];
	tmp.data[3][2] = a.data[1];
	tmp.data[3][3] = a.data[0];

	out = matrix_times_col_vec(tmp,b);

	printf("before destroy matrix\n");
	destroy_matrix(&tmp);
	
	return out;
}


/*******************************************************************************
* int quaternion_rotate(vector_t* q, vector_t tilt)
*
* This performs the same p'=qpq* quaternion rotation as quaternion_rotate_vector
* but applies it to a quaternion with a real part.
*******************************************************************************/
int quaternion_rotate(vector_t* p, vector_t q){
	vector_t conj, tmp, result;

	// sanity checks
	if(!q.initialized || !p->initialized){
		printf("ERROR: quaternion vector not initialized yet\n");
		return -1;
	}
	if(q.len!=4 || p->len!=4){
		printf("ERROR: quaternion must be a vector of length 4\n");
		return -1;
	}

	// compute p'=qpq*
	conj = quaternion_conjugate(q);
	tmp = quaternion_multiply(*p,conj);
	result = quaternion_multiply(q,tmp);

	// populate p with result
	p->data[0]=result.data[0];
	p->data[1]=result.data[1];
	p->data[2]=result.data[2];
	p->data[3]=result.data[3];

	// free memory
	destroy_vector(&conj);
	destroy_vector(&tmp);
	destroy_vector(&result);

	return 0;
}



/*******************************************************************************
* int quaternion_rotate_vector(vector_t* p, vector_t q)
*
* rotate a 3D vector v in-place about the origin by quaternion q
* q must be a unit quaternion for this to work properly
* this performs the operation p' = qpq*
* returns 0 on success, -1 on failure
*******************************************************************************/
int quaternion_rotate_vector(vector_t* p, vector_t q){
	vector_t pq, conj, tmp, result;

	// sanity checks
	if(!q.initialized){
		printf("ERROR: quaternion vector not initialized yet\n");
		return -1;
	}
	if(!p->initialized){
		printf("ERROR: vector not initialized yet\n");
		return -1;
	}
	if(q.len != 4){
		printf("ERROR: quaternion must be a vector of length 4\n");
		return -1;
	}
	if(p->len != 3){
		printf("ERROR: vector must be a vector of length 3\n");
		return -1;
	}

	// duplicate v into a quaternion with 0 real part
	pq = create_vector(4);
	pq.data[0]=0.0;
	pq.data[1]=p->data[0];
	pq.data[2]=p->data[1];
	pq.data[3]=p->data[2];

	// compute p'=qpq*
	printf("before conjugate\n");
	conj = quaternion_conjugate(q);
	printf("before multiply\n");
	tmp = quaternion_multiply(pq,conj);
	result = quaternion_multiply(q,tmp);
	printf("after multiply\n");

	// populate v with result
	p->data[0]=result.data[1];
	p->data[1]=result.data[2];
	p->data[2]=result.data[3];

	// free memory
	destroy_vector(&pq);
	destroy_vector(&conj);
	destroy_vector(&tmp);
	destroy_vector(&result);

	return 0;
}


/*******************************************************************************
* matrix_t quaternion_to_rotation_matrix(vector_t q)
*
* returns a 3x3 rotation matrix which performs the equivalent rotation
* as quaternion q when multiplied by a 3D vector
*******************************************************************************/
matrix_t quaternion_to_rotation_matrix(vector_t q){
	double q0s, q1s, q2s, q3s;
	matrix_t out;
	// sanity checks
	if(!q.initialized){
		printf("ERROR: quaternion vector not initialized yet\n");
		return empty_matrix();
	}
	if(q.len != 4){
		printf("ERROR: quaternion must be a vector of length 4\n");
		return empty_matrix();
	}

	// compute squares which will be used multiple times
	q0s = q.data[0]*q.data[0];
	q1s = q.data[1]*q.data[1];
	q2s = q.data[2]*q.data[2];
	q3s = q.data[3]*q.data[3];

	// compute diagonal entries
	out=create_matrix(3,3);
	out.data[0][0] = q0s+q1s-q2s-q3s;
	out.data[1][1] = q0s-q1s+q2s-q3s;
	out.data[2][2] = q0s-q1s-q2s+q3s;
	// compute upper triangle
	out.data[0][1] = 2.0 * (q.data[1]*q.data[2] - q.data[0]*q.data[3]);
	out.data[0][2] = 2.0 * (q.data[1]*q.data[3] + q.data[0]*q.data[2]);
	out.data[1][2] = 2.0 * (q.data[2]*q.data[3] - q.data[0]*q.data[1]);
	// mirror lower triangle
	out.data[1][0] = out.data[0][1];
	out.data[2][0] = out.data[0][2];
	out.data[2][1] = out.data[1][2];
	return out;
}