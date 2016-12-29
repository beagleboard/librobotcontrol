/*******************************************************************************
* linear_algebra.c
*
* James Strawson & Matt Atlas 2016
* Numerical Renaissance codebase used as reference for many algorithms
*******************************************************************************/

#include "../roboticscape.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h> // for memset

#define ZERO_TOLERANCE 1e-9 // consider v to be zero if fabs(v)<ZERO_TOLERANCE


/*******************************************************************************
* typedef struct givens_data_t
*
* the parameters for a fast givens rotation are packaged here for neatness
* if donothing==1 then there is nothing to be done in the rotation as g==0
*******************************************************************************/
typedef struct givens_data_t{
	int donothing;
	double a;
	double b;
	double gamma;
	double dnew0;
	double dnew1;
} givens_data_t;


/*******************************************************************************
* vector_t matrix_times_col_vec(matrix_t A, vector_t v)
*
* 
*******************************************************************************/
vector_t matrix_times_col_vec(matrix_t A, vector_t v){
	int i,j;
	vector_t out = empty_vector();
	if(!A.initialized || !v.initialized){
		printf("ERROR: matrix or vector not initialized yet\n");
		return out;
	}
	if(A.cols != v.len){
		printf("ERROR: dimensions do not match\n");
		return out;
	}
	out = create_vector(A.rows);
	for(i=0;i<A.rows;i++){
		for(j=0;j<A.cols;j++){	
			out.data[i] += v.data[j]*A.data[i][j];
		}
	}
	return out;
}

/*******************************************************************************
* vector_t row_vec_times_matrix(vector_t v, matrix_t A)
*
* 
*******************************************************************************/
vector_t row_vec_times_matrix(vector_t v, matrix_t A){
	int i,j;
	vector_t out = empty_vector();
	if(!A.initialized || !v.initialized){
		printf("ERROR: matrix or vector not initialized yet\n");
		return out;
	}
	if(A.rows != v.len){
		printf("ERROR: dimensions do not match\n");
		return out;
	}
	
	out = create_vector(A.cols);
	for(i=0;i<A.cols;i++){
		for(j=0;j<A.rows;j++){
			out.data[i] += v.data[j]*A.data[j][i];
		}
	}
	return out;
}


/*******************************************************************************
* double matrix_determinant(matrix_t A)
*
* 
*******************************************************************************/
double matrix_determinant(matrix_t A){
	int i,j,k;
	double ratio, det;
	matrix_t tmp;

	// sanity checks
	if(!A.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return -1;
	}
	if(A.rows != A.cols){
		printf("Error: cannot invert non-square matrix\n");
		return -1;
	}
	// shortcut for 1x1 matrix
	if(A.rows==1) return A.data[0][0];
	// shortcut for 2x2 matrix
	if(A.rows==2) return A.data[0][0]*A.data[1][1] - A.data[0][1]*A.data[1][0];

	tmp = duplicate_matrix(A);
	for(i=0;i<(A.rows-1);i++){
		for(j=i+1;j<A.rows;j++){
			ratio = tmp.data[j][i]/tmp.data[i][i];
			for(k=0;k<A.rows;k++){
				tmp.data[j][k] = tmp.data[j][k] - ratio * tmp.data[i][k];
			}
		}
	}
	det = 1.0; //storage for determinant
	for(i=0;i<A.rows;i++) det = det*tmp.data[i][i];

	destroy_matrix(&tmp);
	return det;  
}

/*******************************************************************************
* int LUP_decomposition(matrix_t A, matrix_t* L, matrix_t* U, matrix_t* P)
*
* LUP decomposition with partial pivoting  
*******************************************************************************/
int LUP_decomposition(matrix_t A, matrix_t* L, matrix_t* U, matrix_t* P){
	int i, j, k, m, index;
	double s1, s2, temp;
	m = A.cols;
	matrix_t Adup, Lt, Ut, Pt;
	if(!A.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return -1;
	}
	if(A.cols != A.rows){
		printf("ERROR: matrix is not square\n");
		return -1;
	}
	Adup = duplicate_matrix(A);
	Lt = create_identity_matrix(m);
	Ut = create_square_matrix(m);
	Pt = create_identity_matrix(m);
	for(i=0;i<m-1;i++){
		index = i;
		for(j=i;j<m;j++){
			if(fabs(A.data[j][i]) >= fabs(A.data[index][i])){
				index = j;
			}
		}
		if(index != i){
			for(j=0;j<m;j++){
				temp 				= Adup.data[index][j];
				Adup.data[index][j]	= Adup.data[i][j];
				Adup.data[i][j]		= temp;
				temp				= Pt.data[index][j];
				Pt.data[index][j]	= Pt.data[i][j];
				Pt.data[i][j]		= temp;	
			}
		}	
	}
	for(i=0;i<m;i++){
		for(j=0;j<m;j++){
			s1 = 0;
			s2 = 0;
			for(k=0;k<i;k++){
				s1 += Ut.data[k][j] * Lt.data[i][k];
			}
			for(k=0;k<j;k++){
				s2 += Ut.data[k][j] * Lt.data[i][k];
			}
			
			if(j>=i)	Ut.data[i][j] = Adup.data[i][j] - s1;
			
			if(i>=j)	Lt.data[i][j] = (Adup.data[i][j] - s2)/Ut.data[j][j];
		}
	}
	destroy_matrix(L);
	destroy_matrix(U);
	destroy_matrix(P);
	*L = Lt;
	*U = Ut;
	*P = Pt;
	destroy_matrix(&Adup);
	return 0;
}

/*******************************************************************************
* int qr_multiply_Q_right(matrix_t* A, matrix_t x)
*
* performs a right matrix multiplication of x on the bottom right minor of A 
* where x is smaller than or equal to the size of A
*******************************************************************************/
int qr_multiply_Q_right(matrix_t* A, matrix_t x){
	int i,j,k,q;
	double new;
	matrix_t tmp;
	if(!A->initialized){
		printf("ERROR: Matrix A not initialized yet\n");
		return -1;
	}
	if(!x.initialized){
		printf("ERROR: Matrix x not initialized yet\n");
		return -1;
	}
	if(A->cols<x.rows){
		printf("ERROR: multiply_matrix_minor dimension mismatch\n");
		return -1;
	}
	if(A->rows<x.cols){
		printf("ERROR: multiply_matrix_minor dimension mismatch\n");
		return -1;
	}
	// q is the number of columns of A left untouched because x is smaller
	q=A->cols-x.rows;
	// duplicate the subset of A to be operated on
	tmp = create_matrix_fast(A->rows, x.rows);
	for(i=0;i<A->rows;i++){
		for(j=0;j<x.rows;j++){
			tmp.data[i][j]=A->data[i][j+q];
		}
	}
	// do the multiplication, overwriting A
	for(i=0;i<A->rows;i++){
		// left q columns of A remain untouched 
		for(j=q;j<A->cols;j++){
			new = 0.0;
			for(k=0;k<(x.rows);k++){
				// do the matrix multiplication
				new += tmp.data[i][k]*x.data[k][j-q];
			}
			A->data[i][j]=new;
		}
	}
	// free up tmp memory
	destroy_matrix(&tmp);
	return 0;
}

/*******************************************************************************
* int qr_multiply_r_left(matrix_t x, matrix_t* Am, double norm)
*
* performs a left matrix multiplication of x on the bottom right minor of A 
* where x is smaller than or equal to the size of A
*******************************************************************************/
int qr_multiply_r_left(matrix_t x, matrix_t* A, double norm){
	int i,j,k,p;
	double new;
	matrix_t tmp;
	if(!A->initialized){
		printf("ERROR: Matrix A not initialized yet\n");
		return -1;
	}
	if(!x.initialized){
		printf("ERROR: Matrix x not initialized yet\n");
		return -1;
	}
	if(A->rows<x.cols){
		printf("ERROR: qr_multiply_r_left dimension mismatch\n");
		return -1;
	}
	// p is the index of A defining the top left corner of the operation
	// p=q=0 if x is same size as A'
	p=A->rows-x.cols;

	// duplicate the subset of A to be operated on
	tmp = create_matrix_fast(A->rows-p, A->cols-p);
	for(i=0;i<A->rows-p;i++){
		for(j=0;j<A->cols-p;j++){
			tmp.data[i][j]=A->data[i+p][j+p];
		}
	}
	// we know first column of A will be mostly zeros, so fill in zeros where
	// possible and multiply otherwise. overwrite A here
	for(i=0;i<(A->rows-p);i++){
		if(i==0)	A->data[i+p][p]=norm;
		else		A->data[i+p][p]=0.0;
		// do multiplication for the rest of the columns
		for(j=1;j<(A->cols-p);j++){
			new = 0.0;
			for(k=0;k<x.cols;k++){
				// do the matrix multiplication
				new += x.data[i][k]*tmp.data[k][j];
			}
			A->data[i+p][j+p]=new;
		}
	}
	// free up tmp memory
	destroy_matrix(&tmp);
	return 0;
}



/*******************************************************************************
* matrix_t qr_householder_matrix(vector_t x, double* new_norm)
*
* returns the householder reflection matrix for a given vector
* where u=x-ae1, v=u/norm(u), H=I-(2/norm(x))vv'
* warning! modifies x!, only for use by qr decomposition below
*******************************************************************************/
matrix_t qr_householder_matrix(vector_t x, double* new_norm){
	int i, j;
	double norm, tau, dot;
	matrix_t out;
	vector_t v;

	if(!x.initialized){
		printf("ERROR: vector not initialized yet\n");
		return empty_matrix();
	}
	// find the coefficient 
	// allocate memory for output matrix
	out = create_matrix_fast(x.len, x.len);
	v = create_vector(x.len);
	norm = vector_norm(x,2);

	// set sign of norm to opposite of the pivot to avoid loss of significance
	if(x.data[0]>=0.0){
		x.data[0]=(x.data[0]+norm);
		*new_norm = -norm;
	}
	else {
		x.data[0]=(x.data[0]-norm);
		*new_norm = norm;
	}
	// pre-calculate matrix multiplication coefficient
	// doing this on one line causes a compiler optimization error :-/
	dot = vector_dot_product(x,x);
	tau = -2.0/dot;
	// fill in diagonal and upper triangle of H
	for(i=0;i<v.len;i++){
		// H=I-(2/norm(x))vv' so add 1 on the diagonal
		out.data[i][i] = 1.0 + tau*x.data[i]*x.data[i];
		for(j=i+1;j<v.len;j++){
			out.data[i][j] = tau*x.data[i]*x.data[j];
		}
	}
	// copy to lower triangle
	for(i=1;i<v.len;i++){
		for(j=0;j<i;j++){
			out.data[i][j] = out.data[j][i];
		}
	}
	destroy_vector(&v);
	return out;
}

/*******************************************************************************
* int QR_decomposition(matrix_t A, matrix_t* Q, matrix_t* R)
*
* Using householder reflection method
*******************************************************************************/
int QR_decomposition(matrix_t A, matrix_t* Q, matrix_t* R){
	int i,j,steps;
	double norm;
	vector_t x;
	matrix_t H;

	// Sanity Checks
	if(!A.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return -1;
	}

	// make sure any previous memory allocated in q&r are freed
	destroy_matrix(Q);
	destroy_matrix(R);
	// start R as A, will fill in q later
	*R = duplicate_matrix(A);

	// find out how many householder reflections are necessary
	if(A.rows==A.cols) steps=A.cols-1;		// square
	else if(A.rows>A.cols) steps=A.cols;	// tall
	else steps=A.rows-1;					// wide

	// iterate through columns of A doing householder reflection to zero
	// the entries below the diagonal
	for(i=0;i<steps;i++){

		// take col of R from diag down
		x = create_vector(A.rows-i);
		for(j=i;j<A.rows;j++) x.data[j-i]=R->data[j][i];

		// get the ever-shrinking householder reflection for that column
		// qr_householder also fills in the norm of that column to 'norm'
		H = qr_householder_matrix(x, &norm);
		destroy_vector(&x);

		// left multiply R
		qr_multiply_r_left(H,R,norm);

		// on first loop, Q is just the householder matrix, otherwise 
		// right multiply and free the memory
		if(i==0){
			*Q = H;
		}
		else{
			qr_multiply_Q_right(Q,H);
			destroy_matrix(&H);
		}
	}
	return 0;
}

/*******************************************************************************
* matrix_t matrix_inverse(matrix_t A)
*
* Invert Matrix function based on LUP decomposition and then forward and
* backward substitution.
*******************************************************************************/
matrix_t matrix_inverse(matrix_t A){
	int i,j,k,m;
	matrix_t L,U,P,D,temp;
	matrix_t out = empty_matrix();
	if(!A.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return out;
	}
	if(A.cols != A.rows){
		printf("ERROR: matrix is not square\n");
		return out;
	}
	if(fabs(matrix_determinant(A)) < 1.0e-6){
		printf("ERROR: matrix is singular, not invertible\n");
		return out;
	}
	m = A.cols;
	LUP_decomposition(A,&L,&U,&P);
	D    = create_identity_matrix(m);
	temp = create_square_matrix(m);

	for(j=0;j<m;j++){
		for(i=0;i<m;i++){
			for(k=0;k<i;k++){
				D.data[i][j] -= L.data[i][k] * D.data[k][j];
			}
		}
		// backwards.. last to first
		for(i=m-1;i>=0;i--){
			temp.data[i][j] = D.data[i][j];
			for(k=i+1;k<m;k++){
				temp.data[i][j] -= U.data[i][k] * temp.data[k][j];
			}
			temp.data[i][j] = temp.data[i][j] / U.data[i][i];
		}
	}
	// multiply by permutation matrix
	out = multiply_matrices(temp, P);
	// free allocation	
	destroy_matrix(&temp);
	destroy_matrix(&L);
	destroy_matrix(&U);
	destroy_matrix(&P);
	destroy_matrix(&D);
	return out;
}


/*******************************************************************************
* vector_t lin_system_solve(matrix_t A, vector_t b)
*
* Returns the vector x that solves Ax=b
* Thank you to  Henry Guennadi Levkin for open sourcing this routine.
*******************************************************************************/
vector_t lin_system_solve(matrix_t A, vector_t b){
	double fMaxElem, fAcc;
	int nDim,i,j,k,m;
	vector_t xout = empty_vector();
	if(!A.initialized || !b.initialized){
		printf("ERROR: matrix or vector not initialized yet\n");
		return xout;
	}
	if(A.cols != b.len){
		printf("ERROR: matrix dimensions do not match\n");
		return xout;
	}
	
	nDim = A.cols;
	xout = create_vector(nDim);
	matrix_t Atemp = duplicate_matrix(A); // duplicate the given matrix 
	vector_t btemp = duplicate_vector(b); // duplicate the given vector
	
	for(k=0; k<(nDim-1); k++){ // base row of matrix
		// search of line with max element
		fMaxElem = fabs( Atemp.data[k][k]);
		m = k;
		for(i=k+1; i<nDim; i++){
			if(fMaxElem < fabs(Atemp.data[i][k])){
				fMaxElem = Atemp.data[i][k];
				m = i;
			}
		}
		// permutation of base line (index k) and max element line(index m)
		if(m != k){
			for(i=k; i<nDim; i++){
				fAcc = Atemp.data[k][i];
				Atemp.data[k][i] = Atemp.data[m][i];
				Atemp.data[m][i]  = fAcc;
			}
			fAcc = btemp.data[k];
			btemp.data[k] = btemp.data[m];
			btemp.data[m] = fAcc;
		}
		if(Atemp.data[k][k]  == 0.0) return xout; // needs improvement !!!
		// triangulation of matrix with coefficients
		for(j=(k+1); j<nDim; j++){ // current row of matrix
			fAcc = - Atemp.data[j][k]  / Atemp.data[k][k];
			for(i=k; i<nDim; i++){
				Atemp.data[j][i] = Atemp.data[j][i] + fAcc*Atemp.data[k][i] ;
			}
			// free member recalculation
			btemp.data[j] = btemp.data[j] + fAcc*btemp.data[k]; 
		}
	}

	for(k=(nDim-1); k>=0; k--){
		xout.data[k] = btemp.data[k];
		for(i=(k+1); i<nDim; i++){
			xout.data[k] -= (Atemp.data[k][i]*xout.data[i]);
		}
		xout.data[k] = xout.data[k] / Atemp.data[k][k];
	}

	destroy_matrix(&Atemp);
	destroy_vector(&btemp);
	return xout;
}

/*******************************************************************************
* vector_t lin_system_solve_qr(matrix_t A, vector_t b)
*
* Gives a least-squares solution to the system AX=b for non-square A using QR.
*
*  Ax=b
* QRx=b
*  Rx=Q'b  (because Q'Q=I)
*  then solve for x with gaussian elimination
*******************************************************************************/
vector_t lin_system_solve_qr(matrix_t A, vector_t b){
	vector_t xout = empty_vector();
	vector_t temp;
	matrix_t Q,R;
	int i,k;
	if(!A.initialized || !b.initialized){
		printf("ERROR: matrix or vector not initialized yet\n");
		return xout;
	}
	// do QR decomposition
	if(QR_decomposition(A,&Q,&R)<0){
		printf("failed to perform QR decomposition on A\n");
		return xout;
	}
	// transpose Q matrix
	if(transpose_matrix(&Q)<0){
		printf("ERROR: failed to transpose Q\n");
		return xout;
	}
	// multiply through
	temp = matrix_times_col_vec(Q,b);
	destroy_matrix(&Q);
	
	// solve for x knowing R is upper triangular
	int nDim = R.cols;
	xout = create_vector(nDim);
	for(k=(nDim-1); k>=0; k--){
		xout.data[k] = temp.data[k];
		for(i=(k+1); i<nDim; i++){
			xout.data[k] -= (R.data[k][i]*xout.data[i]);
		}
		xout.data[k] = xout.data[k] / R.data[k][k];
	}
	destroy_matrix(&R);
	destroy_vector(&temp);
	
	return xout;
}

/*******************************************************************************
* int fit_ellipsoid(matrix_t points, vector_t* center, vector_t* lengths)
*
* Fits an ellipsoid to a set of points in 3D space. The principle axes of the
* fitted ellipsoid align with the global coordinate system. Therefore there are
* 6 degrees of freedom defining the ellipsoid: the x,y,z coordinates of the
* centroid and the lengths from the centroid to the surfance in each of the 3
* directions. 
*
* matrix_t points is a tall matrix with 3 columns and at least 6 rows. Each row
* must contain the xy&z components of each individual point to be fit. If only 
* 6 rows are provided, the resulting ellipsoid will be an exact fit. Otherwise
* the result is a least-squares fit to the overdefined dataset.
*
* vector_t* center is a pointer to a user-created vector which will contain the
* x,y,z position of the centroid of the fit ellipsoid.
*
* vector_t* lengths is a pointer to a user-created vector which will be 
* populated with the 3 distances from the surface to the centroid in each of the 
* 3 directions.
*
* See Numerical Renaissance chapter 4 for algorithm 
*******************************************************************************/
int fit_ellipsoid(matrix_t points, vector_t* center, vector_t* lengths){
	int i,p;
	matrix_t A;
	vector_t b;
	if(!points.initialized){
		printf("ERROR: matrix_t points not initialized\n");
		return -1;
	}
	if(points.cols!=3){
		printf("ERROR: matrix_t points must have 3 columns\n");
		return -1;
	}
	p = points.rows;
	if(p<6){
		printf("ERROR: matrix_t points must have at least 6 rows\n");
		return -1;
	}
	
	b = create_vector_of_ones(p);
	A = create_matrix(p,6);
	for(i=0;i<p;i++){
		A.data[i][0] = points.data[i][0] * points.data[i][0];
		A.data[i][1] = points.data[i][0];
		A.data[i][2] = points.data[i][1] * points.data[i][1];
		A.data[i][3] = points.data[i][1];
		A.data[i][4] = points.data[i][2] * points.data[i][2];
		A.data[i][5] = points.data[i][2];
	}
	
	vector_t f = lin_system_solve_qr(A,b);
	destroy_matrix(&A);
	destroy_vector(&b);
	
	// compute center 
	*center = create_vector(3);
	center->data[0] = -f.data[1]/(2*f.data[0]);
	center->data[1] = -f.data[3]/(2*f.data[2]);
	center->data[2] = -f.data[5]/(2*f.data[4]);
	
	// Solve for lengths
	A = create_square_matrix(3);
	b = create_vector(3);
	
	// fill in A
	A.data[0][0] = (f.data[0] * center->data[0] * center->data[0]) + 1.0;
	A.data[0][1] = (f.data[0] * center->data[1] * center->data[1]);
	A.data[0][2] = (f.data[0] * center->data[2] * center->data[2]);
	
	A.data[1][0] = (f.data[2] * center->data[0] * center->data[0]);
	A.data[1][1] = (f.data[2] * center->data[1] * center->data[1]) + 1.0;
	A.data[1][2] = (f.data[2] * center->data[2] * center->data[2]);
	
	A.data[2][0] = (f.data[4] * center->data[0] * center->data[0]);
	A.data[2][1] = (f.data[4] * center->data[1] * center->data[1]);
	A.data[2][2] = (f.data[4] * center->data[2] * center->data[2]) + 1.0;
	
	// fill in b
	b.data[0] = f.data[0];
	b.data[1] = f.data[2];
	b.data[2] = f.data[4];

	// solve for lengths
	vector_t scales = lin_system_solve(A, b);
	
	*lengths = create_vector(3);
	lengths->data[0] = 1.0/sqrt(scales.data[0]);
	lengths->data[1] = 1.0/sqrt(scales.data[1]);
	lengths->data[2] = 1.0/sqrt(scales.data[2]);
	// cleanup
	destroy_vector(&scales);
	destroy_matrix(&A);
	destroy_vector(&b);
	return 0;
}

/*******************************************************************************
* givens_data_t fast_givens_compute(double f, double g, double di, double dk)
*
* Compute the parameters {a,b,gamma,donothing} of a Fast Givens transformation 
* matrix designed to transform the vector (f;g) to (*;0).
* returns 1 if there is nothing to do (g is already 0)
* return 0 is all went well
*
* See Numerical Renaissance chapter 4 for algorithm 
*******************************************************************************/
// givens_data_t fast_givens_compute(double f, double g, double di, double dk){
// 	givens_data_t out;
// 	// if g is sufficiently small, rotation has nothing to do
// 	if(fabs(g)<ZERO_TOLERANCE){
// 		out.donothing = 1;
// 		return out;
// 	}
// 	out.donothing = 0;
// 	out.a = -f/g;
// 	out.b = - *a * dk/di;
// 	out.gamma = -*a * *b;
// 	if(out.gamma<=1.0){
// 		out.dnew0=(1.0 + *gamma)*dk;
// 		out.dnew1=(1.0 + *gamma)*di;
// 	}
// 	else{
// 		out.dnew0=(1.0+(1.0 / *gamma)) * di;
// 		out.dnew1=(1.0+(1.0 / *gamma)) * dK;
// 		out.a = 1.0 / out.a;
// 		out.b = 1.0 / out.b;
// 	}
// 	return out;
// }


/*******************************************************************************
* int fast_givens(matrix_t* X, givens_data_t gd, double i, double k, double p,
												double q, char c)
*
* performs fast givens transformation on X in-place with givens_data_t gd 
* obtained from fast_givens_compute(). 
* c is a character that can be 'L', 'R', or 'B' indicating do perform a
* left premultiply by F^H, a right postmultiply by F, or to do both.
*
* See Numerical Renaissance chapter 4 for algorithm 
*******************************************************************************/
// int fast_givens(matrix_t* X, givens_data_t gd, double i, double k, double p,
// 												double q, char c){
// 	int j;
// 	matrix_t tmp;

// 	// nothing to do!!
// 	if(gd.donothing) return 0;

// 	// sanity checks
// 	if(!X->initialized){
// 		printf("ERROR: trying to do fast_givens on uninitialized matrix\n");
// 		return -1;
// 	}
// 	if(q>=p){
// 		printf("ERROR: q must be greater than p\n");
// 		return -1;
// 	}
// 	if(c!='L' && c!='R' && c!='B'){
// 		printf("ERROR: c must be L R or B\n");
// 		return -1;
// 	}

// 	// do left-multiply if requested (or both are requested)
// 	if(c=='L' || c=='B'){
// 		if(i>X->rows || i<0){
// 			printf("ERROR: i out of bounds\n");
// 			return -1;
// 		}
// 		if(k>X->rows || K<0){
// 			printf("ERROR: k out of bounds\n");
// 			return -1;
// 		}
// 		if(p>X->cols || p<0){
// 			printf("ERROR: p out of bounds\n");
// 			return -1;
// 		}
// 		if(q>X->cols || q<0){
// 			printf("ERROR: q out of bounds\n");
// 			return -1;
// 		}
// 		// first make tmp matrix to hold original contents of X
// 		tmp = create_matrix(2,q=p+1);
// 		for(j=0;j<=q-p;j++){
// 			tmp.data[0][j]=X->data[i][j+p];
// 			tmp.data[1][j]=X->data[k][j+p];
// 		}
// 		if(gamma<=1.0){
// 			// for row i, then row k, operate on columns p through q
// 			for(j=0;j<=q-p;j++){
// 				X->data[i][j+p]=gd.b*tmp.data[0][j]+tmp.data[1][j];
// 				X->data[k][j+p]=tmp.data[0][j]+a*tmp.data[1][j];
// 			}
// 		}
// 		else{
// 			for(j=0;j<=q-p;j++){
// 				X->data[i][j+p]=tmp.data[0][j]+gd.b*tmp.data[1][j];
// 				X->data[k][j+p]=a*tmp.data[0][j]+tmp.data[1][j];
// 			}
// 		}
// 		destroy_matrix(*tmp);
// 	}

// 	// do right-multiply if requested (or both are requested)
// 	if(c=='R' || c=='B'){
// 		if(i>X->cols || i<0){
// 			printf("ERROR: i out of bounds\n");
// 			return -1;
// 		}
// 		if(k>X.cols || k<0){
// 			printf("ERROR: k out of bounds\n");
// 			return -1;
// 		}
// 		if(p>X.rows || p<0){
// 			printf("ERROR: p out of bounds\n");
// 			return -1;
// 		}
// 		if(q>X.rows || p<0){
// 			printf("ERROR: q out of bounds\n");
// 			return -1;
// 		}
// 		// first make tmp matrix to hold original contents of X
// 		tmp = create_matrix(q=p+1,2);
// 		for(j=0;j<=q-p;j++){
// 			tmp.data[j][0]=X->data[j+p][i];
// 			tmp.data[j][1]=X->data[j+p][k];
// 		}
// 		if(gamma<=1.0){
// 			// for row i, then row k, operate on columns p through q
// 			for(j=0;j<=q-p;j++){
// 				X->data[j+p][i]=gd.b*tmp.data[j][0]+tmp.data[j][1];
// 				X->data[j+p][k]=tmp.data[j][0]+a*tmp.data[j][1];
// 			}
// 		}
// 		else{
// 			for(j=0;j<=q-p;j++){
// 				X->data[j+p][i]=tmp.data[j][0]+gd.b*tmp.data[j][1];
// 				X->data[j+p][k]=a*tmp.data[j][0]+tmp.data[j][1];
// 			}
// 		}
// 		destroy_matrix(*tmp);
// 	}
// 	return 0;
// }



/*******************************************************************************
* int diopohantine(vector_t a, vector_t b, vector_t c, \
					vector_t* x, vector_t* y, vector_t* r, vector_t* s)
*
* Solve the polynomial Diophantine eqn a*x+b*y=c via the Extended Euclidean 
* algorithm for coprime {a,b}. The solution {x,y} returned is the solution with 
* the lowest order for y
* the general solution is given by {x+r*t,y+s*t} for any polynomial t.
* Refer to Numerical Renaissance for background and uses
*******************************************************************************/
/*
int diopohantine(vector_t a, vector_t b, vector_t c, \
					vector_t* x, vector_t* y, vector_t* r, vector_t* s){

	// sanity checks
	if(!a.initialized){
		printf("ERROR: a not initialized yet\n");
		return empty_vector();
	}
	if(!b.initialized){
		printf("ERROR: b not initialized yet\n");
		return empty_vector();
	}
	if(!c.initialized){
		printf("ERROR: c not initialized yet\n");
		return empty_vector();
	}


	n=max(2,abs(length(a)-length(b))+1); 
rm=a; 
r=b; 

for i=1:n+2
	r=r(find(r,1):end);
	[quo,rem]=PolyDiv(rm,r);  // Reduce (rm,r) to their GCD via Euclid's
	q(i,n+1-length(quo):n)=quo;
	rm=r; 
	r=rem;      % algorithm, saving the quotients quo.
	if norm(r,inf)<1e-13, 
		g=rm, 
		break, 
		end, 
	end
	r=-PolyDiv(b,g); 
	s=PolyDiv(a,g); 
	y=PolyDiv(c,g);
	x=0;  
	for j=i-1:-1:1                            % Using q as computed above, compute {x,y}
		t=x; 
		x=y; 
		y=PolyAdd(t,-PolyConv(q(j,:),y));   % via the Extended Euclidean algorithm
	end, 
	y=y(find(y,1):end); 
	[div,rem]=PolyDiv(y,s), 
	t=-div   % Find the solution {x,y} that
	x=PolyAdd(x,PolyConv(r,t)); 
	x=x(find(abs(x)>1e-8,1):end); % minimizes the order of y; this
	y=PolyAdd(y,PolyConv(s,t)); 
	y=y(find(abs(y)>1e-8,1):end); % is the most useful in practice
end % function Diophantine


} */