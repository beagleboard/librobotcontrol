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

#define PI (double)M_PI




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
	if(!A.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return -1;
	}
	if (A.rows != A.cols){
		printf("Error: Matrix is not square\n");
		return -1;
	}
	matrix_t temp = duplicate_matrix(A);
	for(i=0;i<A.rows;i++){
        for(j=0;j<A.rows;j++){
            if(j>i){
				ratio = temp.data[j][i]/temp.data[i][i];
                for(k=0;k<A.rows;k++){
                    temp.data[j][k] = temp.data[j][k] - ratio * temp.data[i][k];
                }
            }
        }
    }
	det = 1; //storage for determinant
    for(i=0;i<A.rows;i++) det = det*temp.data[i][i];

	destroy_matrix(&temp);
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
	destroy_matrix(L);
	destroy_matrix(U);
	destroy_matrix(P);
	matrix_t Lt, Ut, Pt;
	if(!A.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return -1;
	}
	if(A.cols != A.rows){
		printf("ERROR: matrix is not square\n");
		return -1;
	}
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
				temp 				= A.data[index][j];
				A.data[index][j] 	= A.data[i][j];
				A.data[i][j]		= temp;
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
			
			if(j>=i)	Ut.data[i][j] = A.data[i][j] - s1;
			
			if(i>=j)	Lt.data[i][j] = (A.data[i][j] - s2)/Ut.data[j][j];	
		}
	}
	*L = Lt;
	*U = Ut;
	*P = Pt;
	return 0;
}

/*******************************************************************************
* int QR_decomposition(matrix_t A, matrix_t* Q, matrix_t* R)
*
* 
*******************************************************************************/
int QR_decomposition(matrix_t A, matrix_t* Q, matrix_t* R){
	int i, j, k, s;
	int m = A.rows;
	int n = A.cols;
	vector_t xtemp;
	matrix_t Qt, Rt, Qi, F, temp;
	
	if(!A.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return -1;
	}
	
	destroy_matrix(Q);
	destroy_matrix(R);
	
	Qt = create_matrix(m,m);
	for(i=0;i<m;i++){					// initialize Qt as I
		Qt.data[i][i] = 1;
	}
	
	Rt = duplicate_matrix(A);			// duplicate A to Rt

	for(i=0;i<n;i++){					// iterate through columns of A
		xtemp = create_vector(m-i);		// allocate length, decreases with i
		
		for(j=i;j<m;j++){						// take col of -R from diag down
			xtemp.data[j-i] = -Rt.data[j][i]; 	
		}
		if(Rt.data[i][i] > 0)	s = -1;			// check the sign
		else					s = 1;
		xtemp.data[0] += s*vector_norm(xtemp, 2);	// add norm to 1st element
		
		Qi = create_square_matrix(m);			// initialize Qi
		F  = create_square_matrix(m-i);			// initialize shrinking householder_matrix
		F  = householder_matrix(xtemp);			// fill in Househodor
		
		for(j=0;j<i;j++){
			Qi.data[j][j] = 1;				// fill in partial I matrix
		}
		for(j=i;j<m;j++){					// fill in remainder (householder_matrix)
			for(k=i;k<m;k++){
				Qi.data[j][k] = F.data[j-i][k-i];
			}
		}
		// multiply new Qi to old Qtemp
		temp = duplicate_matrix(Qt);
		destroy_matrix(&Qt);
		Qt = multiply_matrices(Qi,temp);
		destroy_matrix(&temp);
		
		// same with Rtemp
		temp = duplicate_matrix(Rt);
		destroy_matrix(&Rt);
		Rt = multiply_matrices(Qi,temp);
		destroy_matrix(&temp);
		
		// free other allocation used in this step
		destroy_matrix(&Qi);					
		destroy_matrix(&F);
		destroy_vector(&xtemp);
	}
	transpose_matrix(&Qt);
	*Q = Qt;
	*R = Rt;
	return 0;
}

/*******************************************************************************
* matrix_t invert_matrix(matrix_t A)
*
* Invert Matrix function based on LUP decomposition and then forward and
* backward substitution.
*******************************************************************************/
matrix_t invert_matrix(matrix_t A){
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
	if(matrix_determinant(A) == 0){
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
		for(i=m-1;i>=0;i--){				// backwards.. last to first
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
* matrix_t householder_matrix(vector_t v)
*
* returns the householder reflection matrix for a given vector
*******************************************************************************/
matrix_t householder_matrix(vector_t v){
	int i, j;
	double tau;
	matrix_t out = empty_matrix();
	if(!v.initialized){
		printf("ERROR: vector not initialized yet\n");
		return out;
	}
	out = create_square_matrix(v.len);
	for(i=0;i<v.len;i++){
		out.data[i][i] = 1;
	}
	tau = 2.0/vector_dot_product(v,v);
	for(i=0;i<v.len;i++){
		for(j=0;j<v.len;j++){
			out.data[i][j] -= tau * v.data[i]*v.data[j];
		}
	}
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