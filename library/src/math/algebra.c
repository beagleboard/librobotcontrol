/**
 * @file math/algebra.c
 *
 * @brief      Collection of advanced linear algebra functions
 *
 * @author     James Strawson
 * @date       2016
 *
 */

#include <stdio.h>
#include <stdlib.h>	// for malloc,calloc,free
#include <math.h>	// for sqrt, pow, etc
#include <string.h>	// for memcpy

#include <rc/math/vector.h>
#include <rc/math/matrix.h>
#include <rc/math/algebra.h>
#include "algebra_common.h"



#define DEFAULT_ZERO_TOLERANCE 1e-8 // consider v to be zero if fabs(v)<ZERO_TOLERANCE

// current tolerance, can be changed with rc_algebra_set_zero_tolerance.
double zero_tolerance=DEFAULT_ZERO_TOLERANCE;


// used by QR decomposition
static int __householder_reflection(int step, rc_matrix_t* Q, rc_matrix_t* R)
{
	int i,j,k;
	double norm, tau, taui, dot;
	int n = R->rows-step;
	int Rrows = R->rows;
	// p is the index of A defining the top left corner of the operation
	// p=q=0 if x is same size as A',
	int p=R->rows-n;
	// x will hold each column of R from diag down at each step
	double x[n];
	// H will be the ever-shrinking householder matrix
	//NEW_STACK_MATRIX(H,R->rows-step,R->rows-step);
	double H[n][n];
	// tmp is a duplicate of the the subset of A to be operated on when
	// left-multiplying R
	double tmp[R->cols-p][R->rows-p];
	// duplicate pf the subset of Q to be operated on when right-multiplying R
	double tmp2[Q->rows][n];
	// allocate memory for a column of tmp2
	double col[n];
	// q is the number of columns of Q left untouched because H is smaller
	int q = Q->cols-n;


	////////////////////////////////////////////////////////////////////////
	// get the ever-shrinking householder reflection for that column
	////////////////////////////////////////////////////////////////////////

	// take col of R from diag down
	for(j=step;j<Rrows;j++) x[j-step]=R->d[j][step];

	// find norm of x
	norm = 0.0;
	for(i=0;i<n;i++) norm += x[i]*x[i];
	norm=sqrt(norm);

	// set sign of norm to opposite of the pivot to avoid loss of significance
	if(x[0]>=0.0){
		x[0]=(x[0]+norm);
		norm = -norm;
	}
	else x[0]=(x[0]-norm);


	// pre-calculate matrix multiplication coefficient tau
	// doing this on one line causes a compiler optimization error :-/
	dot = __vectorized_square_accumulate(x,n);
	tau = -2.0/dot;

	// fill in diagonal and upper triangle of H
	for(i=0;i<n;i++){
		taui = tau*x[i];
		// H=I-(2/norm(x))vv' so add 1 on the diagonal
		H[i][i] = 1.0 + taui*x[i];
		for(j=i+1;j<n;j++){
			H[i][j] = taui*x[j];
		}
	}

	// copy to lower triangle
	for(i=1;i<n;i++){
		for(j=0;j<i;j++){
			H[i][j] = H[j][i];
		}
	}

	////////////////////////////////////////////////////////////////////////
	// left multiply R
	////////////////////////////////////////////////////////////////////////

	// Copy Section of R to be operated on
	// store it in transpose form so memory access later is contiguous
	for(i=0;i<R->rows-p;i++){
		for(j=0;j<R->cols-p;j++){
			tmp[j][i]=R->d[i+p][j+p];
		}
	}
	// go through the rows of R starting from the first row that requires
	// modifying. as H shrinks, only the lower rows need modifying
	for(i=0;i<(R->rows-p);i++){
		// we know first column of R will be mostly zeros, so fill in zeros
		// or known norm where possible. we use p to
		if(i==0)	R->d[i+p][p]=norm;
		else		R->d[i+p][p]=0.0;
		// do multiplication for the rest of the columns
		// A has already been transposed so don't transpose each column
		for(j=1;j<(R->cols-p);j++){
			R->d[i+p][j+p]=__vectorized_mult_accumulate(H[i],tmp[j],n);
		}
	}

	////////////////////////////////////////////////////////////////////////
	// right multiply Q by H
	////////////////////////////////////////////////////////////////////////

	// duplicate the subset of Q to be operated on
	for(i=0;i<Q->rows;i++){
		for(j=0;j<n;j++)	tmp2[i][j]=Q->d[i][j+q];
	}

	// do the multiplication, overwriting A left q columns of A remain untouched
	for(j=0;j<(Q->cols-q);j++){
		// put column of x in sequential memory slot
		for(k=0;k<n;k++) col[k]=H[k][j];
		// now go down the i'th column of A
		// H is hermetian so don't bother transposing its columns
		for(i=0;i<Q->rows;i++){
			Q->d[i][j+q]=__vectorized_mult_accumulate(tmp2[i],col,n);
		}
	}

	return 0;
}

int rc_algebra_qr_decomp(rc_matrix_t A, rc_matrix_t* Q, rc_matrix_t* R)
{
	int i,steps;

	// Sanity Checks
	if(unlikely(!A.initialized)){
		fprintf(stderr,"ERROR in rc_algebra_qr_decomp, matrix not initialized yet\n");
		return -1;
	}
	// start R as A
	if(unlikely(rc_matrix_duplicate(A,R))){
		fprintf(stderr,"ERROR in rc_algebra_qr_decomp, failed to duplicate A\n");
		return -1;
	}
	// start R as square identity
	rc_matrix_identity(Q,A.rows);
	// find out how many householder reflections are necessary
	if(A.rows==A.cols) steps=A.cols-1;	// square
	else if(A.rows>A.cols) steps=A.cols;	// tall
	else steps=A.rows-1;			// wide

	// iterate through columns of A doing householder reflection to zero
	// the entries below the diagonal
	for(i=0;i<steps;i++){
		if(__householder_reflection(i,Q,R)==-1) return -1;
	}
	return 0;
}

int rc_algebra_lup_decomp(rc_matrix_t A, rc_matrix_t* L, rc_matrix_t* U, rc_matrix_t* P)
{
	int i,j,k,m,index,tmpint;
	double s1, s2;
	int* ptmp;
	void* rowtmp;
	rc_matrix_t Adup = RC_MATRIX_INITIALIZER;
	// sanity checks
	if(unlikely(!A.initialized)){
		fprintf(stderr,"ERROR in rc_algebra_lup_decomp, matrix not initialized yet\n");
		return -1;
	}
	if(unlikely(A.cols!=A.rows)){
		fprintf(stderr,"ERROR in rc_algebra_lup_decomp, matrix is not square\n");
		return -1;
	}
	// allocate some memory!
	m = A.cols;
	if(unlikely(rc_matrix_duplicate(A, &Adup))){
		fprintf(stderr,"ERROR in rc_algebra_lup_decomp, failed to duplicate A\n");
		return -1;
	}
	if(unlikely(rc_matrix_identity(L,m))){
		fprintf(stderr,"ERROR in rc_algebra_lup_decomp, failed to allocate identity matrix\n");
		rc_matrix_free(&Adup);
		return -1;
	}
	if(unlikely(rc_matrix_alloc(U,m,m))){
		fprintf(stderr,"ERROR in rc_algebra_lup_decomp, failed to allocate U\n");
		rc_matrix_free(&Adup);
		rc_matrix_free(L);
		return -1;
	}
	if(unlikely(rc_matrix_zeros(P,m,m))){
		fprintf(stderr,"ERROR in rc_algebra_lup_decomp, failed to allocate matrix of zeros\n");
		rc_matrix_free(&Adup);
		rc_matrix_free(L);
		rc_matrix_free(U);
		return -1;
	}
	// represent P as an array of positions 0 through (m-1) for fast pivoting
	// also alloc memory for a row of A as tmp holder while pivoting
	ptmp = alloca(m*sizeof(int));
	rowtmp = alloca(m*sizeof(double));
	if(unlikely(ptmp==NULL || rowtmp==NULL)){
		fprintf(stderr,"ERROR in rc_algebra_lup_decomp, alloca failed, stack overflow\n");
		rc_matrix_free(&Adup);
		rc_matrix_free(L);
		rc_matrix_free(U);
		rc_matrix_free(P);
		return -1;
	}
	// make ptmp where each value contains the column position of the 1 in it's
	// initial identity matrix form
	for(i=0;i<m;i++) ptmp[i]=i;
	// now do the pivoting
	for(i=0;i<m-1;i++){
		index = i;
		for(j=i;j<m;j++){
			if(fabs(A.d[j][i])>=fabs(A.d[index][i]))	index=j;
		}
		if(index!=i){
			// swap rows in ptmp
			tmpint = ptmp[index];
			ptmp[index]=ptmp[i];
			ptmp[i]=tmpint;
			// swap rows of A
			memcpy(rowtmp,Adup.d[index],m*sizeof(double));
			memcpy(Adup.d[index],Adup.d[i],m*sizeof(double));
			memcpy(Adup.d[i],rowtmp,m*sizeof(double));
		}
	}
	// construct P from ptmp
	for(i=0;i<m;i++) P->d[i][ptmp[i]]=1.0;
	// now do normal LU
	for(i=0;i<m;i++){
		for(j=0;j<m;j++){
			s1 = 0.0;
			s2 = 0.0;
			for(k=0;k<i;k++) s1 += U->d[k][j] * L->d[i][k];
			for(k=0;k<j;k++) s2 += U->d[k][j] * L->d[i][k];
			if(j>=i) U->d[i][j] = (Adup.d[i][j]-s1);
			if(i>=j) L->d[i][j] = (Adup.d[i][j]-s2)/U->d[j][j];
		}
	}
	rc_matrix_free(&Adup);
	return 0;
}

int rc_algebra_invert_matrix(rc_matrix_t A, rc_matrix_t* Ainv)
{
	int i,j,k;
	rc_matrix_t L = RC_MATRIX_INITIALIZER;
	rc_matrix_t U = RC_MATRIX_INITIALIZER;
	rc_matrix_t P = RC_MATRIX_INITIALIZER;
	rc_matrix_t D = RC_MATRIX_INITIALIZER;
	rc_matrix_t tmp = RC_MATRIX_INITIALIZER;
	// sanity checks
	if(unlikely(!A.initialized)){
		fprintf(stderr,"ERROR in rc_matrix_inverse, matrix uninitialized\n");
		return -1;
	}
	if(unlikely(A.cols!=A.rows)){
		fprintf(stderr,"ERROR in rc_matrix_inverse, nonsquare matrix\n");
		return -1;
	}
	if(fabs(rc_matrix_determinant(A)) < zero_tolerance){
		fprintf(stderr,"ERROR in rc_matrix_inverse, matrix is singular\n");
		return -1;
	}
	// allocate memory
	if(unlikely(rc_matrix_identity(&D,A.cols))){
		fprintf(stderr,"ERROR in rc_matrix_inverse, failed to alloc identity\n");
		return -1;
	}
	if(unlikely(rc_matrix_alloc(&tmp,A.rows,A.rows))){
		fprintf(stderr,"ERROR in rc_matrix_inverse, failed to alloc matrix\n");
		rc_matrix_free(&D);
		return -1;
	}
	// do LUP
	if(unlikely(rc_algebra_lup_decomp(A,&L,&U,&P))){
		fprintf(stderr,"ERROR in rc_matrix_inverse, failed to LUP decomp\n");
		rc_matrix_free(&D);
		rc_matrix_free(&tmp);
		return -1;
	}
	// solve for Inv
	for(j=0;j<A.cols;j++){
		for(i=0;i<A.cols;i++){
			for(k=0;k<i;k++){
				D.d[i][j] -= L.d[i][k] * D.d[k][j];
			}
		}
		// backwards.. last to first
		for(i=A.cols-1;i>=0;i--){
			tmp.d[i][j] = D.d[i][j];
			for(k=i+1;k<A.cols;k++){
				tmp.d[i][j] -= U.d[i][k] * tmp.d[k][j];
			}
			tmp.d[i][j] = tmp.d[i][j] / U.d[i][i];
		}
	}
	// free up some memory
	rc_matrix_free(&L);
	rc_matrix_free(&U);
	rc_matrix_free(&D);
	// use i as new return value
	i=0;
	// multiply by permutation matrix
	if(unlikely(rc_matrix_multiply(tmp, P, Ainv))){
		fprintf(stderr,"ERROR in rc_matrix_inverse, failed to multiply matrix\n");
		i=-1;
	}
	// free allocation
	rc_matrix_free(&tmp);
	rc_matrix_free(&P);
	return i;
}


int rc_algebra_invert_matrix_inplace(rc_matrix_t* A)
{
	rc_matrix_t Atmp = RC_MATRIX_INITIALIZER;
	if(unlikely(rc_algebra_invert_matrix(*A,&Atmp))){
		fprintf(stderr, "ERROR in rc_algebra_invert_matrix_inplace, failed to invert\n");
		return -1;
	}
	// free original memory and copy new matrix into place
	rc_matrix_free(A);
	*A = Atmp;
	return 0;
}


int rc_algebra_lin_system_solve(rc_matrix_t A, rc_vector_t b, rc_vector_t* x)
{
	/*Thank you to Henry Guennadi Levkin for open sourcing this routine, it's
	* adapted here and includes better detection of unsolvable systems.
	*/
	double fMaxElem, fAcc;
	int nDim,i,j,k,m;
	rc_matrix_t Atemp = RC_MATRIX_INITIALIZER;
	rc_vector_t btemp = RC_VECTOR_INITIALIZER;
	// sanity checks
	if(!A.initialized || !b.initialized){
		fprintf(stderr,"ERROR in rc_algebra_lin_system_solve, matrix or vector uninitialized\n");
		return -1;
	}
	if(A.cols != b.len){
		fprintf(stderr,"ERROR in rc_algebra_lin_system_solve, dimension mismatch\n");
		return -1;
	}
	// alloc memory for x
	nDim = A.cols;
	if(unlikely(rc_vector_alloc(x,nDim))){
		fprintf(stderr,"ERROR in rc_algebra_lin_system_solve, failed to alloc vector\n");
		return -1;
	}
	// duplicate user arguments so we don't have to modify them
	if(unlikely(rc_matrix_duplicate(A, &Atemp))){
		fprintf(stderr,"ERROR in rc_algebra_lin_system_solve, failed to duplicate matrix\n");
		rc_vector_free(x);
		return -1;
	}
	if(unlikely(rc_vector_duplicate(b, &btemp))){
		fprintf(stderr,"ERROR in rc_algebra_lin_system_solve, failed to duplicate vector\n");
		rc_vector_free(x);
		rc_matrix_free(&Atemp);
		return -1;
	}
	// gaussian elemination
	for(k=0;k<(nDim-1);k++){ // base row of matrix
		// search of line with max element
		fMaxElem=fabs(Atemp.d[k][k]);
		m=k;
		for(i=k+1;i<nDim;i++){
			if(fMaxElem<fabs(Atemp.d[i][k])){
				fMaxElem=Atemp.d[i][k];
				m=i;
			}
		}
		// permutation of base line (index k) and max element line(index m)
		if(m!=k){
			for(i=k;i<nDim;i++){
				fAcc=Atemp.d[k][i];
				Atemp.d[k][i]=Atemp.d[m][i];
				Atemp.d[m][i]=fAcc;
			}
			fAcc=btemp.d[k];
			btemp.d[k]=btemp.d[m];
			btemp.d[m]=fAcc;
		}
		// check if we got 0 on the diagonal indicating matrix isn't full rank
		if(unlikely(fabs(Atemp.d[k][k])<zero_tolerance)){
			fprintf(stderr,"ERROR in rc_algebra_lin_system_solve, matrix not full rank\n");
			rc_matrix_free(&Atemp);
			rc_vector_free(&btemp);
			rc_vector_free(x);
			return -1;
		}
		// triangulation of matrix with coefficients
		for(j=(k+1);j<nDim;j++){ // current row of matrix
			fAcc = -Atemp.d[j][k]/Atemp.d[k][k];
			for(i=k;i<nDim;i++){
				Atemp.d[j][i]=Atemp.d[j][i]+fAcc*Atemp.d[k][i];
			}
			// free member recalculation
			btemp.d[j] = btemp.d[j] + (fAcc*btemp.d[k]);
		}
	}
	// now run up the upper diagonal matrix solving for x
	for(k=nDim-1;k>=0;k--){
		x->d[k]=btemp.d[k];
		for(i=k+1;i<nDim;i++) x->d[k]-=Atemp.d[k][i]*x->d[i];
		x->d[k]=x->d[k]/Atemp.d[k][k];
	}
	// free memory
	rc_matrix_free(&Atemp);
	rc_vector_free(&btemp);
	return 0;
}

void rc_algebra_set_zero_tolerance(double tol){
	zero_tolerance=tol;
	return;
}

int rc_algebra_lin_system_solve_qr(rc_matrix_t A, rc_vector_t b, rc_vector_t* x)
{
	int i,k;
	rc_vector_t temp = RC_VECTOR_INITIALIZER;
	rc_matrix_t Q = RC_MATRIX_INITIALIZER;
	rc_matrix_t R = RC_MATRIX_INITIALIZER;
	if(unlikely(!A.initialized || !b.initialized)){
		fprintf(stderr,"ERROR in rc_algebra_lin_system_solve_qr, matrix or vector uninitialized\n");
		return -1;
	}
	// do QR decomposition
	if(unlikely(rc_algebra_qr_decomp(A,&Q,&R))){
		fprintf(stderr,"ERROR in rc_algebra_lin_system_solve_qr, failed to perform QR decomp\n");
		return -1;
	}
	// Ax=b
	// QRx=b
	// Rx=Q'b	because Q'Q=I
	// RX=(b'Q)'	to avoid transposing q
	// multiply through right hand side. No difference between row and col
	// vector so avoid transposing Q by left instead of right multiplying
	if(unlikely(rc_matrix_row_vec_times_matrix(b,Q,&temp))){
		fprintf(stderr,"ERROR in rc_algebra_lin_system_solve_qr, failed to multiply vec by matrix\n");
		rc_matrix_free(&Q);
		rc_matrix_free(&R);
		return -1;
	}
	// allocate memory for the output x
	if(unlikely(rc_vector_alloc(x,R.cols))){
		fprintf(stderr,"ERROR in rc_algebra_lin_system_solve_qr, failed to alloc vector\n");
		rc_matrix_free(&Q);
		rc_matrix_free(&R);
		rc_vector_free(&temp);
		return -1;
	}
	// solve for x knowing R is upper triangular
	for(k=R.cols-1;k>=0;k--){
		x->d[k]=temp.d[k];
		for(i=k+1;i<R.cols;i++)	x->d[k]-=R.d[k][i]*x->d[i];
		x->d[k] = x->d[k]/R.d[k][k];
	}
	// free memory and return
	rc_matrix_free(&Q);
	rc_matrix_free(&R);
	rc_vector_free(&temp);
	return 0;
}


int rc_algebra_fit_ellipsoid(rc_matrix_t pts, rc_vector_t* ctr, rc_vector_t* lens)
{
	int i,p;
	rc_matrix_t A = RC_MATRIX_INITIALIZER;
	rc_vector_t b = RC_VECTOR_INITIALIZER;
	rc_vector_t f = RC_VECTOR_INITIALIZER;
	// sanity checks
	if(unlikely(!pts.initialized)){
		fprintf(stderr,"ERROR in rc_fit_ellipsoid, matrix not initialized\n");
		return -1;
	}
	if(unlikely(pts.cols!=3)){
		fprintf(stderr,"ERROR in rc_fit_ellipsoid, matrix pts must have 3 columns\n");
		return -1;
	}
	p = pts.rows;
	if(p<6){
		fprintf(stderr,"ERROR in rc_fit_ellipsoid, matrix pts must have at least 6 rows\n");
		return -1;
	}
	// allocate memory for linear system
	if(unlikely(rc_vector_ones(&b,p))){
		fprintf(stderr,"ERROR in rc_fit_ellipsoid, failed to alloc vector\n");
		return -1;
	}
	if(unlikely(rc_matrix_alloc(&A,p,6))){
		fprintf(stderr,"ERROR in rc_fit_ellipsoid, failed to alloc matrix\n");
		rc_vector_free(&b);
		return -1;
	}
	// fill in A for QR
	for(i=0;i<p;i++){
		A.d[i][0] = pts.d[i][0] * pts.d[i][0];
		A.d[i][1] = pts.d[i][0];
		A.d[i][2] = pts.d[i][1] * pts.d[i][1];
		A.d[i][3] = pts.d[i][1];
		A.d[i][4] = pts.d[i][2] * pts.d[i][2];
		A.d[i][5] = pts.d[i][2];
	}
	// solve least squares fit for centroid
	if(unlikely(rc_algebra_lin_system_solve_qr(A,b,&f))){
		fprintf(stderr,"ERROR in rc_fit_ellipsoid, failed to solve QR\n");
		rc_matrix_free(&A);
		rc_vector_free(&b);
		rc_vector_free(&f);
		return -1;
	}
	// done with A&b now
	rc_matrix_free(&A);
	rc_vector_free(&b);

	// compute center
	if(unlikely(rc_vector_alloc(ctr,3))){
		fprintf(stderr,"ERROR in rc_fit_ellipsoid, failed to allocate ctr\n");
		rc_vector_free(&f);
		return -1;
	}
	ctr->d[0] = -f.d[1]/(2.0*f.d[0]);
	ctr->d[1] = -f.d[3]/(2.0*f.d[2]);
	ctr->d[2] = -f.d[5]/(2.0*f.d[4]);

	// Solve for lengths
	if(unlikely(rc_vector_alloc(&b,3))){
		fprintf(stderr,"ERROR in rc_fit_ellipsoid, failed to alloc vector\n");
		return -1;
	}
	if(unlikely(rc_matrix_alloc(&A,3,3))){
		fprintf(stderr,"ERROR in rc_fit_ellipsoid, failed to alloc matrix\n");
		rc_vector_free(&b);
		return -1;
	}
	// fill in A
	A.d[0][0] = (f.d[0] * ctr->d[0] * ctr->d[0]) + 1.0;
	A.d[0][1] = (f.d[0] * ctr->d[1] * ctr->d[1]);
	A.d[0][2] = (f.d[0] * ctr->d[2] * ctr->d[2]);
	A.d[1][0] = (f.d[2] * ctr->d[0] * ctr->d[0]);
	A.d[1][1] = (f.d[2] * ctr->d[1] * ctr->d[1]) + 1.0;
	A.d[1][2] = (f.d[2] * ctr->d[2] * ctr->d[2]);
	A.d[2][0] = (f.d[4] * ctr->d[0] * ctr->d[0]);
	A.d[2][1] = (f.d[4] * ctr->d[1] * ctr->d[1]);
	A.d[2][2] = (f.d[4] * ctr->d[2] * ctr->d[2]) + 1.0;
	// fill in b
	b.d[0] = f.d[0];
	b.d[1] = f.d[2];
	b.d[2] = f.d[4];
	// solve for lengths
	if(unlikely(rc_algebra_lin_system_solve(A,b,lens))){
		fprintf(stderr,"ERROR in rc_fit_ellipsoid, failed to solve linear system\n");
		rc_matrix_free(&A);
		rc_vector_free(&b);
		rc_vector_free(&f);
		return -1;
	}
	lens->d[0] = 1.0/sqrt(lens->d[0]);
	lens->d[1] = 1.0/sqrt(lens->d[1]);
	lens->d[2] = 1.0/sqrt(lens->d[2]);
	// cleanup
	rc_matrix_free(&A);
	rc_vector_free(&b);
	rc_vector_free(&f);
	return 0;
}
