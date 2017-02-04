/*******************************************************************************
* rc_linear_algebra.c
*
* James Strawson & Matt Atlas 2016
* Numerical Renaissance codebase used as reference for many algorithms
*******************************************************************************/

#include "rc_algebra_common.h"

/*******************************************************************************
* int rc_matrix_times_col_vec(rc_matrix_t A, rc_vector_t v, rc_vector_t* c)
*
* Multiplies matrix A times column vector v and places the result in column
* vector c. Any existing data in c is freed if necessary and c is resized
* appropriately. Vectors v and c are interpreted as column vectors, but nowhere
* in their definitions are they actually specified as one or the other.
* Returns 0 on success and -1 on failure.
*******************************************************************************/
int rc_matrix_times_col_vec(rc_matrix_t A, rc_vector_t v, rc_vector_t* c){
	int i;
	// sanity checks
	if(unlikely(!A.initialized || !v.initialized)){
		fprintf(stderr,"ERROR in rc_matrix_times_col_vec, matrix or vector uninitialized\n");
		return -1;
	}
	if(unlikely(A.cols!=v.len)){
		fprintf(stderr,"ERROR in rc_matrix_times_col_vec, dimension mismatch\n");
		return -1;
	}
	if(unlikely(rc_alloc_vector(c,A.rows))){
		fprintf(stderr,"ERROR in rc_matrix_times_col_vec, failed to allocate c\n");
		return -1;
	}
	// run the sum
	for(i=0;i<A.rows;i++) c->d[i]=rc_mult_accumulate(A.d[i],v.d,v.len);
	return 0;
}

/*******************************************************************************
* int rc_row_vec_times_matrix(rc_vector_t v, rc_matrix_t A, rc_vector_t* c)
*
* Multiplies row vector v times matrix A and places the result in row
* vector c. Any existing data in c is freed if necessary and c is resized
* appropriately. Vectors v and c are interpreted as row vectors, but nowhere
* in their definitions are they actually specified as one or the other.
* Returns 0 on success and -1 on failure.
*******************************************************************************/
int rc_row_vec_times_matrix(rc_vector_t v, rc_matrix_t A, rc_vector_t* c){
	int i,j;
	float* tmp;
	// sanity checks
	if(unlikely(!A.initialized || !v.initialized)){
		fprintf(stderr,"ERROR in rc_row_vec_times_matrix, matrix or vector uninitialized\n");
		return -1;
	}
	if(unlikely(A.rows!=v.len)){
		fprintf(stderr,"ERROR in rc_row_vec_times_matrix, dimension mismatch\n");
		return -1;
	}
	// allocate memory for a column of A from the stack, this is faster than 
	// malloc and the memory is freed automatically when this function returns
	// it is faster to put a column of A in contiguous memory then multiply
	tmp = alloca(A.rows*sizeof(float));
	if(unlikely(tmp==NULL)){
		fprintf(stderr,"ERROR in rc_row_vec_times_matrix, alloca failed, stack overflow\n");
		return -1;
	}
	// make sure c is allocated correctly
	if(unlikely(rc_alloc_vector(c,A.cols))){
		fprintf(stderr,"ERROR in rc_row_vec_times_matrix, failed to allocate c\n");
		return -1;
	}
	// go through columns of A calculating c left to right
	for(i=0;i<A.cols;i++){
		// put column of A in sequential memory slot
		for(j=0;j<A.rows;j++) tmp[j]=A.d[j][i];
		// calculate each entry in c
		c->d[i]=rc_mult_accumulate(v.d,tmp,v.len);
	}
	return 0;
}

/*******************************************************************************
* float rc_matrix_determinant(rc_matrix_t A)
*
* Returns the determinant of square matrix A or -1.0f on failure.
*******************************************************************************/
float rc_matrix_determinant(rc_matrix_t A){
	int i,j,k;
	float ratio, det;
	rc_matrix_t tmp = rc_empty_matrix();
	// sanity checks
	if(unlikely(!A.initialized)){
		fprintf(stderr,"ERROR in rc_matrix_determinant, received uninitialized matrix\n");
		return -1.0f;
	}
	if(unlikely(A.rows!=A.cols)){
		fprintf(stderr,"ERROR in rc_matrix_determinant, expected square matrix\n");
		return -1.0f;
	}
	// shortcut for 1x1 matrix
	if(A.rows==1) return A.d[0][0];
	// shortcut for 2x2 matrix
	if(A.rows==2) return A.d[0][0]*A.d[1][1] - A.d[0][1]*A.d[1][0];
	// allocate a duplicate to shuffle around
	if(unlikely(rc_duplicate_matrix(A,&tmp))){
		fprintf(stderr,"ERROR in rc_matrix_determinant, failed to allocate duplicate\n");
		return -1.0f;
	}
	for(i=0;i<(A.rows-1);i++){
		for(j=i+1;j<A.rows;j++){
			ratio = tmp.d[j][i]/tmp.d[i][i];
			for(k=0;k<A.rows;k++) tmp.d[j][k] -= ratio*tmp.d[i][k];
		}
	}
	// multiply along the main diagonal
	det = 1.0f;
	for(i=0;i<A.rows;i++) det *= tmp.d[i][i];
	// free memory and return
	rc_free_matrix(&tmp);
	return det;
}

/*******************************************************************************
* int rc_lup_decomp(rc_matrix_t A, rc_matrix_t* L, rc_matrix_t* U, rc_matrix_t* P)
*
* Performs LUP decomposition on matrix A with partial pivoting and places the
* result in matrices L,U,&P. Matrix A remains untouched and the original
* contents of LUP (if any) are freed and LUP are resized appropriately.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_lup_decomp(rc_matrix_t A, rc_matrix_t* L, rc_matrix_t* U, rc_matrix_t* P){
	int i,j,k,m,index,tmpint;
	float s1, s2;
	int* ptmp;
	void* rowtmp;
	rc_matrix_t Adup = rc_empty_matrix();
	// sanity checks
	if(unlikely(!A.initialized)){
		fprintf(stderr,"ERROR in rc_lup_decomp, matrix not initialized yet\n");
		return -1;
	}
	if(unlikely(A.cols!=A.rows)){
		fprintf(stderr,"ERROR in rc_lup_decomp, matrix is not square\n");
		return -1;
	}
	// allocate some memory!
	m = A.cols;
	if(unlikely(rc_duplicate_matrix(A, &Adup))){
		fprintf(stderr,"ERROR in rc_lup_decomp, failed to duplicate A\n");
		return -1;
	}
	if(unlikely(rc_identity_matrix(L,m))){
		fprintf(stderr,"ERROR in rc_lup_decomp, failed to allocate identity matrix\n");
		rc_free_matrix(&Adup);
		return -1;
	}
	if(unlikely(rc_alloc_matrix(U,m,m))){
		fprintf(stderr,"ERROR in rc_lup_decomp, failed to allocate U\n");
		rc_free_matrix(&Adup);
		rc_free_matrix(L);
		return -1;
	}
	if(unlikely(rc_matrix_zeros(P,m,m))){
		fprintf(stderr,"ERROR in rc_lup_decomp, failed to allocate matrix of zeros\n");
		rc_free_matrix(&Adup);
		rc_free_matrix(L);
		rc_free_matrix(U);
		return -1;
	}
	// represent P as an array of positions 0 through (m-1) for fast pivoting
	// also alloc memory for a row of A as tmp holder while pivoting
	ptmp = alloca(m*sizeof(int));
	rowtmp = alloca(m*sizeof(float));
	if(unlikely(ptmp==NULL || rowtmp==NULL)){
		fprintf(stderr,"ERROR in rc_lup_decomp, alloca failed, stack overflow\n");
		rc_free_matrix(&Adup);
		rc_free_matrix(L);
		rc_free_matrix(U);
		rc_free_matrix(P);
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
			memcpy(rowtmp,Adup.d[index],m*sizeof(float));
			memcpy(Adup.d[index],Adup.d[i],m*sizeof(float));
			memcpy(Adup.d[i],rowtmp,m*sizeof(float));
		}
	}
	// construct P from ptmp
	for(i=0;i<m;i++) P->d[i][ptmp[i]]=1.0f;
	// now do normal LU
	for(i=0;i<m;i++){
		for(j=0;j<m;j++){
			s1 = 0.0f;
			s2 = 0.0f;
			for(k=0;k<i;k++) s1 += U->d[k][j] * L->d[i][k];
			for(k=0;k<j;k++) s2 += U->d[k][j] * L->d[i][k];
			if(j>=i) U->d[i][j] = (Adup.d[i][j]-s1);
			if(i>=j) L->d[i][j] = (Adup.d[i][j]-s2)/U->d[j][j];
		}
	}
	rc_free_matrix(&Adup);
	return 0;
}

/*******************************************************************************
* int qr_multiply_q_right(rc_matrix_t* A, rc_matrix_t x)
*
* performs a right matrix multiplication of x on the bottom right minor of A 
* where x is smaller than or equal to the size of A. Only used here in the
* backend, not for user access.
*******************************************************************************/
int qr_multiply_q_right(rc_matrix_t* A, rc_matrix_t x){
	int i,j,k,q;
	rc_matrix_t tmp = rc_empty_matrix();
	float* col;
	if(unlikely(!A->initialized || !x.initialized)){
		fprintf(stderr,"ERROR in qr_multiply_q_right, uninitialized matrix\n");
		return -1;
	}
	if(unlikely(A->cols<x.rows || A->rows<x.cols)){
		fprintf(stderr,"ERROR in qr_multiply_q_right, dimension mismatch\n");
		return -1;
	}
	// q is the number of columns of A left untouched because x is smaller
	q=A->cols-x.rows;
	// duplicate the subset of A to be operated on
	if(unlikely(rc_alloc_matrix(&tmp,A->rows, x.rows))){
		fprintf(stderr,"ERROR in qr_multiply_q_right, failed to allocate tmp\n");
		return -1;
	}
	for(i=0;i<A->rows;i++){
		for(j=0;j<x.rows;j++){
			tmp.d[i][j]=A->d[i][j+q];
		}
	}
	// allocate memory for a column of tmp from the stack, this is faster than 
	// malloc and the memory is freed automatically when this function returns
	// it is faster to put a column in contiguous memory before multiplying
	col = alloca(x.rows*sizeof(float));
	if(unlikely(col==NULL)){
		fprintf(stderr,"ERROR in qr_multiply_q_right, alloca failed, stack overflow\n");
		rc_free_matrix(&tmp);
		return -1;
	}
	// do the multiplication, overwriting A left q columns of A remain untouched 
	for(j=0;j<(A->cols-q);j++){
		// put column of x in sequential memory slot
		for(k=0;k<x.rows;k++) col[k]=x.d[k][j];
		// now go down the i'th column of A
		// x is hermetian so don't bother transposing its columns
		for(i=0;i<A->rows;i++){
			A->d[i][j+q]=rc_mult_accumulate(tmp.d[i],col,x.rows);
		}
	}
	// free up tmp memory
	rc_free_matrix(&tmp);
	return 0;
}

/*******************************************************************************
* int qr_multiply_r_left(rc_matrix_t x, rc_matrix_t* Am, float norm)
*
* performs a left matrix multiplication of x on the bottom right minor of A 
* where x is smaller than or equal to the size of A. Only used here in the
* backend, not for user access.
*******************************************************************************/
int qr_multiply_r_left(rc_matrix_t H, rc_matrix_t* R, float norm){
	int i,j,p;
	rc_matrix_t tmp = rc_empty_matrix();
	// sanity checks
	if(unlikely(!R->initialized || !H.initialized)){
		fprintf(stderr,"ERROR in qr_multiply_q_right, uninitialized matrix\n");
		return -1;
	}
	if(R->rows<H.cols){
		fprintf(stderr,"ERROR in  qr_multiply_r_left dimension mismatch\n");
		return -1;
	}
	// p is the index of A defining the top left corner of the operation
	// p=q=0 if x is same size as A', 
	p=R->rows-H.rows; 
	//q=R->cols-H.cols;
	// alloc memory for a duplicate the subset of A to be operated on
	if(unlikely(rc_alloc_matrix(&tmp, R->cols-p, R->rows-p))){
		fprintf(stderr,"ERROR in qr_multiply_r_left, failed to allocate tmp\n");
		return -1;
	}
	// Copy Section of R to be operated on
	// store it in transpose form so memory access later is contiguous
	for(i=0;i<R->rows-p;i++){
		for(j=0;j<R->cols-p;j++){
			tmp.d[j][i]=R->d[i+p][j+p];
		}
	}
	// go through the rows of R starting from the first row that requires
	// modifying. as H shrinks, only the lower rows need modifying
	for(i=0;i<(R->rows-p);i++){
		// we know first column of R will be mostly zeros, so fill in zeros
		// or known norm where possible. we use p to 
		if(i==0)	R->d[i+p][p]=norm;
		else		R->d[i+p][p]=0.0f;
		// do multiplication for the rest of the columns
		// A has already been transposed so don't transpose each column
		for(j=1;j<(R->cols-p);j++){
			R->d[i+p][j+p]=rc_mult_accumulate(H.d[i],tmp.d[j],H.cols);
		}
	}
	// free up tmp memory
	rc_free_matrix(&tmp);
	return 0;
}

/*******************************************************************************
* rc_matrix_t qr_householder_matrix(rc_vector_t x, float* new_norm)
*
* returns the householder reflection matrix for a given vector
* where u=x-ae1, v=u/norm(u), H=I-(2/norm(x))vv'
* warning! modifies x!, only for use by qr decomposition below
*******************************************************************************/
rc_matrix_t qr_householder_matrix(rc_vector_t x, float* new_norm){
	int i, j;
	float norm, tau, taui, dot;
	rc_matrix_t out = rc_empty_matrix();
	rc_vector_t v = rc_empty_vector();

	if(unlikely(!x.initialized)){
		fprintf(stderr,"ERROR in qr_householder_matrix, vector uninitialized\n");
		return rc_empty_matrix();
	}
	// allocate memory for output matrix
	if(unlikely(rc_alloc_matrix(&out,x.len,x.len))){
		fprintf(stderr,"ERROR in qr_householder_matrix, failed to alloc matrix\n");
		return rc_empty_matrix();
	}
	// allocate memory for output matrix
	if(unlikely(rc_alloc_vector(&v,x.len))){
		fprintf(stderr,"ERROR in qr_householder_matrix, failed to alloc vector\n");
		rc_free_matrix(&out);
		return rc_empty_matrix();
	}
	// set sign of norm to opposite of the pivot to avoid loss of significance
	norm = rc_vector_norm(x,2);
	if(x.d[0]>=0.0){
		x.d[0]=(x.d[0]+norm);
		*new_norm = -norm;
	}
	else {
		x.d[0]=(x.d[0]-norm);
		*new_norm = norm;
	}
	// pre-calculate matrix multiplication coefficient
	// doing this on one line causes a compiler optimization error :-/
	dot = rc_vector_dot_product(x,x);
	tau = -2.0/dot;
	// fill in diagonal and upper triangle of H
	for(i=0;i<v.len;i++){
		taui = tau*x.d[i];
		// H=I-(2/norm(x))vv' so add 1 on the diagonal
		out.d[i][i] = 1.0 + taui*x.d[i];
		for(j=i+1;j<v.len;j++){
			out.d[i][j] = taui*x.d[j];
		}
	}
	// copy to lower triangle
	for(i=1;i<v.len;i++){
		for(j=0;j<i;j++){
			out.d[i][j] = out.d[j][i];
		}
	}
	rc_free_vector(&v);
	return out;
}

/*******************************************************************************
* int rc_qr_decomp(rc_matrix_t A, rc_matrix_t* Q, rc_matrix_t* R)
*
* Uses householder reflection method to find the QR decomposition of A.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_qr_decomp(rc_matrix_t A, rc_matrix_t* Q, rc_matrix_t* R){
	int i,j,steps;
	float norm;
	rc_vector_t x = rc_empty_vector();
	rc_matrix_t H = rc_empty_matrix();
	// Sanity Checks
	if(unlikely(!A.initialized)){
		fprintf(stderr,"ERROR in rc_qr_decomp, matrix not initialized yet\n");
		return -1;
	}
	// start R as A
	if(unlikely(rc_duplicate_matrix(A,R))){
		fprintf(stderr,"ERROR in rc_qr_decomp, failed to duplicate A\n");
		return -1;
	}
	// start R as square identity
	rc_identity_matrix(Q,A.rows);
	// find out how many householder reflections are necessary
	if(A.rows==A.cols) steps=A.cols-1;		// square
	else if(A.rows>A.cols) steps=A.cols;	// tall
	else steps=A.rows-1;					// wide
	// iterate through columns of A doing householder reflection to zero
	// the entries below the diagonal
	for(i=0;i<steps;i++){
		// take col of R from diag down
		rc_alloc_vector(&x,A.rows-i);
		for(j=i;j<A.rows;j++) x.d[j-i]=R->d[j][i];
		// get the ever-shrinking householder reflection for that column
		// qr_householder also fills in the norm of that column to 'norm'
		H = qr_householder_matrix(x, &norm);
		rc_free_vector(&x);
		// left multiply R
		qr_multiply_r_left(H,R,norm);
		qr_multiply_q_right(Q,H);
		rc_free_matrix(&H);
	}
	return 0;
}

/*******************************************************************************
* int rc_invert_matrix(rc_matrix_t A, rc_matrix_t* Ainv)
*
* Inverts Matrix A via LUP decomposition method and places the result in matrix
* Ainv. Any existing memory allocated for Ainv is freed if necessary and its
* contents are overwritten. Returns 0 on success or -1 on failure such as if
* matrix A is not invertible.
*******************************************************************************/
int rc_invert_matrix(rc_matrix_t A, rc_matrix_t* Ainv){
	int i,j,k;
	rc_matrix_t L = rc_empty_matrix();
	rc_matrix_t U = rc_empty_matrix();
	rc_matrix_t P = rc_empty_matrix();
	rc_matrix_t D = rc_empty_matrix();
	rc_matrix_t tmp = rc_empty_matrix();
	// sanity checks
	if(unlikely(!A.initialized)){
		fprintf(stderr,"ERROR in rc_matrix_inverse, matrix uninitialized\n");
		return -1;
	}
	if(unlikely(A.cols!=A.rows)){
		fprintf(stderr,"ERROR in rc_matrix_inverse, nonsquare matrix\n");
		return -1;
	}
	if(fabs(rc_matrix_determinant(A)) < 0.0001f){
		fprintf(stderr,"ERROR in rc_matrix_inverse, matrix is singular\n");
		return -1;
	}
	// allocate memory
	if(unlikely(rc_identity_matrix(&D,A.cols))){
		fprintf(stderr,"ERROR in rc_matrix_inverse, failed to alloc identity\n");
		return -1;
	}
	if(unlikely(rc_alloc_matrix(&tmp,A.rows,A.rows))){
		fprintf(stderr,"ERROR in rc_matrix_inverse, failed to alloc matrix\n");
		rc_free_matrix(&D);
		return -1;
	}
	// do LUP
	if(unlikely(rc_lup_decomp(A,&L,&U,&P))){
		fprintf(stderr,"ERROR in rc_matrix_inverse, failed to LUP decomp\n");
		rc_free_matrix(&D);
		rc_free_matrix(&tmp);
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
	rc_free_matrix(&L);
	rc_free_matrix(&U);
	rc_free_matrix(&D);
	// use i as new return value
	i=0;
	// multiply by permutation matrix
	if(unlikely(rc_multiply_matrices(tmp, P, Ainv))){
		fprintf(stderr,"ERROR in rc_matrix_inverse, failed to multiply matrix\n");
		i=-1;
	}
	// free allocation	
	rc_free_matrix(&tmp);
	rc_free_matrix(&P);
	return i;
}

/*******************************************************************************
* int rc_invert_matrix_inplace(rc_matrix_t* A)
*
* Inverts Matrix A in place. The original contents of A are lost.
* Returns 0 on success or -1 on failure such as if A is not invertible.
*******************************************************************************/
int rc_invert_matrix_inplace(rc_matrix_t* A){
	rc_matrix_t Atmp = rc_empty_matrix();
	if(unlikely(rc_invert_matrix(*A,&Atmp))){
		fprintf(stderr, "ERROR in rc_invert_matrix_inplace, failed to invert\n");
		return -1;
	}
	// free original memory and copy new matrix into place
	rc_free_matrix(A);
	*A = Atmp;
	return 0;
}

/*******************************************************************************
* int rc_lin_system_solve(rc_matrix_t A, rc_vector_t b, rc_vector_t* x)
*
* Solves Ax=b for given matrix A and vector b. Places the result in vector x.
* existing contents of x are freed and new memory is allocated if necessary.
* Thank you to Henry Guennadi Levkin for open sourcing this routine, it's
* adapted here for RC use and includes better detection of unsolvable systems.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_lin_system_solve(rc_matrix_t A, rc_vector_t b, rc_vector_t* x){
	float fMaxElem, fAcc;
	int nDim,i,j,k,m;
	rc_matrix_t Atemp = rc_empty_matrix();
	rc_vector_t btemp = rc_empty_vector();
	// sanity checks
	if(!A.initialized || !b.initialized){
		fprintf(stderr,"ERROR in rc_lin_system_solve, matrix or vector uninitialized\n");
		return -1;
	}
	if(A.cols != b.len){
		fprintf(stderr,"ERROR in rc_lin_system_solve, dimension mismatch\n");
		return -1;
	}
	// alloc memory for x
	nDim = A.cols;
	if(unlikely(rc_alloc_vector(x,nDim))){
		fprintf(stderr,"ERROR in rc_lin_system_solve, failed to alloc vector\n");
		return -1;
	}
	// duplicate user arguments so we don't have to modify them
	if(unlikely(rc_duplicate_matrix(A, &Atemp))){
		fprintf(stderr,"ERROR in rc_lin_system_solve, failed to duplicate matrix\n");
		rc_free_vector(x);
		return -1;
	}
	if(unlikely(rc_duplicate_vector(b, &btemp))){
		fprintf(stderr,"ERROR in rc_lin_system_solve, failed to duplicate vector\n");
		rc_free_vector(x);
		rc_free_matrix(&Atemp);
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
		if(unlikely(fabs(Atemp.d[k][k])<ZERO_TOLERANCE)){
			fprintf(stderr,"ERROR in rc_lin_system_solve, matrix not full rank\n");
			rc_free_matrix(&Atemp);
			rc_free_vector(&btemp);
			rc_free_vector(x);
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
	rc_free_matrix(&Atemp);
	rc_free_vector(&btemp);
	return 0;
}

/*******************************************************************************
* int rc_lin_system_solve_qr(rc_matrix_t A, rc_vector_t b, rc_vector_t* x)
*
* Finds a least-squares solution to the system Ax=b for non-square A using QR
* decomposition method and places the solution in x. 
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_lin_system_solve_qr(rc_matrix_t A, rc_vector_t b, rc_vector_t* x){
	int i,k;
	rc_vector_t temp = rc_empty_vector();
	rc_matrix_t Q = rc_empty_matrix();
	rc_matrix_t R = rc_empty_matrix();
	if(unlikely(!A.initialized || !b.initialized)){
		fprintf(stderr,"ERROR in rc_lin_system_solve_qr, matrix or vector uninitialized\n");
		return -1;
	}
	// do QR decomposition
	if(unlikely(rc_qr_decomp(A,&Q,&R))){
		fprintf(stderr,"ERROR in rc_lin_system_solve_qr, failed to perform QR decomp\n");
		return -1;
	}
	// Ax=b
	// QRx=b
	// Rx=Q'b		because Q'Q=I
	// RX=(b'Q)'	to avoid transposing q
	// multiply through right hand side. No difference between row and col
	// vector so avoid transposing Q by left instead of right multiplying
	if(unlikely(rc_row_vec_times_matrix(b,Q,&temp))){
		fprintf(stderr,"ERROR in rc_lin_system_solve_qr, failed to multiply vec by matrix\n");
		rc_free_matrix(&Q);
		rc_free_matrix(&R);
		return -1;
	}
	// allocate memory for the output x
	if(unlikely(rc_alloc_vector(x,R.cols))){
		fprintf(stderr,"ERROR in rc_lin_system_solve_qr, failed to alloc vector\n");
		rc_free_matrix(&Q);
		rc_free_matrix(&R);
		rc_free_vector(&temp);
		return -1;
	}
	// solve for x knowing R is upper triangular
	for(k=R.cols-1;k>=0;k--){
		x->d[k]=temp.d[k];
		for(i=k+1;i<R.cols;i++)	x->d[k]-=R.d[k][i]*x->d[i];
		x->d[k] = x->d[k]/R.d[k][k];
	}
	// free memory and return
	rc_free_matrix(&Q);
	rc_free_matrix(&R);
	rc_free_vector(&temp);
	return 0;
}

/*******************************************************************************
* int rc_fit_ellipsoid(rc_matrix_t pts, rc_vector_t* ctr, rc_vector_t* lens)
*
* Fits an ellipsoid to a set of points in 3D space. The principle axes of the
* fitted ellipsoid align with the global coordinate system. Therefore there are
* 6 degrees of freedom defining the ellipsoid: the x,y,z coordinates of the
* centroid and the lengths from the centroid to the surface in each of the 3
* directions. 
*
* rc_matrix_t 'pts' is a tall matrix with 3 columns and at least 6 rows.
* Each row must contain the x,y&z components of each individual point to be fit.
* If only 6 rows are provided, the resulting ellipsoid will be an exact fit.
* Otherwise the result is a least-squares fit to the over-defined dataset.
*
* The final x,y,z position of the centroid will be placed in vector 'ctr' and
* the lengths or radius from the centroid to the surface along each axis will
* be placed in the vector 'lens'
*
* Returns 0 on success or -1 on failure. 
*******************************************************************************/
int rc_fit_ellipsoid(rc_matrix_t pts, rc_vector_t* ctr, rc_vector_t* lens){
	int i,p;
	rc_matrix_t A = rc_empty_matrix();
	rc_vector_t b = rc_empty_vector();
	rc_vector_t f = rc_empty_vector();
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
	if(unlikely(rc_alloc_matrix(&A,p,6))){
		fprintf(stderr,"ERROR in rc_fit_ellipsoid, failed to alloc matrix\n");
		rc_free_vector(&b);
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
	if(unlikely(rc_lin_system_solve_qr(A,b,&f))){
		fprintf(stderr,"ERROR in rc_fit_ellipsoid, failed to solve QR\n");
		rc_free_matrix(&A);
		rc_free_vector(&b);
		rc_free_vector(&f);
		return -1;
	}
	// done with A&b now
	rc_free_matrix(&A);
	rc_free_vector(&b);
	
	// compute center 
	if(unlikely(rc_alloc_vector(ctr,3))){
		fprintf(stderr,"ERROR in rc_fit_ellipsoid, failed to allocate ctr\n");
		rc_free_vector(&f);
		return -1;
	}
	ctr->d[0] = -f.d[1]/(2.0f*f.d[0]);
	ctr->d[1] = -f.d[3]/(2.0f*f.d[2]);
	ctr->d[2] = -f.d[5]/(2.0f*f.d[4]);
	
	// Solve for lengths
	if(unlikely(rc_alloc_vector(&b,3))){
		fprintf(stderr,"ERROR in rc_fit_ellipsoid, failed to alloc vector\n");
		return -1;
	}
	if(unlikely(rc_alloc_matrix(&A,3,3))){
		fprintf(stderr,"ERROR in rc_fit_ellipsoid, failed to alloc matrix\n");
		rc_free_vector(&b);
		return -1;
	}
	// fill in A
	A.d[0][0] = (f.d[0] * ctr->d[0] * ctr->d[0]) + 1.0f;
	A.d[0][1] = (f.d[0] * ctr->d[1] * ctr->d[1]);
	A.d[0][2] = (f.d[0] * ctr->d[2] * ctr->d[2]);
	A.d[1][0] = (f.d[2] * ctr->d[0] * ctr->d[0]);
	A.d[1][1] = (f.d[2] * ctr->d[1] * ctr->d[1]) + 1.0f;
	A.d[1][2] = (f.d[2] * ctr->d[2] * ctr->d[2]);
	A.d[2][0] = (f.d[4] * ctr->d[0] * ctr->d[0]);
	A.d[2][1] = (f.d[4] * ctr->d[1] * ctr->d[1]);
	A.d[2][2] = (f.d[4] * ctr->d[2] * ctr->d[2]) + 1.0f;
	// fill in b
	b.d[0] = f.d[0];
	b.d[1] = f.d[2];
	b.d[2] = f.d[4];
	// solve for lengths
	if(unlikely(rc_lin_system_solve(A,b,lens))){
		fprintf(stderr,"ERROR in rc_fit_ellipsoid, failed to solve linear system\n");
		rc_free_matrix(&A);
		rc_free_vector(&b);
		rc_free_vector(&f);
		return -1;
	}
	lens->d[0] = 1.0f/sqrt(lens->d[0]);
	lens->d[1] = 1.0f/sqrt(lens->d[1]);
	lens->d[2] = 1.0f/sqrt(lens->d[2]);
	// cleanup
	rc_free_matrix(&A);
	rc_free_vector(&b);
	rc_free_vector(&f);
	return 0;
}
