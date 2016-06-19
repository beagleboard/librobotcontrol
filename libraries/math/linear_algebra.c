/*******************************************************************************
* linear_algebra.c
*
* James Strawson & Matt Atlas 2016
*******************************************************************************/

#include "../robotics_cape.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/*******************************************************************************
* 
*
* 
*******************************************************************************/
matrix_t createMatrix(int rows, int cols){
	int i;
	matrix_t A;
	if(rows<1 || cols<1){
		printf("error creating matrix, row or col must be >=1");
		return A;
	}
	A.rows = rows;
	A.cols = cols;
	A.data = (float**)malloc(rows * sizeof(float*));
		for (i=0; i<rows; i++)
			A.data[i] = (float*)calloc(cols, sizeof(float));
		
	A.initialized = 1;
	return A;
}

/*******************************************************************************
* 
*
* copy information of one matrix to a new memory location 
*******************************************************************************/
matrix_t duplicateMatrix(matrix_t A){
	int i,j;
	matrix_t out;
	if(!A.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return out;
	}
	out = createMatrix(A.rows,A.cols);
	for(i=0;i<A.rows;i++){
        for(j=0;j<A.cols;j++){
			out.data[i][j] = A.data[i][j];
		}
	}
	return out;
}


/*******************************************************************************
* 
*
* 
*******************************************************************************/
matrix_t createSquareMatrix(int n){
	matrix_t A = createMatrix(n,n);
	return A;
}

/*******************************************************************************
* 
*
* 
*******************************************************************************/
matrix_t createRandomMatrix(int rows, int cols){
	int i,j;
	matrix_t A;
	if(rows<1 || cols<1){
		printf("error creating matrix, row or col must be >=1");
		return A;
	}
	A = createMatrix(rows, cols);
	for(i=0;i<rows;i++){
		for(j=0;j<cols;j++){
			A.data[i][j]=get_random_float();
		}
	}
	return A;
}

/*******************************************************************************
* matrix_t createIdentityMatrix(int dim)
*
* 
*******************************************************************************/
matrix_t createIdentityMatrix(int dim){
	int i;
	matrix_t A;
	if(dim<1){
		printf("error creating matrix, dim must be >=1");
		return A;
	}
	A = createSquareMatrix(dim);
	for(i=0;i<dim;i++){
		A.data[i][i]=1;
	}
	return A;
}

/*******************************************************************************
* matrix_t createDiagonalMatrix(vector_t v)
*
* 
*******************************************************************************/
matrix_t createDiagonalMatrix(vector_t v){
	int i;
	matrix_t A;
	if(!v.initialized){
		printf("error creating matrix, vector_t v not initialized");
		return A;
	}
	A = createSquareMatrix(v.len);
	for(i=0;i<v.len;i++){
		A.data[i][i]=v.data[i];
	}
	return A;
}

/*******************************************************************************
* matrix_t createMatrixOfOnes(int dim)
*
* 
*******************************************************************************/
matrix_t createMatrixOfOnes(int dim){
	int i,j;
	matrix_t A;
	if(dim<1){
		printf("error creating matrix, dim must be >=1");
		return A;
	}
	A = createSquareMatrix(dim);
	for(i=0;i<dim;i++){
		for(j=0;j<dim;j++){
			A.data[i][j]=1;
		}
	}
	return A;
}

/*******************************************************************************
* 
*
* 
*******************************************************************************/
int setMatrixEntry(matrix_t* A, int row, int col, float val){
	if(A==NULL){
		printf("ERROR: matrix is null pointer\n");
		return -1;
	}
	if(!A->initialized){
		printf("ERROR: A not initialized yet\n");
		return -1;
	}
	if(row<0 || row>=A->rows){
		printf("ERROR: row out of bounds\n");
		return -1;
	}
	if(col<0 || col>=A->cols){
		printf("ERROR: col out of bounds\n");
		return -1;
	}
	A->data[row][col] = val;
	return 0;
}

/*******************************************************************************
* 
*
* 
*******************************************************************************/
float getMatrixEntry(matrix_t A, int row, int col){
	if(!A.initialized){
		printf("ERROR: A not initialized yet\n");
		return -1;
	}
	if(row<0 || row>=A.rows){
		printf("ERROR: row out of bounds\n");
		return -1;
	}
	if(col<0 || col>=A.cols){
		printf("ERROR: col out of bounds\n");
		return -1;
	}
	return  A.data[row][col];
}

/*******************************************************************************
* 
*
* 
*******************************************************************************/
void printMatrix(matrix_t A){
	int i,j;
	if(!A.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return;
	}
	for(i=0;i<A.rows;i++){
		for(j=0;j<A.cols;j++){
			printf("%7.3f  ",A.data[i][j]);
		}	
		printf("\n");
	}
	return;
}	

/*******************************************************************************
* 
*
* 
*******************************************************************************/
void printMatrixSciNotation(matrix_t A){
	int i,j;
	if(!A.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return;
	}
	for(i=0;i<A.rows;i++){
		for(j=0;j<A.cols;j++){
			printf("%11.4e  ",A.data[i][j]);
		}	
		printf("\n");
	}
	return;
}	

/*******************************************************************************
* 
*
* 
*******************************************************************************/
void destroyMatrix(matrix_t* A){
	int i;
	for (i=0; i<A->rows; i++) free(A->data[i]);
	free(A->data);
	A->rows = 0;
	A->cols = 0;
	A->initialized = 0;
	return;
}

/*******************************************************************************
* 
*
* 
*******************************************************************************/
vector_t createVector(int n){
	vector_t v;
	if(n<1){
		printf("error creating vector, n must be >=1");
		return v;
	}
	v.len = n;
	v.data = (float*)calloc(n, sizeof(float));
	v.initialized = 1;
	return v;
}

/*******************************************************************************
* 
*
* 
*******************************************************************************/
vector_t duplicateVector(vector_t v){
	int i;
	vector_t out;
	if(!v.initialized){
		printf("ERROR: vector not initialized yet\n");
		return out;
	}
	out = createVector(v.len);
	for(i=0;i<v.len;i++){
		out.data[i] = v.data[i];
	}
	return out;
}

/*******************************************************************************
* 
*
* 
*******************************************************************************/
vector_t createRandomVector(int len){
	int i;
	vector_t v;
	if(len<1){
		printf("error creating vector, len must be >=1");
		return v;
	}
	v = createVector(len);
	for(i=0;i<len;i++){
		v.data[i]=get_random_float();
	}
	return v;
}

/*******************************************************************************
* 
*
* 
*******************************************************************************/
vector_t createVectorOfOnes(int len){
	int i;
	vector_t v;
	if(len<1){
		printf("error creating vector, len must be >=1");
		return v;
	}
	v = createVector(len);
	for(i=0;i<len;i++){
		v.data[i]=1;
	}
	return v;
}

/*******************************************************************************
* 
*
* 
*******************************************************************************/
int setVectorEntry(vector_t* v, int pos, float val){
	if(v==NULL){
		printf("ERROR: v is null pointer\n");
		return -1;
	}
	if(!v->initialized){
		printf("ERROR: v not initialized yet\n");
		return -1;
	}
	if(pos<0 || pos>=v->len){
		printf("ERROR: pos out of bounds\n");
		return -1;
	}
	v->data[pos] = val;
	return 0;
}

/*******************************************************************************
* 
*
* 
*******************************************************************************/
float getVectorEntry(vector_t v, int pos){
	if(!v.initialized){
		printf("ERROR: v not initialized yet\n");
		return -1;
	}
	if(pos<0 || pos>=v.len){
		printf("ERROR: pos out of bounds\n");
		return -1;
	}
	return v.data[pos];
}

/*******************************************************************************
* 
*
* 
*******************************************************************************/
void printVector(vector_t v){
	int i;
	if(!v.initialized){
		printf("ERROR: vector not initialized yet\n");
		return;
	}
	for(i=0;i<v.len;i++){
		printf("%7.3f  ",v.data[i]);
	}
	printf("\n");
	return;
}	

/*******************************************************************************
* 
*
* 
*******************************************************************************/
void printVectorSciNotation(vector_t v){
	int i;
	if(!v.initialized){
		printf("ERROR: vector not initialized yet\n");
		return;
	}
	for(i=0;i<v.len;i++){
		printf("%.4e  ",v.data[i]);
	}
	printf("\n");
	return;
}	

/*******************************************************************************
* 
*
* 
*******************************************************************************/
void destroyVector(vector_t* v){
	free(v->data);
	v->len = 0;
	v->initialized = 0;
	return;
}



/*******************************************************************************
* 
*
* 
*******************************************************************************/
matrix_t matrixMultiply(matrix_t A, matrix_t B){
	int i,j,k;
	float sum = 0;
	matrix_t out;
	if(!A.initialized||!B.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return out;
	}
	if (A.cols != B.rows){
		printf("ERROR: Invalid matrix sizes");
		return out;
	}
	out = createMatrix(A.rows, B.cols);	
	for(i=0;i<(A.rows);i++){
		for(j=0;j<(B.cols);j++){	
			for(k=0;k<(A.cols);k++){
				// do the matrix multiplication
				sum = sum + A.data[i][k]*B.data[k][j];
			}
			// save mult sum to new location
			out.data[i][j] = sum;
			sum = 0; 	// re-initialize sum for next loop
		}
	}
	return out;
}

/*******************************************************************************
* 
*
* 
*******************************************************************************/
int matrixTimesScalar(matrix_t* A, float s){
	int i,j;
	if(!A->initialized){
		printf("ERROR: matrix not initialized yet\n");
		return -1;
	}
	for(i=0;i<(A->rows);i++){
		for(j=0;j<(A->cols);j++){	
			A->data[i][j] = s*A->data[i][j];
		}
	}
	return 0;
}

/*******************************************************************************
* 
*
* 
*******************************************************************************/
int vectorTimesScalar(vector_t* v, float s){
	int i;
	if(!v->initialized){
		printf("ERROR: vector not initialized yet\n");
		return -1;
	}
	for(i=0;i<(v->len);i++){	
		v->data[i] = s*v->data[i];
	}
	return 0;
}

/*******************************************************************************
* 
*
* 
*******************************************************************************/
vector_t matrixTimesColVec(matrix_t A, vector_t v){
	int i,j;
	vector_t out;
	if(!A.initialized || !v.initialized){
		printf("ERROR: matrix or vector not initialized yet\n");
		return out;
	}
	if(A.cols != v.len){
		printf("ERROR: dimensions do not match\n");
		return out;
	}
	out = createVector(A.rows);
	
	for(i=0;i<A.rows;i++){
		for(j=0;j<A.cols;j++){	
			out.data[i] += v.data[j]*A.data[i][j];
		}
	}
	return out;
}

/*******************************************************************************
* 
*
* 
*******************************************************************************/
vector_t rowVecTimesMatrix(vector_t v, matrix_t A){
	int i,j;
	vector_t out;
	if(!A.initialized || !v.initialized){
		printf("ERROR: matrix or vector not initialized yet\n");
		return out;
	}
	if(A.rows != v.len){
		printf("ERROR: dimensions do not match\n");
		return out;
	}
	
	out = createVector(A.cols);
	for(i=0;i<A.cols;i++){
		for(j=0;j<A.rows;j++){	
			out.data[i] += v.data[j]*A.data[j][i];
		}
	}
	return out;
}



/*******************************************************************************
* 
*
* 
*******************************************************************************/
matrix_t matrixAdd(matrix_t A, matrix_t B){
	int i,j;
	matrix_t out;
	if(!A.initialized||!B.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return out;
	}
	if ((A.rows != B.rows)||(A.cols != B.cols)){
		printf("Invalid matrix sizes");
		return out;
	}
	out = createMatrix(A.rows, A.cols);
	for(i=0;i<(A.rows);i++){
		for(j=0;j<(A.cols);j++){	
			out.data[i][j] = A.data[i][j] + B.data[i][j];
		}
	}
	return  out;
}

/*******************************************************************************
* 
*
* 
*******************************************************************************/
int transposeMatrix(matrix_t* A){
	int i,j;
	if(!A->initialized){
		printf("ERROR: matrix not initialized yet\n");
		return -1;
	}
	// swap rows and cols
	matrix_t temp = createMatrix(A->cols, A->rows);
	for(i=0;i<(A->rows);i++){
		for(j=0;j<(A->cols);j++){	
			temp.data[i][j] = A->data[j][i];
		}
	}
	// unallocate the original matrix A and set its data pointer to point
	// to the newly allocated memory
	destroyMatrix(A);
	*A=temp;
	return  0;
}


/*******************************************************************************
* float vectorNorm(vector_t v)
*
* 
*******************************************************************************/
float vectorNorm(vector_t v){
	float out = 0;
	int i;
	if(!v.initialized){
		printf("ERROR: vector not initialized yet\n");
		return -1;
	}
	for(i=0;i<v.len;i++){
		out = out + v.data[i]*v.data[i];
	}
	return sqrt(out);
}

/*******************************************************************************
* Projects vector v onto e
* 
* 
*******************************************************************************/
vector_t vectorProjection(vector_t v, vector_t e){
	
	int i;
	float factor;
	vector_t out;
	
	if(!v.initialized || !e.initialized){
	printf("ERROR: vectors not initialized yet\n");
	return out;
	}
	if(v.len != e.len){
	printf("ERROR: vectors not of same dimension\n");
	return out;
	}
	out = createVector(v.len);
	
	factor = dotProduct(v,e)/dotProduct(e,e);
	for(i=0;i<v.len;i++){
		out.data[i] = factor * e.data[i];
	}
	
	return out;
}


/*******************************************************************************
* v1 x v2 = v1^T v2
* 
* 
*******************************************************************************/
matrix_t outerProduct(vector_t v1, vector_t v2){
	
	int i, j;
	int m = v1.len;
	int n = v2.len;
	matrix_t out;
	
	if(!v1.initialized || !v2.initialized){
	printf("ERROR: vectors not initialized yet\n");
	return out;
	}

	out = createMatrix(m,n);
	
	for(i=0;i<m;i++){
		for(j=0;j<n;j++){
			out.data[j][i] = v1.data[i]*v2.data[j];
		}
	}
	return out;
}

/*******************************************************************************
* 
*
* 
*******************************************************************************/
float dotProduct(vector_t v1, vector_t v2){
	float out;
	int i;
	
	if(!v1.initialized || !v2.initialized){
		printf("ERROR: vector not initialized yet\n");
		return -1;
	}
	if(v1.len != v2.len){
		printf("ERROR: vector dimensions do not match\n");
		return -1;
	}
	for(i=0; i<v1.len; i++){
		out = out + (v1.data[i] * v2.data[i]);
	}
	return out;
}

/*******************************************************************************
* 
*
* 
*******************************************************************************/
vector_t crossProduct3D(vector_t v1, vector_t v2){
	vector_t out;
	if(!v1.initialized || !v2.initialized){
		printf("ERROR: vector not initialized yet\n");
		return out;
	}
	if((v1.len != 3) || (v2.len != 3)){
		printf("ERROR: vectors not of dimension 3\n");
		return out;
	}
	
	out = createVector(v1.len);
	out.data[0] = (v1.data[1]*v2.data[2]) - (v1.data[2]*v2.data[1]);
	out.data[1] = (v1.data[2]*v2.data[0]) - (v1.data[0]*v2.data[2]);
	out.data[2] = (v1.data[0]*v2.data[1]) - (v1.data[1]*v2.data[0]);
	return out;	
}


/*******************************************************************************
* 
*
* 
*******************************************************************************/
float matrixDeterminant(matrix_t A){
	int i,j,k;
	float ratio, det;
	if(!A.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return -1;
	}
	if (A.rows != A.cols){
		printf("Error: Matrix is not square");
		return -1;
	}
	matrix_t temp = duplicateMatrix(A);
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
	destroyMatrix(&temp);
    return det;  
}


/*******************************************************************************
* 
*
* 
*******************************************************************************/
int matrixInv(matrix_t* A){
	int i,j,ii,jj,i1,j1;
	float det,coDet;
	matrix_t out, cofactors;
	if(!A->initialized){
		printf("ERROR: matrix not initialized yet\n");
		return -1;
	}
	det = matrixDeterminant(*A);
	
	printf("det = %f\n",det);
	if (det == 0){
		printf("Error: Matrix is not invertable");
		return -1;
	}
	out = createSquareMatrix(A->rows);
	cofactors = createSquareMatrix(A->rows - 1);
	
	for (i=0;i<A->rows;i++){				// current row of A to test
		for (j=0;j<A->rows;j++){			// current col of A to test

			i1 = 0;							// index for cofactor row
			for (ii=0;ii<A->rows;ii++){		// count up thru # of rows of A
				if (ii == i) continue;		// if = to current row of A.. skip
										
				j1 = 0;						// index for cofactor col
				for (jj=0;jj<A->rows;jj++){	// count up thru # of cols of A
					if (jj == j) continue;	// if = to current col of A.. skip
					// place proper element in new matrix
					cofactors.data[i1][j1] = A->data[ii][jj]; 	
					j1++;
				}
				i1++;
			}
			coDet = matrixDeterminant(cofactors);
			// saves as transpose
			out.data[j][i] = (pow(-1.0,i+j+2.0) * coDet)/det; 	
		}
	}
	destroyMatrix(&cofactors);
	destroyMatrix(A);
	*A=out;
	return 0;
}

/*******************************************************************************
* 
*
* 
*******************************************************************************/
matrix_t Householder(vector_t v){
	
	int i, j;
	float tau;
	matrix_t out;
	
	if(!v.initialized){
	printf("ERROR: vector not initialized yet\n");
	return out;
	}
	
	out = createSquareMatrix(v.len);
	for(i=0;i<v.len;i++){
		out.data[i][i] = 1;
	}
	
	tau = 2*dotProduct(v,v);
	
	for(i=0;i<v.len;i++){
		for(j=0;j<v.len;j++){
			out.data[i][j] -= tau * v.data[i]*v.data[j];
		}
	}
	return out;
}

/*******************************************************************************
* 
*
* 
*******************************************************************************/
int QRdecomposition(matrix_t A, matrix_t*Q, matrix_t* R){
	
	int i, j, k, s;
	int m = A.rows;
	int n = A.cols;

	vector_t xtemp;
	matrix_t Qt, Rt, Qi, F;
	
	if(!A.initialized){
	printf("ERROR: matrix not initialized yet\n");
	return -1;
	}
	
	destroyMatrix(Q);
	destroyMatrix(R);
	Qt = createMatrix(m,m);
	for(i=0;i<m;i++){					// initialize Qt as I
		Qt.data[i][i] = 1;
	}
	Rt = duplicateMatrix(A);			// duplicate A to Rt

	for(i=0;i<n;i++){					// iterate through columns of A
		xtemp = createVector(m-i);		// allocate length, decreases with i
		
		for(j=i;j<m;j++){						// take col of -R from diag down
			xtemp.data[j-i] = -Rt.data[j][i]; 	
		}
		if(Rt.data[i][i] > 0)	s = -1;			// check the sign
		else					s = 1;
		xtemp.data[0] += s*vectorNorm(xtemp);	// add norm to 1st element
		
		Qi = createSquareMatrix(m);			// initialize Qi
		F  = createSquareMatrix(m-i);			// initialize shrinking Householder
		F  = Householder(xtemp);			// fill in Househodor
		
		for(j=0;j<i;j++){
			Qi.data[j][j] = 1;				// fill in partial I matrix
		}
		for(j=i;j<m;j++){					// fill in remainder (Householder)
			for(k=i;k<m;k++){
				Qi.data[j][k] = F.data[j-i][k-i];
			}
		}
		Qt = matrixMultiply(Qi,Qt);			// multiply new Qi to old Qtemp
		Rt = matrixMultiply(Qi,Rt);			// same with Rtemp
		
		destroyMatrix(&Qi);					// free allocation
		destroyMatrix(&F);
		destroyVector(&xtemp);
	}
	transposeMatrix(&Qt);
	*Q = Qt;
	*R = Rt;
	return 0;
}

/*******************************************************************************
* vector_t linSolve(matrix_t A, vector_t b)
*
* This is a feeble attempt to duplicate the functionality of Matlab's A\b method
* for solving linear equations. It returns the vector x that solves Ax=b
* Thank you to  Henry Guennadi Levkin for open sourcing this routine.
*******************************************************************************/
vector_t linSolve(matrix_t A, vector_t b){
	float fMaxElem, fAcc;
	int nDim,i,j,k,m;
	vector_t xout;
	
	if(!A.initialized || !b.initialized){
		printf("ERROR: matrix or vector not initialized yet\n");
		return xout;
	}
	if(A.cols != b.len){
		printf("ERROR: matrix dimensions do not match\n");
		return xout;
	}
	
	nDim = A.cols;
	xout = createVector(nDim);
	matrix_t Atemp = duplicateMatrix(A); // duplicate the given matrix 
	vector_t btemp = duplicateVector(b); // duplicate the given vector
	
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

	destroyMatrix(&Atemp);
	destroyVector(&btemp);
	return xout;
}

/*******************************************************************************
* vector_t linSolveQR(matrix_t A, vector_t b)
*
* Gives a least-squares solution to the system AX=b for non-square A using QR.
*
*  Ax=b
* QRx=b
*  Rx=Q'b  (because Q'Q=I)
*  x=(R^-1)Q'b
*******************************************************************************/
vector_t linSolveQR(matrix_t A, vector_t b){
	vector_t xout, Qb;
	matrix_t Q,R;
		
	if(!A.initialized || !b.initialized){
		printf("ERROR: matrix or vector not initialized yet\n");
		return xout;
	}
	// do QR decomposition
	if(QRdecomposition(A,&Q,&R)<0){
		printf("failed to perform QR decomposition on A\n");
		return xout;
	}
	// transpose Q matrix
	if(transposeMatrix(&Q)<0){
		printf("ERROR: failed to transpose Q\n");
		return xout;
	}
	// invert R
	if(matrixInv(&R)<0){
		printf("ERROR: failed to invert R\n");
		return xout;
	}
	// multiply through
	Qb = matrixTimesColVec(Q,b);
	destroyMatrix(&Q);
	xout = matrixTimesColVec(R,Qb);
	destroyMatrix(&R);
	destroyVector(&Qb);
	
	return xout;
}

/*******************************************************************************
* int fitEllipsoid(matrix_t points, vector_t* center, vector_t* lengths)
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
int fitEllipsoid(matrix_t points, vector_t* center, vector_t* lengths){
	int i;
	int p = points.rows;
	if(!points.initialized){
		printf("ERROR: matrix_t points not initialized\n");
		return -1;
	}
	if(points.cols!=3){
		printf("ERROR: matrix_t points must have 3 columns\n");
		return -1;
	}
	if(p<6){
		printf("ERROR: matrix_t points must have at least 6 rows\n");
		return -1;
	}
	
	vector_t b = createVectorOfOnes(p);
	matrix_t A = createMatrix(p,6);
	for(i=0;i<p;i++){
		A.data[i][0] = points.data[i][0] * points.data[i][0];
		A.data[i][1] = points.data[i][0];
		A.data[i][2] = points.data[i][1] * points.data[i][1];
		A.data[i][3] = points.data[i][1];
		A.data[i][4] = points.data[i][2] * points.data[i][2];
		A.data[i][5] = points.data[i][2];
	}
	
	vector_t f = linSolveQR(A,b);
	destroyMatrix(&A);
	destroyVector(&b);
	
	// fill in center vector.
	destroyVector(center);
	*center = createVector(3);
	center->data[0] = -f.data[1]/(2*f.data[0]);
	center->data[1] = -f.data[3]/(2*f.data[2]);
	center->data[2] = -f.data[5]/(2*f.data[4]);
	
	// Solve for lengths
	A = createSquareMatrix(3);
	b = createVector(3);
	
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
	destroyVector(lengths);
	*lengths = linSolve(A, b);
	
	// cleanup
	destroyMatrix(&A);
	destroyVector(&b);
	return 0;
}


