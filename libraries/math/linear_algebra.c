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
matrix_t createSqrMatrix(int n){
	matrix_t A = createMatrix(n,n);
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
	printf("\n");
	for(i=0;i<A.rows;i++){
		for(j=0;j<A.cols;j++){
			printf("%f\t",A.data[i][j]);
		}	
		printf("\n");
	}
	printf("\n");
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
	printf("\n");
	for(i=0;i<v.len;i++){
		printf("%f\t",v.data[i]);
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
matrix_t matrixMultiply(matrix_t* A, matrix_t* B){
	int i,j,k;
	float sum = 0;
	matrix_t out;
	if(!A->initialized||!B->initialized){
		printf("ERROR: matrix not initialized yet\n");
		return out;
	}
	if (A->cols != B->rows){
		printf("ERROR: Invalid matrix sizes");
		return out;
	}
	out = createMatrix(A->rows, B->cols);	
	for(i=0;i<(A->rows);i++){
		for(j=0;j<(B->cols);j++){	
			for(k=0;k<(A->cols);k++){
				// do the matrix multiplication
				sum = sum + A->data[i][k]*B->data[k][j];
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
int matrixMultiplyScalar(matrix_t* A, float s){
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
float matrixDet(matrix_t A){
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
matrix_t matrixInv(matrix_t A){
	int i,j,ii,jj,i1,j1;
	float det,coDet;
	matrix_t out, cofactors;
	if(!A.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return out;
	}
	det = matrixDet(A);
	
	printf("det = %f\n",det);
	if (det == 0){
		printf("Error: Matrix is not invertable");
		return out;
	}
	out = createSqrMatrix(A.rows);
	cofactors = createSqrMatrix(A.rows - 1);
	
	for (i=0;i<A.rows;i++){					// current row of A to test
		for (j=0;j<A.rows;j++){				// current col of A to test

			i1 = 0;							// index for cofactor row
			for (ii=0;ii<A.rows;ii++){		// count up thru # of rows of A
				if (ii == i) continue;		// if = to current row of A.. skip
										
				j1 = 0;						// index for cofactor col
				for (jj=0;jj<A.rows;jj++){	// count up thru # of cols of A
					if (jj == j) continue;	// if = to current col of A.. skip
					// place proper element in new matrix
					cofactors.data[i1][j1] = A.data[ii][jj]; 	
					j1++;
				}
				i1++;
			}
			coDet = matrixDet(cofactors);
			// saves as transpose
			out.data[j][i] = (pow(-1.0,i+j+2.0) * coDet)/det; 	
		}
	}
	destroyMatrix(&cofactors);
	return out;
}



//==============================================================================
// return 1 if system not solving
// nDim - system dimension
// pfMatr - matrix with coefficients
// pfVect - vector with free members
// xout - vector with system solution
// pfMatr becames trianglular after function call
// pfVect changes after function call
//
// Developer:
//
//==============================================================================
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





