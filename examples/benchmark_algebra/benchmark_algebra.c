/*******************************************************************************
* test_algebra.c
*
* James Strawson 2016
* This tests some of the more common functions in linear_algebra.c, it is not a
* complete test of all available linear algebra functions but should get you
* started.
*******************************************************************************/

#include "../../libraries/roboticscape-usefulincludes.h"
#include "../../libraries/roboticscape.h"

#define DEFAULT_DIM 50
#define MIN_DIM		1
#define MAX_DIM		500
#define MULT_TIMES	50

// printed if some invalid argument was given
void print_usage(){
	printf("\n");
	printf("-d         use default matrix size (50x50)\n");
	printf("-s {size}  use custom matrix size\n");
	printf("-h         print this help message\n");
	printf("\n");
}


int main(int argc, char *argv[]){
	int dim = 0;
	int c, i;
	uint64_t start_time, tmp1, tmp2;
	double segment_time, mflops;
	matrix_t AA;

	if(argc>3){
		printf("Too many arguments given.\n");
		print_usage();
		return -1;
	}
	if(argc<2){
		printf("Not enough arguments given.\n");
		print_usage();
		return -1;
	}
	// parse arguments
	opterr = 0;
	while ((c = getopt(argc, argv, "ds:h")) != -1){
		switch (c){
		case 'd': // default size option
			if(dim!=0){
				printf("invalid combination of arguments\n");
				print_usage();
				return -1;
			}
			dim = DEFAULT_DIM;
			break;
		case 's': // custom size option
			if(dim!=0){
				printf("invalid combination of arguments\n");
				print_usage();
				return -1;
			}
			dim = atoi(optarg);
			if(dim>MAX_DIM || dim<MIN_DIM){
				printf("requested size out of bounds\n");
				print_usage();
				return -1;
			}
			break;
		case 'h':
			print_usage();
			return 0;
		default:
			printf("inavlid argument\n");
			print_usage();
			return -1;
		}
	}

	start_time = micros_since_epoch();
	// create a random nxn matrix for later use
	matrix_t A = create_random_matrix(dim,dim);
	vector_t b = create_random_vector(dim);
	tmp1 = micros_since_epoch();
	segment_time = (double)(tmp1-start_time)/1000000.0;
	printf("%7.4f Time to make random matrix & vector\n", segment_time);

	// duplicate matrix
	for(i=0;i<MULT_TIMES;i++){
		AA=duplicate_matrix(A);
		destroy_matrix(&AA);
	}
	tmp2 = tmp1;
	tmp1 = micros_since_epoch();
	segment_time = (double)(tmp1-tmp2)/1000000.0;
	printf("%7.4f Time to duplicate matrix %d times\n", segment_time, MULT_TIMES);


	// get determinant of A
	double det = matrix_determinant(A);
	det--; // shut up warning about unused variable
	tmp2 = tmp1;
	tmp1 = micros_since_epoch();
	segment_time = (double)(tmp1-tmp2)/1000000.0;
	printf("%7.4f Time to take matrix determinant\n", segment_time);

	// get an inverse for A
	matrix_t Ainv = matrix_inverse(A);
	tmp2 = tmp1;
	tmp1 = micros_since_epoch();
	segment_time = (double)(tmp1-tmp2)/1000000.0;
	printf("%7.4f Time to invert matrix\n", segment_time);

	// multiply A times A inverse
	for(i=0;i<MULT_TIMES;i++){
		AA = multiply_matrices(A,Ainv);
		destroy_matrix(&AA);
	}
	tmp2 = tmp1;
	tmp1 = micros_since_epoch();
	segment_time = (double)(tmp1-tmp2)/1000000.0;
	printf("%7.4f Time to multiply A*Ainv %d times\n", segment_time, MULT_TIMES);
	mflops = (dim*dim*dim*MULT_TIMES)/(segment_time*1000000);

	// do an LUP decomposition on A
	matrix_t L,U,P;
	LUP_decomposition(A,&L,&U,&P);
	tmp2 = tmp1;
	tmp1 = micros_since_epoch();
	segment_time = (double)(tmp1-tmp2)/1000000.0;
	printf("%7.4f Time to do LUP decomposition\n", segment_time);

	// do a QR decomposition on A
	matrix_t Q,R;
	QR_decomposition(A,&Q,&R);
	tmp2 = tmp1;
	tmp1 = micros_since_epoch();
	segment_time = (double)(tmp1-tmp2)/1000000.0;
	printf("%7.4f Time to do QR decomposition\n", segment_time);

	// solve a square linear system
	vector_t x = lin_system_solve(A, b);
	tmp2 = tmp1;
	tmp1 = micros_since_epoch();
	segment_time = (double)(tmp1-tmp2)/1000000.0;
	printf("%7.4f Time to do gaussian elimination A\\b\n", segment_time);

	// clean up all the allocated memory. This isn't strictly necessary since
	// we are already at the end of the program, but good practice to do.
	destroy_matrix(&A);
	destroy_vector(&b);
	destroy_matrix(&Ainv);
	destroy_matrix(&AA);
	destroy_matrix(&L);
	destroy_matrix(&U);
	destroy_matrix(&P);
	destroy_matrix(&Q);
	destroy_matrix(&R);
	destroy_vector(&x);

	tmp2 = tmp1;
	tmp1 = micros_since_epoch();
	segment_time = (double)(tmp1-tmp2)/1000000.0;
	printf("%7.4f Time to free memory\n", segment_time);

	segment_time = (double)(tmp1-start_time)/1000000.0;
	printf("%7.4f Total time\n", segment_time);

	printf("%7.1f Million floating point operations per second\n", mflops);

	printf("DONE\n");
	return 0;
}
