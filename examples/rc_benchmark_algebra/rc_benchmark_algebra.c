/*******************************************************************************
* rc_benchmark_algebra.c
*
* James Strawson 2016
* This tests some of the more common functions in linear_algebra.c, it is not a
* complete test of all available linear algebra functions but should get you
* started.
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

#define DEFAULT_DIM 140
#define MIN_DIM		1
#define MAX_DIM		250

#define TIMER rc_nanos_thread_time()
#define TIMER_DELAY 2100 // ns consumed just by reading the thread time

// printed if some invalid argument was given
void print_usage(){
	printf("\n");
	printf("-d         use default matrix size (%dx%d)\n",DEFAULT_DIM,DEFAULT_DIM);
	printf("-s {size}  use custom matrix size\n");
	printf("-h         print this help message\n");
	printf("\n");
}


int main(int argc, char *argv[]){
	int dim = 0;
	int c;
	uint64_t t1, t2, diff, flops, mflops;
	rc_vector_t b = rc_empty_vector();
	rc_matrix_t A = rc_empty_matrix();
	rc_matrix_t AA = rc_empty_matrix();
	rc_matrix_t B = rc_empty_matrix();
	rc_matrix_t L = rc_empty_matrix();
	rc_matrix_t U = rc_empty_matrix();
	rc_matrix_t P = rc_empty_matrix();
	rc_matrix_t Q = rc_empty_matrix();
	rc_matrix_t R = rc_empty_matrix();
	rc_vector_t x = rc_empty_vector();
	// make sure user gave an argument
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

	// set clock speed to 1000mhz to make sure scaling doesn't effect results
	rc_set_cpu_freq(FREQ_1000MHZ);
	printf("Starting\n");
	
	// create a random nxn matrix for later use
	t1 = TIMER;
	rc_random_matrix(&A,dim,dim);
	rc_vector_zeros(&b,dim);
	t2 = TIMER;
	diff = (t2-t1-TIMER_DELAY);
	printf("%10lldus Time to make random matrix & vector\n", diff/1000);
	
	// duplicate matrix
	t1 = TIMER;
	rc_duplicate_matrix(A,&AA);
	t2 = TIMER;
	diff = (t2-t1-TIMER_DELAY);
	printf("%10lldus Time to duplicate matrix\n", diff/1000);
	
	// Multiply matrices
	rc_alloc_matrix(&B,dim,dim);
	t1 = TIMER;
	rc_multiply_matrices(A, AA, &B);
	t2 = TIMER;
	diff = (t2-t1-TIMER_DELAY);
	printf("%10lldus Time to multiply matrices\n", diff/1000);
	
	// calculate floating pointer operations per second, both multiplication
	// and addition count as operations, hence multiply by 2
	flops = ((uint64_t)2*dim*dim*dim*1000000000)/(diff);
	mflops = flops/(uint64_t)1000000;
	printf("%10lld MFLOPS multiplying matrices\n", mflops);
	
	// find determinant
	t1 = TIMER;
	rc_matrix_determinant(A);
	t2 = TIMER;
	diff = (t2-t1-TIMER_DELAY);
	printf("%10lldus Time to find matrix determinant\n", diff/1000);
	
	// find inverse
	t1 = TIMER;
	rc_invert_matrix(A, &AA);
	t2 = TIMER;
	diff = (t2-t1-TIMER_DELAY);
	printf("%10lldus Time to invert matrix\n", diff/1000);
	
	// LUP
	rc_alloc_matrix(&L,dim,dim);
	rc_alloc_matrix(&U,dim,dim);
	rc_alloc_matrix(&P,dim,dim);
	t1 = TIMER;
	rc_lup_decomp(A,&L,&U,&P);
	t2 = TIMER;
	diff = (t2-t1-TIMER_DELAY);
	printf("%10lldus Time to do LUP decomposition\n", diff/1000);

	// do a QR decomposition on A
	rc_alloc_matrix(&Q,dim,dim);
	rc_alloc_matrix(&R,dim,dim);
	t1 = TIMER;
	rc_qr_decomp(A,&Q,&R);
	t2 = TIMER;
	diff = (t2-t1-TIMER_DELAY);
	printf("%10lldus Time to do QR decomposition\n", diff/1000);

	// do a QR decomposition on A
	rc_alloc_vector(&x,dim);
	t1 = TIMER;
	rc_lin_system_solve(A,b,&x);
	t2 = TIMER;
	diff = (t2-t1-TIMER_DELAY);
	printf("%10lldus Time to solve linear system\n", diff/1000);

	printf("DONE\n");
	rc_set_cpu_freq(FREQ_ONDEMAND);
	return 0;
}
