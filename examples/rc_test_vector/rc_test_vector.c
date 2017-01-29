/*******************************************************************************
* rc_test_vector.c
*
* James Strawson 2016
* Tests the vector operations in rc_vector.c
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

#define LEN 4

int main(){
	int i;
	rc_vector_t a = rc_empty_vector();
	rc_vector_t b = rc_empty_vector();
	
	printf("Testing vector functions\n\n");
	
	//  create ones vector
	rc_vector_ones(&a, LEN);
	printf("\nVector of ones:\n");
	rc_print_vector(a);
	
	//  create zero vector
	rc_vector_zeros(&a, LEN);
	printf("\nVector of Zeros:\n");
	rc_print_vector(a);
	
	//  create random vector
	rc_random_vector(&a, LEN);
	printf("\nRandom Vector:\n");
	rc_print_vector(a);
	
	//  duplicate vector
	rc_duplicate_vector(a,&b);
	printf("\nDuplicate vector:\n");
	rc_print_vector(b);
	
	// times scalar
	rc_vector_times_scalar(&b,2.0f);
	printf("\nVector Times 2\n");
	rc_print_vector(b);
	
	// sum
	rc_vector_sum_inplace(&b,a);
	printf("\nsum a+2a\n");
	rc_print_vector(b);
	
	// vector from array
	float fib[] = {1,1,2,3,5,8};
	rc_vector_from_array(&a,fib,6);
	printf("\nFibonacci vector from array:\n");
	rc_print_vector(a);
	
	// fibonnaci vector
	rc_vector_fibonnaci(&a,6);
	printf("\nFibonacci vector built-in function:\n");
	rc_print_vector(a);
	
	// vector norm
	printf("\n%7f Vector 1-norm\n", rc_vector_norm(a,1.0f));
	printf("%7f Vector 2-norm\n", rc_vector_norm(a,2.0f));
	printf("%7f Vector 3-norm\n", rc_vector_norm(a,3.0f));
	
	// vector max min
	i = rc_vector_max(a);
	printf("%7f Vector Max at position %d\n", a.d[i],i);
	i = rc_vector_min(a);
	printf("%7f Vector Min at position %d\n", a.d[i],i);

	// standard deviation
	printf("%7f standard deviation\n", rc_std_dev(a));
	
	// mean
	printf("%7f mean\n", rc_vector_mean(a));
	
	// projection
	
	printf("\nDONE\n");
	return 0;
}
