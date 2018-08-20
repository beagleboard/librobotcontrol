/**
 * @example    rc_test_vector.c
 *
 * @brief      Tests the functions in rc_vector.h
 *
 *
 * @author     James Strawson
 * @date       1/29/2018
 */

#include <stdio.h>
#include <rc/math.h>


#define LEN 4 // length of vector to test

int main()
{
	int i;
	rc_vector_t a = RC_VECTOR_INITIALIZER;
	rc_vector_t b = RC_VECTOR_INITIALIZER;

	printf("Testing vector functions\n\n");

	// create ones vector
	rc_vector_ones(&a, LEN);
	printf("\nVector of ones:\n");
	rc_vector_print(a);

	// create zero vector
	rc_vector_zeros(&a, LEN);
	printf("\nVector of Zeros:\n");
	rc_vector_print(a);

	// create random vector
	rc_vector_random(&a, LEN);
	printf("\nRandom Vector:\n");
	rc_vector_print(a);

	// duplicate vector
	rc_vector_duplicate(a,&b);
	printf("\nDuplicate vector:\n");
	rc_vector_print(b);

	// times scalar
	rc_vector_times_scalar(&b,2.0f);
	printf("\nVector Times 2\n");
	rc_vector_print(b);

	// sum
	rc_vector_sum_inplace(&b,a);
	printf("\nsum a+2a\n");
	rc_vector_print(b);

	// vector from array
	double fib[] = {1,1,2,3,5,8};
	rc_vector_from_array(&a,fib,6);
	printf("\nFibonacci vector from array:\n");
	rc_vector_print(a);

	// fibonnaci vector
	rc_vector_fibonnaci(&a,6);
	printf("\nFibonacci vector built-in function:\n");
	rc_vector_print(a);

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
	printf("%7f standard deviation\n", rc_vector_std_dev(a));

	// mean
	printf("%7f mean\n", rc_vector_mean(a));

	// cleanup
	rc_vector_free(&a);
	rc_vector_free(&b);

	printf("\nDONE\n");
	return 0;
}
