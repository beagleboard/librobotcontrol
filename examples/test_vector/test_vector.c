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


int main(){
	printf("Let's test some vector functions....\n\n");
	
	
	//  create random vector
	vector_t a = create_random_vector(4);
	printf("\nNew Random Vector a:\n");
	print_vector(a);
	
	// If b are the coefficients of a polynomial, get the coefficients of the
	// new polynomial b^2
	vector_t a_squared = poly_power(a,2);
	printf("\nCoefficients of polynomial a^2\n");
	print_vector(a_squared);

	// now try a vector norm
	printf("\n2-Norm of a_squared: %f\n", vector_norm(a_squared, 2));
	
	// now try making 2 vectors from arrays
	double num_array[] = {2,6,1};
	double den_array[] = {1,2};
	vector_t num = create_vector_from_array(num_array, 3);
	vector_t den = create_vector_from_array(den_array, 2);
	printf("\nnumerator from array:");
	print_vector(num);
	printf("\ndenominator from array:");
	print_vector(den);


	// divide num by den
	printf("\nDividing den into num\n");
	vector_t div, remainder;
	div = poly_div(num,den,&remainder);
	printf("\nDivisor:");
	print_vector(div);
	printf("\nRemainder:");
	print_vector(remainder);


	// find first and second derivatives of a
	vector_t diff_1 = poly_diff(num, 1);
	printf("\nFirst derivative of num: ");
	print_vector(diff_1);
	vector_t diff_2 = poly_diff(num, 2);
	printf("\nSecond derivative of num: ");
	print_vector(diff_2);

	// try summing vectors of different lengths
	vector_t sum = poly_add(den, num);
	printf("\nsum of num&den: ");
	print_vector(sum);

	// free all memory. 
	// Not strictly necessary since the program will return right after
	destroy_vector(&a);
	destroy_vector(&a_squared);
	destroy_vector(&num);
	destroy_vector(&den);
	destroy_vector(&div);
	destroy_vector(&remainder);
	destroy_vector(&diff_1);
	destroy_vector(&diff_2);
	destroy_vector(&sum);


	printf("\nDONE\n");
	return 0;
}
