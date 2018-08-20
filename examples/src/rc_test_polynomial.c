/**
 * @example    rc_test_polynomial.c
 *
 * @brief      Tests the functions in rc_polynomial.h
 *
 * @author     James Strawson
 * @date       1/29/2018
 */

#include <stdio.h>
#include <rc/math.h>

#define LEN 4 // length of polynomial to test

int main()
{
	int i;
	rc_vector_t a = RC_VECTOR_INITIALIZER;
	rc_vector_t b = RC_VECTOR_INITIALIZER;
	rc_vector_t c = RC_VECTOR_INITIALIZER;
	rc_vector_t d = RC_VECTOR_INITIALIZER;
	rc_vector_t e = RC_VECTOR_INITIALIZER;

	printf("\nRandom polynomials of orders 0-9\n");
	for(i=1;i<=10;i++){
		rc_vector_random(&a,i);
		rc_poly_print(a);
	}

	printf("\npolynomial of ones a:\n");
	rc_vector_ones(&a, 2);
	rc_poly_print(a);

	printf("polynomial of ones b:\n");
	rc_vector_ones(&b, 3);
	rc_poly_print(b);

	printf("Polynomial convolution a*b:\n");
	rc_poly_conv(a,b,&c);
	rc_poly_print(c);

	// test powers
	printf("\na to the power of 0-5:\n");
	for(i=0;i<=5;i++){
		rc_poly_power(a,i,&c);
		rc_poly_print(c);
	}

	// test subtract
	printf("\nrc_poly_subtract c=b-a\n");
	rc_poly_subtract(b,a,&c);
	rc_poly_print(c);

	// test subtract
	printf("\nrc_poly_subtract_inplace c=c-a\n");
	rc_poly_subtract_inplace(&c,a);
	rc_poly_print(c);

	// test add
	printf("\nrc_poly_add a+b=c\n");
	rc_poly_add(a,b,&c);
	rc_poly_print(c);

	// test add
	printf("\nrc_poly_add_inplace c=c+a\n");
	rc_poly_add_inplace(&c,a);
	rc_poly_print(c);

	// differentiate
	printf("0st-3rd derivative of b\n");
	for(i=0;i<=3;i++){
		rc_poly_differentiate(b,i,&c);
		rc_poly_print(c);
	}

	// divide
	rc_vector_fibonnaci(&a, 2);
	rc_vector_fibonnaci(&b, 5);
	printf("\nNew polynomial a:\n");
	rc_poly_print(a);
	printf("\nNew polynomial b:\n");
	rc_poly_print(b);
	printf("\ndivide a into b\n");
	rc_poly_divide(b,a,&c,&d);
	rc_poly_print(c);
	printf("remainder:\n");
	rc_poly_print(d);

	// confirm
	printf("\nConfirm by multiplying the result and adding remainder\n");
	rc_poly_conv(c,a,&e);
	rc_poly_add_inplace(&e,d);
	rc_poly_print(e);

	// butterworth
	printf("\nfirst 4 butterworth polynomials\n");
	for(i=1;i<=4;i++){
		rc_poly_butter(i,1,&a);
		rc_poly_print(a);
	}

	rc_vector_free(&a);
	rc_vector_free(&b);
	rc_vector_free(&c);
	rc_vector_free(&d);
	rc_vector_free(&e);
	printf("\nDONE\n");
	return 0;
}
