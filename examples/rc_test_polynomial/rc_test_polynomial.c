/*******************************************************************************
* rc_test_polynomial.c
*
* Tests the vector operations in rc_polynomial.c
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

#define LEN 4

int main(){
	int i;
	
	rc_vector_t a = rc_empty_vector();
	rc_vector_t b = rc_empty_vector();
	rc_vector_t c = rc_empty_vector();
	rc_vector_t d = rc_empty_vector();
	rc_vector_t e = rc_empty_vector();
	
	printf("\nRandom polynomials of orders 0-9\n");
	for(i=1;i<=10;i++){
		rc_random_vector(&a,i);
		rc_print_poly(a);
	}
	
	// test convolution
	rc_vector_ones(&a, 2);
	rc_vector_ones(&b, 3);
	printf("\npolynomial of ones a:\n");
	rc_print_poly(a);
	printf("polynomial of ones b:\n");
	rc_print_poly(b);
	rc_poly_conv(a,b,&c);
	printf("Polynomial convolution a*b:\n");
	rc_print_poly(c);
	
	// test powers
	printf("\na to the power of 0-5:\n");
	for(i=0;i<=5;i++){
		rc_poly_power(a,i,&c);
		rc_print_poly(c);
	}
	
	// test subtract
	printf("\nrc_poly_subtract c=b-a\n");
	rc_poly_subtract(b,a,&c);
	rc_print_poly(c);
	
	// test subtract
	printf("\nrc_poly_subtract_inplace c=c-a\n");
	rc_poly_subtract_inplace(&c,a);
	rc_print_poly(c);
	
	// test add
	printf("\nrc_poly_add a+b=c\n");
	rc_poly_add(a,b,&c);
	rc_print_poly(c);
	
	// test add
	printf("\nrc_poly_add_inplace c=c+a\n");
	rc_poly_add_inplace(&c,a);
	rc_print_poly(c);
	
	// differentiate
	printf("0st-3rd derivative of b\n");
	for(i=0;i<=3;i++){
		rc_poly_differentiate(b,i,&c);
		rc_print_poly(c);
	}
	
	// divide
	rc_vector_fibonnaci(&a, 2);
	rc_vector_fibonnaci(&b, 5);
	printf("\nNew polynomial a:\n");
	rc_print_poly(a);
	printf("\nNew polynomial b:\n");
	rc_print_poly(b);
	printf("\ndivide a into b\n");
	rc_poly_divide(b,a,&c,&d);
	rc_print_poly(c);
	printf("remainder:\n");
	rc_print_poly(d);
	
	// confirm
	printf("\nConfirm by multiplying the result and adding remainder\n");
	rc_poly_conv(c,a,&e);
	rc_poly_add_inplace(&e,d);
	rc_print_poly(e);
	
	// butterworth
	printf("\nfirst 3 butterworth polynomials\n");
	for(i=1;i<=3;i++){
		rc_poly_butter(i,1,&a);
		rc_print_poly(a);
	}
	
	printf("\nDONE\n");
	return 0;
}
