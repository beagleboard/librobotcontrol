/**
 * @file rc_model.c
 * @example rc_model
 *
 * Prints the current BealgleBoard model the software is running on
 *
 *
 *
 * @author     James Strawson
 * @date       2/23/2018
 */


#include <stdio.h>
#include <rc/model.h>

int main()
{
	printf("\nCurrently running on a:\n");
	rc_model_print();
	printf("\n\nmodel category:\n");
	rc_model_category_print();
	printf("\n");
	return 0;
}
