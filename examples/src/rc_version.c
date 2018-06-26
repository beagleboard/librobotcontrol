/**
 * @file rc_version.c
 * @example    rc_version
 *
 * Prints the current version of the Robot Control Library.
 *
 *
 *
 * @author     James Strawson
 * @date       1/29/2018
 */



#include <stdio.h>
#include <rc/version.h>

int main()
{
	// simply print the version and a newline
	rc_version_print();
	printf("\n");
	// alternatively you could use rc_version() for numeric comparisons
	// or get the string directly with rc_version_string();
	return 0;
}
