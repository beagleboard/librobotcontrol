/**
 * @file version.c
 */

#include <stdio.h>
#include <rc/version.h>


float rc_version_float()
{
	return RC_LIB_VERSION_FLOAT;
}


const char* rc_version_string()
{
	// // annoying macro hack to convert to string
	// #define Q(x) #x
	// #define QUOTE(x) Q(x)
	// return QUOTE(RC_LIB_VERSION_STRING);
	// #undef Q
	// #undef QUOTE
	return RC_LIB_VERSION_STRING;
}

void rc_version_print()
{
	printf("%s",RC_LIB_VERSION_STRING);
	return;
}