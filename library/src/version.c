/**
 * @file version.c
 */

#include <stdio.h>
#include <rc/version.h>

#define RC_STRINGIFY(v) RC_STRINGIFY_HELPER(v)
#define RC_STRINGIFY_HELPER(v) #v

#define RC_LIB_VERSION_STRING	RC_STRINGIFY(RC_LIB_VERSION_MAJOR) "." \
				RC_STRINGIFY(RC_LIB_VERSION_MINOR) "." \
				RC_STRINGIFY(RC_LIB_VERSION_PATCH)


unsigned int rc_version(void)
{
	return RC_LIB_VERSION_HEX;
}


const char* rc_version_string(void)
{
	return RC_LIB_VERSION_STRING;
}

void rc_version_print(void)
{
	printf(RC_LIB_VERSION_STRING);
	return;
}