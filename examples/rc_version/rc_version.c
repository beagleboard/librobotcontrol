/*******************************************************************************
* rc_version.c
*
* Prints the current version of the robotics cape library. 
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

int main(){
	// simply print the version and a newline
	printf("%s\n", rc_version_string());
	// alternatively you could use rc_version_float() for numeric comparisons
	return 0;
}
