/*******************************************************************************
* test_initialization.c
*
* James Strawson 2016
* Simple check to make sure the cape library initializes without errors.
* This is called by the Auto_Run_Script.sh on boot to make sure the cape works
* before loading your project. You don't need to use this directly.
*******************************************************************************/

#include <useful_includes.h>
#include <robotics_cape.h>

int main(){
	if(initialize_cape()<0){
		printf("FAILURE: initialize_cape() failed\n");
		cleanup_cape();
		return -1;
	}
	else{
		printf("SUCCESS: initialize_cape() worked\n");
		cleanup_cape();
		return 0;
	}
}
