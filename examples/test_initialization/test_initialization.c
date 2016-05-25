/*******************************************************************************
* test_initialization.c
*
* Simple check to make sure the cape library initializes without errors.
* This is called by the Auto_Run_Script.sh on boot to make sure the cape works
* before loading your project.
*******************************************************************************/
#include <../robotics_cape.h>
#include <../useful_includes.h>

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
