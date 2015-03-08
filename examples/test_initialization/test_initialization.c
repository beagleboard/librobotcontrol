// test_initialization.c
// James Strawson - 2013

// check if the cape initializes properly.
// this is called by the Auto_Run_Script.sh on boot to make sure
// all facilities are working before calling your project

#include <robotics_cape.h>

int main(){
	if(initialize_cape()<0){
		printf("initialize_cape() failed\n");
		cleanup_cape();
		return -1;
	}
	else{
		printf("initialize_cape() worked\n");
		cleanup_cape();
		return 0;
	}
}