/******************************************************************************* 
* kill_robot.c
*
* James Strawson 2016
* Useful program for shutting down any existing program using robotics cape
* library resources. It accomplishes this by looking for the PID file created
* by initialize_cape(). If that Process ID is still running, it sends the 
* shutdown signal SIGINT. If that doesn't work then kill it with SIGKILL.
*******************************************************************************/ 

#include <useful_includes.h>
#include <robotics_cape.h>

int main(){
	printf("\nAttempting to shut down existing project\n\n");
	int ret = kill_robot();
	
	switch(ret){
	case -2:
		printf("WARNING: invalid contents in PID_FILE\n");
		break;
	case -1:
		printf("Existing project failed to close and had to be killed.\n");
		break;
	case 0:
		printf("No existing program is running.\n");
		break;
	case 1:
		printf("An existing program was running and shut down cleanly.\n");
		break;
	default:
		return 0;
	}
	
	return 0;
}