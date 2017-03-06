/******************************************************************************* 
* rc_kill.c
*
* Useful program for shutting down any existing program using robotics cape
* library resources. It accomplishes this by looking for the PID file created
* by rc_initialize(). If that Process ID is still running, it sends the 
* shutdown signal SIGINT. If that doesn't work then kill it with SIGKILL.
*******************************************************************************/ 

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

int main(){
	int ret = rc_kill();
	switch(ret){
	case -2:
		printf("WARNING: invalid contents in PID_FILE\n");
		break;
	case -1:
		printf("Existing project failed to close and had to be killed.\n");
		break;
	case 0:
		printf("No existing roboticscape program is running.\n");
		break;
	case 1:
		printf("An existing program was running and shut down cleanly.\n");
		break;
	default:
		return 0;
	}
	return 0;
}
