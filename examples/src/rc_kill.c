/**
 * @file rc_kill.c
 * @example    rc_kill
 *
 * Example program for shutting down any existing process that uses the robotics
 * cape library PID file functionality in <rc/start_stop.h> such as the
 * rc_balance and rc_test_start_stop examples.
 */

#include <stdio.h>
#include <rc/start_stop.h>

#define TIMEOUT 1.5 // seconds

int main()
{
	int ret = rc_kill_existing_process(TIMEOUT);
	switch(ret){
	case -2:
		printf("WARNING: invalid contents in PID_FILE, deleted file anyway\n");
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
