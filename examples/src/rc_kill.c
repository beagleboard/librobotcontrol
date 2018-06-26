/**
 * @file rc_kill.c
 * @example    rc_kill
 *
 * Example program for shutting down any existing process that uses the robot
 * control library PID file functionality in <rc/start_stop.h> such as the
 * rc_balance and rc_test_start_stop examples.
 */

#include <stdio.h>
#include <rc/start_stop.h>

#define TIMEOUT 1.5 // seconds

int main()
{
	int ret = rc_kill_existing_process(TIMEOUT);
	// print success messages since rc_kill_existing_process is normally
	// quiet on success but prints errors and warnings already.
	switch(ret){
	case 0:
		printf("No existing robot control program is running.\n");
		break;
	case 1:
		printf("An existing program was running and shut down cleanly.\n");
		break;
	default:
		break;
	}
	return ret;
}
