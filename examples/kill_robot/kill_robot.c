// kill_robot
// James Strawson - 2014
// 

#include <robotics_cape.h>

int main(){
	FILE* fd;
	int old_pid;
	
	fd = fopen(PID_FILE, "r");
	if (fd != NULL) {
		fscanf(fd,"%d", &old_pid);
		if(old_pid != 0){
			printf("Cleanly shutting down existing robotics project\n");
			kill((pid_t)old_pid, SIGINT);
			sleep(3);
		}
	}
	else{
		printf("no robot project running\n");
	}
	
	
	// force kill the program if the PID file never got cleaned up
	fd = fopen(PID_FILE, "r");
	if (fd != NULL) {
		kill((pid_t)old_pid, SIGKILL);
		printf("Force closing existing robotics project\n");
		// close and delete the old file
		fclose(fd);
		remove(PID_FILE);
	}
	
	return 0;
}