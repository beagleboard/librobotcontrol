// kill_robot
// James Strawson - 2014
// 

#include <robotics_cape.h>

int main(){
	FILE* fd;
	int old_pid;
	
	fd = fopen(LOCKFILE, "r");
	if (fd != NULL) {
		fscanf(fd,"%d", &old_pid);
		if(old_pid != 0){
			printf("Cleanly shutting down existing robotics project\n");
			kill((pid_t)old_pid, SIGINT);
			sleep(1);
		}
	}
	else{
		printf("no robot project running\n");
	}
	
	
	// clean up the lockfile if it still exists
	fd = fopen(LOCKFILE, "r");
	if (fd != NULL) {
		kill((pid_t)old_pid, SIGKILL);
		printf("Force closing existing robotics project\n");
		// close and delete the old file
		fclose(fd);
		remove(LOCKFILE);
	}
	
	//cleanup_cape();
	return 0;
}