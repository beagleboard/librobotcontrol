// Program to test Hobby Brushless Speed controller function
// oscillates output between %0 - %10
// Hold the START button on the cape or hit Ctrl-C to exit
// James Strawson - 2013

#include <robotics_cape.h>
#define MAX_DUTY 0.1 //%10, this is often just enough to make the motor spin
		
int main(){
	int i;
	
	initialize_cape();
	set_esc(1,0); //this also sets the PWM period for esc/servos
	printf("\n");
	sleep(1);
	
	//Keep Running until program state changes
	while(get_state() != EXITING){
		for(i=1; i<=6; i++){ 
			set_esc(i,MAX_DUTY);
		}
		setGRN(HIGH);
		printf("\rON ");
		fflush(stdout);
		sleep(1);
		
		for(i=1; i<=6; i++){
			set_esc(i,0);
		}
		printf("\rOFF");
		fflush(stdout);
		setGRN(LOW);
		sleep(1);
	}
	cleanup_cape();
	return 0;
}
