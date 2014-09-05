// Bare Minimum Skeleton for Robotics Cape Project
// James Strawson - 2013

#include <robotics_cape.h>
int i;

int main(){
	initialize_cape();
	
	set_esc(1,0); //this also sets the pwm period for servo/esc use
	
	printf("\nDISCONNECT PROPELLERS FROM MOTORS\n");
	printf("DISCONNECT POWER FROM ESCS\n");
	printf("press start to continue\n");
	
	while(get_start_button() == LOW){
		usleep(10000);
	}
	setGRN(HIGH);
	
	//set everything to full throttle to define upper bound
	for(i=1; i<=6; i++){
		set_esc(i,1);
	}
	
	sleep(1);
	
	printf("\n");
	printf("Now reapply power to the ESCs.\n");
	printf("Press the start button after the ESCs chirp\n");
	
	while(get_start_button() == LOW){
		usleep(10000);
	}
	setGRN(LOW);
	//set everything to 0 throttle to define lower bound
	for(i=1; i<=6; i++){
		set_esc(i,0);
	}
	
	sleep(2);
	
	printf("\nCalibration complete, closing.\n");

	cleanup_cape();
	return 0;
}