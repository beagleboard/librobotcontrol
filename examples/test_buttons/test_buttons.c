// Button and LED tester for the Robotics Cape
// Pressing either button makes an LED blink
// Press both buttons to exit cleanly
// James Strawson - 2013

#include <robotics_cape.h>

int on_start_press(){
	printf("pressed start\n");
	setGRN(HIGH);
	if(get_select_button() == HIGH){
		set_state(EXITING);
	}
	return 0;
}
int on_start_release(){
	printf("released start\n");
	setGRN(LOW);
	return 0;
}
int on_select_press(){
	printf("pressed select\n");
	setRED(HIGH);
	if(get_start_button() == HIGH){
		set_state(EXITING);
	}
	return 0;
}
int on_select_release(){
	setRED(LOW);
	printf("released select\n");
	return 0;
}


int main(){
	initialize_cape();
	
	printf("\nPress buttons to toggle lights\n");
	printf("Press both buttons to exit cleanly\n");
	
	//Assign your own functions to be called when events occur
	set_start_pressed_func(&on_start_press);
	set_start_unpressed_func(&on_start_release);
	set_select_pressed_func(&on_select_press);
	set_select_unpressed_func(&on_select_release);
	
	//run forever till the program state changes
	while(get_state() != EXITING){
		usleep(100000);
	}
	
	cleanup_cape();
	return 0;
}



