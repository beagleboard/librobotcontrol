// Button and LED tester for the Robotics Cape
// Pressing either button makes an LED blink
// Hold the start button or ctrl-c to exit cleanly
// James Strawson - 2013

#include <robotics_cape.h>

// If the user holds the start button , exit cleanly
int on_start_press(){
	printf("pressed start\n");
	setGRN(HIGH);
	int i=0;
	do{
		usleep(100000);
		if(get_start_button() == LOW){
			return 0; //user let go before time-out
		}
		i++;
	}while(i<20);
	//user held the button down long enough, exit cleanly
	set_state(EXITING);
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
		setGRN(HIGH);
		if(get_select_button() == LOW){
			setRED(LOW);
		}
		usleep(500000);
		
		if(get_start_button() == LOW){
			setGRN(LOW);
		}
		setRED(HIGH);
		usleep(500000);
	}
	
	cleanup_cape();
	return 0;
}



