/*******************************************************************************
* test_gps.c
*
* James Strawson 2016
* INCOMPLETE!!!! Work in progress!!!
*******************************************************************************/

#include "../../libraries/roboticscape-usefulincludes.h"
#include "../../libraries/roboticscape.h"


int baudrate_prompt(){
	printf("\n\n");
	printf("all GPS units talk over the UART serial port at a particular BAUD rate\n");
	printf("please select from this list of common baudrates.\n");
	printf("alternatively give the baudrate as a command line argument\n");
	printf("\n");
	printf("1) 4800\n");
	printf("2) 9600\n");
	printf("3) 19200\n");
	printf("4) 38400\n");
	printf("5) 57600\n");
	printf("6) 115200\n");
	printf("\n");

	char c = getchar();
	switch(c){
	case '1':
		return 4800;
	case '2':
		return 9600;
	case '3':
		return 19200;
	case '4':
		return 38400;
	case '5':
		return 57600;
	case '6':
		return 115200;
	default:
		printf("invalid selection, quitting\n");
	}
	return -1;
}

int main(int argc, char *argv[]){
	int baud;

	// Parse arguments
	if(argc == 1){ //argc==2 actually means one argument given
		baud = baudrate_prompt();
		if(baud < 0) return -1;
	}
	// if one argument is given, assume it's a baudrate
	else if(argc == 2) baud = atoi(argv[1]);
	else{
		printf("too many arguments\n");
		return -1;
	}
	
	// Initialization
	if(initialize_roboticscape() < 0){
		printf("failed to initialize cape\n");
		return -1;
	}

	if(initialize_gps(baud) < 0){
		printf("failed to initialize_gps\n");
		return -1;
	}

	// printf print until user exists
	while(rc_get_state()!=EXITING){
		usleep(500000);
	}

	stop_gps_service();
	cleanup_roboticscape();
	return 0;
}
