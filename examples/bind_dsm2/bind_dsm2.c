// routine to put a DSM2 satellite receiver into pairing mode
// This also provides an example of GPIO and PINMUX functions
// James Strawson 2014

// DSM satellite receivers are put into bind mode by sending them a sequence of
// pulses right after it receives power and starts up. This program puts the 
// normally UART signal pin into GPIO pulldown mode temporarily, detects when the 
// user unplugs and plugs back in the receiver, then sends the binding pulses. 

// the number of pulses dictates the mode the satellite receiver will request
// the transmitter to use. The transmitter may bind but use a different mode.
// I suggest configuring your radio to use DSMX 11ms fast mode if it allows that.

// 2048 & 1024 indicates 10 or 11 bit resolution of the transmitted channel positions.
// 11ms & 22ms indicates the time period between the transmitter sending frames.
// 11ms is required for transmitters with 8 or more channels.
// 
// Testing done with DX7s, DX6i, DX8, and Orange T-SIX
// 
// Table of Bind Modes
//  pulses      mode        
//   3      DSM2 1024/22ms 
//   5  	DSM2 2048/11ms
//   7  	DSMX 1024/22ms: 
//   9  	DSMx 2048/11ms: 

#include <robotics_cape.h>

// satellite receiver communication pin is P9.11 gpio_0[30]
#define GPIO_PIN 30
#define PINMUX_PATH "/sys/devices/ocp.3/P9_11_pinmux.18/state"
#define PAUSE 115	//microseconds

int main(){
	unsigned int value;
	int i;
	char c = 0; // for reading user input
	// default to dsmx 11ms mode for most applications
	int pulses = 9; 
	int delay = 200000;
	
	// swap pinmux from UART4_RX to GPIO
	FILE *pinmux_fd;
	pinmux_fd = fopen(PINMUX_PATH, "w+");
	if((int)pinmux_fd == -1){
		printf("error opening pinmux\n");
		return -1;
	}
	fprintf(pinmux_fd, "%s", "gpio_pd");
	fflush(pinmux_fd);
	//export GPIO pin to userspace
	if(gpio_export(GPIO_PIN)){
		printf("error exporting gpio pin\n");
		return -1;
	}
	// first set the pin as input (pulldown) to detect when receiver is attached
	gpio_set_dir(GPIO_PIN, INPUT_PIN);
	
	// give user instructions
	printf("\n\nYou must choose which DSM mode to request from your transmitter\n");
	printf("Note that your transmitter may actually bind in a different mode\n");
	printf("depending on how it is configured.\n");
	printf("We suggest option 1 for 6-channel DSM2 radios,\n");
	printf("and option 4 for 7-9 channel DSMX radios\n");
	printf("\n");
	printf("1: DSM2 10-bit 22ms framerate\n");
	printf("2: DSM2 11-bit 11ms framerate\n"); 
	printf("3: DSMX 10-bit 22ms framerate\n"); 
	printf("4: DSMX 11-bit 11ms framerate\n"); 
	printf("5: Orange 10-bit 22ms framerate\n");
	printf("\n"); 
	printf("Enter mode 1-5: ");
	
	// wait for user input
enter:
	c = getchar();
 
	switch(c){
		case '1':
			pulses = 3;
			break;
		case '2':
			pulses = 5;
			break;
		case '3':
			pulses = 7;
			break;
		case '4':
			pulses = 9;
			break;
		case '5':
			pulses = 9;
			delay = 50000;
			break;
		case '\n':
			goto enter;
			break;
		default:
			printf("incorrect mode number\n");
			getchar();
			goto enter;
			break;
	}
		
    printf("Using mode %c\n", c);

	// wait for user to hit enter before continuing
	printf("\nDisconnect your dsm2 satellite receiver if it is still connected\n");
	printf("Plug it into the cape quickly and firmly to begin binding.\n");
	
	// wait for the receiver to be disconnected
	value = 1;
	while(value==1){ //pin will go low when disconnected
		gpio_get_value(GPIO_PIN, &value);
	}
	usleep(100000);
	
	//wait for the receiver to be plugged in
	//receiver will pull pin up when connected
	while(value==0){ 
		gpio_get_value(GPIO_PIN, &value);
	}
	
	// start pairing packet
	gpio_set_dir(GPIO_PIN, OUTPUT_PIN);
	gpio_set_value(GPIO_PIN, HIGH);
	
	// wait as long as possible before sending pulses
	// in case the user plugs in the receiver slowly at an angle
	// which would delay the power pin from connecting 
	usleep(delay); 
	
	for(i=0; i<pulses; i++){
		gpio_set_value(GPIO_PIN, LOW);
		usleep(PAUSE);
		gpio_set_value(GPIO_PIN, HIGH);
		usleep(PAUSE);
	}
	
	usleep(1000000);
	
	// swap pinmux back to uart
	fprintf(pinmux_fd, "%s", "uart");
	fflush(pinmux_fd);
	fclose(pinmux_fd);
	
	// all done
	printf("Your receiver should now be blinking. If not try again.\n");
	printf("Now turn on your transmitter in bind mode.\n");
	printf("Use test_dsm2 to confirm functionality.\n\n");
	return 0;
}