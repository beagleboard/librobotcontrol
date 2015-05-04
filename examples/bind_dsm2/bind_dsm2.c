// routine to put a DSM2 satellite receiver into pairing mode
// This also provides an example of GPIO and PINMUX functions
// James Strawson 2014

// the number of pulses dictates the mode the radio will be put in
// Testing done with DX7s, DX6i, DX8, and Orange T-SIX
// 
// Table of Bind Modes
// pulses    mode           orange       dx6          dx7            dx8   
//   3  DSM2 1024/22ms      works       works      6 ch work      6 ch work
//   5  DSM2 2048/11ms:     works       works        no bind	  no bind
//   7  DSMX 22ms:          works   ----  dx6 dx7 dx8 bind but can't read    -----------
//   9  DSMx 11ms: 			works	----  bind but no read, also says 22ms    -----------

#include <robotics_cape.h>

// satellite receiver communication pin is P9.11 gpio_0[30]
#define GPIO_PIN 30
#define PINMUX_PATH "/sys/devices/ocp.3/P9_11_pinmux.18/state"
#define PAUSE 115	//microseconds


int main(){
	unsigned int value;
	int i;
	int pulses = 3;
	
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
	
	// wait for user to hit enter before continuing
	printf("\nDisconnect your dsm2 satellite receiver and push back in to continue\n");
	
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
	
	//send pairing packet
	gpio_set_dir(GPIO_PIN, OUTPUT_PIN);
	gpio_set_value(GPIO_PIN, HIGH);
	usleep(90000);
	
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
	printf("Now put your transmitter into bind mode.\n");
	printf("Use test_dsm2 to confirm functionality.\n\n");
	return 0;
}