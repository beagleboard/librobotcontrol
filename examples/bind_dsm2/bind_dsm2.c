// routine to put a DSM2 satellite receiver into pairing mode
// This also provides an example of GPIO and PINMUX functions
// James Strawson 2014

#include <robotics_cape.h>

// satellite receiver communication pin is P9.11 gpio_0[30]
#define GPIO_PIN 30
#define PINMUX_PATH "/sys/devices/ocp.3/P9_11_pinmux.18/state"
#define PAUSE 115	//microseconds


int main(){
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
	printf("\nDisconnect your dsm2 satellite receiver and press enter to continue\n");
	while( getchar() != '\n' );
	
	//wait for the receiver to be plugged in
	printf("Now reconnect to put the receiver into bind mode\n");
	unsigned int value = 0;
	while(value==0){ //receiver will pull pin up when connected
		gpio_get_value(GPIO_PIN, &value);
	}
	
	//send pairing packet
	gpio_set_dir(GPIO_PIN, OUTPUT_PIN);
	gpio_set_value(GPIO_PIN, HIGH);
	usleep(90000);
	gpio_set_value(GPIO_PIN, LOW);
	usleep(PAUSE);
	gpio_set_value(GPIO_PIN, HIGH);
	usleep(PAUSE);
	gpio_set_value(GPIO_PIN, LOW);
	usleep(PAUSE); 
	gpio_set_value(GPIO_PIN, HIGH);
	usleep(PAUSE);
	gpio_set_value(GPIO_PIN, LOW);
	usleep(PAUSE);
	gpio_set_value(GPIO_PIN, HIGH);
	usleep(PAUSE);
	gpio_set_value(GPIO_PIN, LOW);
	usleep(PAUSE);
	gpio_set_value(GPIO_PIN, HIGH);
	usleep(1000000);
	
	// swap pinmux back to uart
	fprintf(pinmux_fd, "%s", "uart");
	fflush(pinmux_fd);
	fclose(pinmux_fd);
	
	// all done
	printf("Finished. Use test_dsm2 to confirm functionality.\n");
	return 0;
}