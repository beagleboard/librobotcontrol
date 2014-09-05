// Sample code to read optical flow data from ADNS-9800 flow sensor
// Using a BeagleBone Black and Robotics Cape Library
// See adns9800.h for more information and resources
// James Strawson 2014
  
#include <robotics_cape.h>
#include "adns9800.h"

int main(){ 
	int dx, dy;
	
	// initialize the chip and store a file descriptor to spidev0
	int adns_fd = initialize_adns9800();
	if(adns_fd == -1){
		printf("failed to start ADNS9800\n");
		return -1;
	}
	
	// print chip data to make sure all is well
	printf("Prod ID: 0x%x	Should be 0x33\n", adns_read_reg(adns_fd, REG_Product_ID));
	printf("Rev  ID: 0x%x	Should be 0x3\n", adns_read_reg(adns_fd, REG_Revision_ID));
	printf("SROM ID: 0x%x   Should be 0xa4\n", adns_read_reg(adns_fd,REG_SROM_ID));
	printf("\n");
	fflush(stdout);
	
	// now read the values
	while(1){
		adns_read_burst(adns_fd, &dx, &dy);
		printf("\rdx: %5d dy: %5d   ", dx, dy);
		fflush(stdout);
		usleep(5000);
	}
  
    close(adns_fd); 
    return 0; 
} 