/*******************************************************************************
* spi_loopback.c
*
* This is a test to check read and write operation of the SPI1 bus. 
* For this example to work, connect the MISO and MOSI wires of one of the 
* included 6-pin JST-SH pigtails and plug into either SPI1 socket.
* The test strings this programs transmits out the MOSI channel will loop back
* in the MISO channel and be read.
*******************************************************************************/

#include "../../libraries/usefulincludes.h"
#include "../../libraries/roboticscape.h"

#define SPI_MODE	0
#define SPI_SPEED	24000000

int main(){
	char test_char = 0x42;
	char test_str[] = "Hello World";
	int bytes = strlen(test_str); // get number of bytes in test string
	char buf[bytes]; 	// read buffer
	int ret; 			// return value
	
	// Initialization
	initialize_cape();
	printf("Testing SPI1 \n\n");
	if(initialize_spi1(SPI_MODE,SPI_SPEED)){
		printf("Failed to initialize_spi1\n");
		cleanup_cape();
		return -1;
	}

	// attempt a string send/receive test
	printf("Sending  %d bytes: %s\n", bytes, test_str);
	ret=spi1_transfer(test_str, bytes, buf);

	// print error or response
	if(ret<0){
		printf("send failed\n");
		goto cleanup;
	}
	else printf("Received %d bytes: %s\n",ret, buf);
	
	// attempt a single byte send/receive test
	printf("Sending byte:      0x%x\n", test_char);
	ret = spi1_read_reg_byte(test_char); 

	// print error or response
	if(ret<0){
		printf("ERROR: failed to send/recieve one byte");
		goto cleanup;
	}
	else printf("Received:          0x%x\n", ret); 

	// if there was no match, alert the user
	if(ret != test_char){
		printf("\nThe wrong data was received but no errors detected\n");
		printf("Likely the MISO and MOSI lines are not connected with\n");
		printf("a loopback jumper which is necessary for this test.\n");
	}
	else printf("Success!\n");

cleanup:
	close_spi1();
	cleanup_cape();
	return 0;
}
