/*******************************************************************************
* rc_spi_loopback.c
*
* This is a test to check read and write operation of the SPI1 bus. 
* For this example to work, connect the MISO and MOSI wires of one of the 
* included 6-pin JST-SH pigtails and plug into either SPI1 socket.
* The test strings this programs transmits out the MOSI channel will loop back
* in the MISO channel and be read.
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

#define SLAVE 		1
#define SPI_MODE	SPI_MODE_CPOL0_CPHA0
#define SPI_SPEED	24000000

int main(){
	char test_char = 0x42;
	char test_str[] = "Hello World";
	int bytes = strlen(test_str); // get number of bytes in test string
	char buf[bytes];	// read buffer
	int ret;			// return value

	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}

	printf("Testing SPI \n\n");
	if(rc_spi_init(SS_MODE_AUTO, SPI_MODE, SPI_SPEED, SLAVE)){
		printf("Failed to rc_spi_init1\n");
		rc_cleanup();
		return -1;
	}

	// attempt a string send/receive test
	printf("Sending  %d bytes: %s\n", bytes, test_str);
	ret=rc_spi_transfer(test_str, bytes, buf, SLAVE);

	// print error or response
	if(ret<0){
		printf("send failed\n");
		goto cleanup;
	}
	else printf("Received %d bytes: %s\n",ret, buf);
	
	// attempt a single byte send/receive test
	printf("Sending byte:      0x%x\n", test_char);
	ret = rc_spi_read_reg_byte(test_char, SLAVE); 

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
	rc_spi_close(SLAVE);
	rc_cleanup();
	return 0;
}
