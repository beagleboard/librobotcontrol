/**
 * @example    rc_spi_loopback.c
 *
 * This is a test to check read and write operation of the SPI1 bus. For this
 * example to work, connect the MISO and MOSI wires of one of the included 6-pin
 * JST-SH pigtails and plug into either SPI1 socket. The test strings this
 * programs transmits out the MOSI channel will loop back in the MISO channel
 * and be read.
 */

#include <stdio.h>
#include <string.h>
#include <rc/spi.h>

// change these for your platform
// on BB this is equivilant to RC_BB_SPI1_SS1
#define BUS		1
#define SLAVE		0


#define BUS_MODE	SPI_MODE_0
#define SPI_SPEED	24000000

int main()
{
	char* test_str = "Hello World";
	int bytes = strlen(test_str);	// get number of bytes in test string
	uint8_t buf[32];		// read buffer
	int ret;			// return value

	printf("Make sure the MISO and MOSI lines are connected with\n");
	printf("a loopback jumper which is necessary for this test.\n");
	printf("Testing SPI \n\n");
	if(rc_spi_init_auto_slave(BUS, SLAVE, BUS_MODE, SPI_SPEED)){
		return -1;
	}

	// attempt a string send/receive test
	printf("Sending  %d bytes: %s\n", bytes, test_str);

	while(1){
		//ret = rc_spi_transfer(BUS, SLAVE, (uint8_t*)test_str, bytes, buf);
		ret = rc_spi_write(BUS, SLAVE, (uint8_t*)test_str, bytes);
		if(ret<0){
			printf("send failed\n");
			rc_spi_close(SLAVE);
			return -1;
		}
		else printf("sent %d bytes: %s\n",bytes, test_str);
		// read back
		ret = rc_spi_read(BUS, SLAVE, buf, bytes);
		if(ret<0){
			printf("read failed\n");
			rc_spi_close(SLAVE);
			return -1;
		}
		else printf("Received %d bytes: %s\n",ret, (char*)buf);
		rc_usleep(10000);
	}

	rc_spi_close(BUS);
	return 0;
}