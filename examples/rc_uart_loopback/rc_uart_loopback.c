/*******************************************************************************
* rc_uart_loopback.c
*
* This is a test to check read and write operation of UART buses. 
* For this example to work, connect the RX and TX wires of one of the included 
* 4-pin JST-SH pigtails and plug into the UART1 or UART5 headers. You may also 
* elect to test UART0 on the debug header or UART2 on the GPS header.
* The test strings this programs transmits will then loopback to the RX channel.
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

#define BUF_SIZE 	32
#define TIMEOUT_S 	0.5
#define BAUDRATE 	115200

void print_usage(){
	printf("please give either 0,1,2 or 5, as an argument corresponding\n");
	printf("to the UART header on the Robotics Cape you wish to test\n");
}

int main(int argc, char *argv[]){
	char test_str[] = "Hello World";
	int bytes = strlen(test_str); // get number of bytes in test string
	char buf[BUF_SIZE];
	int ret; // return value
	int bus; // which bus to use
	
	// Parse arguments
	if(argc!=2){ //argc==2 actually means one argument given
		print_usage();
		return -1;
	}
	else bus = atoi(argv[1]);
	
	if(!(bus==0||bus==1||bus==2||bus==5)){
		print_usage();
		return -1;
	}
	
	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}
	
	printf("\ntesting UART bus %d\n\n", bus);
	if(rc_uart_init(bus, BAUDRATE, TIMEOUT_S)){
		printf("Failed to rc_uart_init%d\n", bus);
		rc_cleanup();
		return -1;
	}
	
	// Flush and Write
	printf("Sending  %d bytes: %s \n", bytes, test_str);
	rc_uart_flush(bus);
	rc_uart_send_bytes(bus, bytes, &test_str[0]);
	
	// Read 
	memset(buf, 0, BUF_SIZE);
	ret = rc_uart_read_bytes(bus, bytes, &buf[0]);
	if(ret<0) printf("Error reading bus\n");
	else if(ret==0)printf("timeout reached, %d bytes read\n", ret);
	else printf("Received %d bytes: %s \n", ret, buf);
	
	// close up
	rc_uart_close(bus);
	rc_cleanup();
	return 0;
}
