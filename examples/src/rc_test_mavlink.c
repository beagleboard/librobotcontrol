/**
 * @file rc_test_mavlink
 * @example rc_test_mavlink
 *
 * @brief      Basic Mavlink UDP heartbeat tester.
 *
 *             Sends a heartbeat packets every second and prints to the screen
 *             when one has been received along with the system identifier of
 *             the device that sent the received heartbeat. Optionally specify
 *             the destination IP address to send to with the -a option,
 *             otherwise messages will be sent to 127.0.0.1 (localhost).
 *             Optionally specify the UDP port which will be used for listening
 *             and sending with the -p option, otherwise RC_MAV_DEFAULT_UDP_PORT
 *             (14551) will be used. Optionally specify the system ID which will
 *             be specified in each sent heartbeat packet to be read by the
 *             listener, otherwise a system ID of 1 will be used.
 *
 * @author     James Strawson & Henry Gaudet
 *
 * @date       1/24/2018
 */

#include <ctype.h> // for isprint()
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h> // to SIGINT signal handler
#include <rc/mavlink_udp.h>

#define LOCALHOST_IP	"127.0.0.1"
#define DEFAULT_SYS_ID	1

static const char* dest_ip;
static uint8_t my_sys_id;
static uint16_t port;
static int running = 0;


static void __print_usage(void)
{
	fprintf(stderr,"usage: rc_test_mavlink [-h] [-a dest_ip_addr] [-p port] [-s sys_id]\n");
	return;
}

static int __parse_args(int argc, char * argv[])
{
	int c,tmp;
	opterr = 0;

	while ((c = getopt(argc, argv, "ha:p:s:")) != -1)
		switch (c)
		{
		case 'h':
			__print_usage();
			exit(0);
			break;
		case 'a':
			dest_ip = optarg;
			break;
		case 'p':
			tmp = atoi(optarg);
			if(tmp<0){
				fprintf(stderr, "UDP port must be greater than 0\n");
				exit(-1);
			}
			if(tmp>UINT16_MAX){
				fprintf(stderr, "UDP port must be less than %d\n", UINT16_MAX);
				exit(-1);
			}
			if(tmp<1024){
				fprintf(stderr, "WARNING: ports less than 1024 are privaledged ports\n");
			}
			port=tmp;
			break;
		case 's':
			tmp=atoi(optarg);
			if(tmp>UINT8_MAX||tmp<0){
				fprintf(stderr, "sys_id must be between 0 and %d\n", UINT8_MAX);
				exit(-1);
			}
			my_sys_id = tmp;
			break;
		case '?':
			if (optopt == 'a' || optopt=='p' || optopt=='s')
				fprintf (stderr, "Option -%c requires an argument.\n", optopt);
			else if (isprint (optopt))
				fprintf (stderr, "Unknown option `-%c'.\n", optopt);
			else
				fprintf (stderr,"Unknown option `\\x%x'.\n",optopt);
			__print_usage();
			exit(-1);
		default:
			__print_usage();
			exit(-1);
		}
	return 0;
}

// called by the rc_mav lib whenever a packet is received
static void __callback_func_any(void)
{
	int sysid = rc_mav_get_sys_id_of_last_msg_any();
	int msg_id = rc_mav_msg_id_of_last_msg();
	printf("received msg_id: %d ", msg_id);
	// TODO uncomment print msg name when this works
	//rc_mav_print_msg_name(msg_id);
	printf(" from sysid: %d \n", sysid);
	return;
}

static void __callback_func_connection_lost(void)
{
	fprintf(stderr,"CONNECTION LOST\n");
	return;
}


// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}

int main(int argc, char * argv[])
{
	// set default options before checking options
	dest_ip=LOCALHOST_IP;
	my_sys_id=DEFAULT_SYS_ID;
	port=RC_MAV_DEFAULT_UDP_PORT;

	// parse arguments
	if(__parse_args(argc,argv)){
		fprintf(stderr,"failed to parse arguments\n");
		return -1;
	}

	printf("run with -h option to see usage and other options\n");
	// inform the user what settings are being used
	printf("\n");
	printf("Initializing with the following settings:\n");
	printf("dest ip addr: %s\n", dest_ip);
	printf("my system id: %d\n", my_sys_id);
	printf("UDP port: %d\n", port);
	printf("\n");

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);

	// initialize the UDP port and listening thread with the rc_mav lib
	if(rc_mav_init(my_sys_id, dest_ip, port,RC_MAV_DEFAULT_CONNECTION_TIMEOUT_US)<0){
		return -1;
	}

	// set the heartbeat callback to print something when receiving
	rc_mav_set_callback_all(__callback_func_any);
	rc_mav_set_callback_connection_lost(__callback_func_connection_lost);
	running=1;
	while(running){
		sleep(1);
		if(rc_mav_send_heartbeat_abbreviated()){
			fprintf(stderr,"failed to send heartbeat\n");
		}
		else{
			printf("sent heartbeat\n");
		}
	}

	// stop listening thread and close UDP port
	printf("closing UDP port\n");
	rc_mav_cleanup();
}