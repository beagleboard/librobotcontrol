// test_mavlink.c  -  James Strawson 2014

// basic example of sending and reading mavlink packets over UDP
// This program sends the heartbeat pack at 1hz, IMU data at 20hz
// and prints any received packets to the console.
// This this heavily-based on mavlink_udp.c from the mavlink website 

#include <robotics_cape.h>
#define DEFAULT_MAV_ADDRESS "192.168.7.1"

int sock;
struct sockaddr_in gcAddr;

// send IMU roll, pitch, yaw data as attitude packet
int send_imu_data(){
	mpudata_t mpu; //struct to read IMU data into
	uint8_t buf[MAV_BUF_LEN];
	mavlink_message_t msg;
	uint16_t len;
	if (mpu9150_read(&mpu) == 0) {
		//Send attitude packet
		memset(buf, 0, MAV_BUF_LEN);
		mavlink_msg_attitude_pack(1, 200, &msg, microsSinceEpoch(), 
											mpu.fusedEuler[VEC3_X], 
											mpu.fusedEuler[VEC3_Y],
											mpu.fusedEuler[VEC3_Z], 
											0, 0, 0); //set gyro rates to 0 for simplicity
		len = mavlink_msg_to_send_buffer(buf, &msg);
		sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
	}
	return 0; 
}

// print out any mavlink packets that come in
void* mavlink_listener(void* ptr){
	ssize_t recsize;
	socklen_t fromlen;
	uint8_t buf[MAV_BUF_LEN];
	int i;
	
	while(get_state() != EXITING){
		recsize = recvfrom(sock, (void *)buf, MAV_BUF_LEN, 0, (struct sockaddr *)&gcAddr, &fromlen);
		if (recsize > 0){
			// Something received - print out all bytes and parse packet
			mavlink_message_t msg;
			mavlink_status_t status;
			for (i = 0; i < recsize; ++i){
				if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)){
					// Packet received, do something
					printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
				}
			}
		}
		else{
			printf("%d ",recsize);
			printf("error in recvfrom\n");
		}
		usleep(10000);
	}
	return NULL;
}

int main(int argc, char* argv[]){
	// start the cape an IMU as usual
	initialize_cape();
	signed char orientation[9] = ORIENTATION_FLAT; 
	initialize_imu(20, orientation); //run IMU slowly at 20hz to send to ground control
	
	// see if the user gave an IP as argument
	char target_ip[100];
	if (argc == 2){
		strcpy(target_ip, argv[1]);
    }
	else{
		strcpy(target_ip, DEFAULT_MAV_ADDRESS);
	}
	
	// open a udp port. 
	// sock and gcAddr are global variables needed to send and receive
	gcAddr = initialize_mavlink_udp(target_ip, &sock);
	
	// Start the IMU interrupt handler sending attitude packets
	set_imu_interrupt_func(&send_imu_data);
	printf("Sending Attitude Packets\n");
	
	// start a thread listening for incoming packets
	pthread_t  mav_listen_thread;
	pthread_create(&mav_listen_thread, NULL, mavlink_listener, (void*) NULL);
	printf("Listening for Packets\n");
		
	// now use the main thread to send heartbeat packets until the program exits
	uint8_t buf[MAV_BUF_LEN];
	mavlink_message_t msg;
	uint16_t len;
	printf("Sending Heartbeat Packets\n");
	while(get_state()!= EXITING){
		memset(buf, 0, MAV_BUF_LEN);
		mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		sleep(1); 	// send heartbeat about once per second
	}
	
	// close the socket and exit cleanly
	close(sock);	
	cleanup_cape();
	return 0;
}
