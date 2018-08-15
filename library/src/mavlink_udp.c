/**
 * @file mavlink_udp.c
 *
 * @brief      C interface for communicating with mavlink over UDP in
 *             Linux/Windows
 *
 *             Uses common mavlink v2 packets generated from official mavlink
 *             source (https://github.com/mavlink/mavlink). Also see
 *             mavlink_udp_helpers.h for additional helper functions for most
 *             commonly used packets.
 *
 * @author     James Strawson & Henry Gaudet
 *
 * @date       1/24/2018
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h> // for close()
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <arpa/inet.h>   // Sockets & networking include <sys/types.h> include <sys/socket.h> include <unistd.h> include
#include <string.h>

#include <rc/pthread.h>
#include <rc/mavlink_udp.h>

#define BUFFER_LENGTH			512 // common networking buffer size
#define MAX_UNIQUE_MSG_TYPES		256
#define MAX_PENDING_CONNECTIONS		32
#define LOCALHOST_IP			"127.0.0.1"
#define CONNECTION_TIMEOUT_US_MIN	200000


// connection stuff
static int init_flag=0;
static int sock_fd;
static int current_port;
static struct sockaddr_in my_address ;
static struct sockaddr_in dest_address;
static uint8_t system_id;
struct timeval rcv_timeo;

// callbacks
static void (*callbacks[MAX_UNIQUE_MSG_TYPES])(void);
static void (*callback_all)(void); // called when any packet arrives
static void (*connection_lost_callback)(void);

// flags and info populated by the listening thread
static int received_flag[MAX_UNIQUE_MSG_TYPES];
static int new_msg_flag[MAX_UNIQUE_MSG_TYPES];
static uint64_t connection_timeout_us_current;
static uint64_t us_of_last_msg[MAX_UNIQUE_MSG_TYPES];
static uint64_t us_of_last_msg_any;
static int msg_id_of_last_msg;
static uint8_t sys_id_of_last_msg;
static mavlink_message_t messages[MAX_UNIQUE_MSG_TYPES];
rc_mav_connection_state_t connection_state;

// thread startup and shutdown flags
static pthread_t listener_thread;
static int shutdown_flag=0;
static int listening_flag=0;
//static int listening_init_flag=0;


// private local function declarations;
static uint64_t __us_since_boot();
static int __address_init(struct sockaddr_in* address, const char* dest_ip, uint16_t port);
int __get_msg_common_checks(int msg_id);


// Returns the number of nanoseconds since boot using system CLOCK_MONOTONIC
// This function itself takes about 1100ns to complete on a BBB at 1ghz under ideal
static uint64_t __us_since_boot()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ((uint64_t)ts.tv_sec*1000000)+(ts.tv_nsec/1000);
}

// background thread for handling packets
static void* __listen_thread_func(__attribute__((unused)) void* ptr)
{
	int i;
	uint64_t time;
	ssize_t num_bytes_rcvd;;
	uint8_t buf[BUFFER_LENGTH];
	socklen_t addr_len = sizeof(my_address);
	mavlink_message_t msg;
	mavlink_status_t parse_status;

	#ifdef DEBUG
	printf("beginning of __listen_thread_func thread\n");
	#endif

	// parse packets as they come in until listening flag set to 0
	listening_flag=1;
	while (shutdown_flag==0){
		memset(buf, 0, BUFFER_LENGTH);
		num_bytes_rcvd = recvfrom(sock_fd, buf, BUFFER_LENGTH, 0, (struct sockaddr *) &my_address, &addr_len);

		// check for timeout
		if(num_bytes_rcvd <= 0){
			if (errno == EAGAIN || errno == EWOULDBLOCK){
				// check last message time > MESSAGE_TIMEOUT then throw warning no heartbeat rcvd
				if((__us_since_boot()-us_of_last_msg_any) > connection_timeout_us_current){
					if(connection_state==MAV_CONNECTION_ACTIVE && connection_lost_callback!=NULL){
						connection_state = MAV_CONNECTION_LOST;
						connection_lost_callback();
					}
				}
				continue;
			}
		}
		// do mavlink's silly byte-wise parsing method
		for(i=0; i<num_bytes_rcvd; ++i){
			// parse on channel 0 (MAVLINK_COMM_0)
			if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &parse_status)){
				#ifdef DEBUG
				printf("\nReceived packet: SYSID: %d, MSG ID: %d\n", msg.sysid, msg.msgid);
				#endif
				// update timestamps and received flag
				time = __us_since_boot();
				us_of_last_msg[msg.msgid]=time;
				us_of_last_msg_any = time;
				received_flag[msg.msgid] = 1;
				new_msg_flag[msg.msgid] = 1;
				sys_id_of_last_msg=msg.sysid;
				msg_id_of_last_msg=msg.msgid;
				connection_state = MAV_CONNECTION_ACTIVE;

				// save local copy of message
				messages[msg.msgid]=msg;

				// run the generic callback
				if(callback_all!=NULL) callback_all();

				// run the msg-specific callback
				if(callbacks[msg.msgid]!=NULL) callbacks[msg.msgid]();
			}
		}
	}

	#ifdef DEBUG
	printf("exiting __listen_thread_func thread\n");
	#endif

	return 0;
}


// configures sockaddr_in struct for UDP port
int __address_init(struct sockaddr_in* address, const char* dest_ip, uint16_t port)
{
	// sanity check
	if(address == NULL || port < 1){
		fprintf(stderr, "ERROR: in __address_init: received NULL address struct\n");
		return -1;
	}
	memset((char*) address, 0, sizeof address);
	address->sin_family = AF_INET;
	// convert port from host to network byte order
	address->sin_port = htons(port);
	address->sin_addr.s_addr = ((long)dest_ip==0) ? htonl(INADDR_ANY) : inet_addr(dest_ip);
	return 0;
}

// common checks done at beginning of rc_mav_get_msg and all helper functions
int __get_msg_common_checks(int msg_id)
{
	if(init_flag==0){
		fprintf(stderr, "ERROR getting message, call rc_mav_init first\n");
		return -1;
	}
	if(msg_id<0 || msg_id>MAX_UNIQUE_MSG_TYPES){
		fprintf(stderr,"ERROR: getting message, msg_id out of bounds\n");
		return -1;
	}
	if(received_flag[msg_id]==0){
		fprintf(stderr,"ERROR: getting message, haven't received packet with specified msg_id\n");
		return -1;
	}
	// set new_msg_flag to indicate the message has been read
	new_msg_flag[msg_id]=0;
	return 0;
}


int rc_mav_init(uint8_t sysid, const char* dest_ip, uint16_t port, uint64_t connection_timeout_us)
{
	int i;

	// sanity checks
	if(init_flag!=0){
		fprintf(stderr, "ERROR, in rc_mav_init, already initialized!\n");
		return -1;
	}
	if(dest_ip==NULL){
		fprintf(stderr, "ERROR: in rc_mav_init received NULL dest_ip string\n");
		return -1;
	}
	if(connection_timeout_us<CONNECTION_TIMEOUT_US_MIN){
		fprintf(stderr,"ERROR in rc_mav_init, connection_timeout_us must be >%d\n", CONNECTION_TIMEOUT_US_MIN);
		return -1;
	}

	// set the connection state as waiting early
	// this will be change by listening thread
	connection_state=MAV_CONNECTION_WAITING;
	connection_timeout_us_current = connection_timeout_us;
	// save port globally for other functions to use
	current_port = port;

	// set all the callback pointers to something sane
	callback_all = NULL;
	connection_lost_callback = NULL;
	for(i=0;i<MAX_UNIQUE_MSG_TYPES;i++) callbacks[i] = NULL;

	// set up all global variables to default values
	for(i=0;i<MAX_UNIQUE_MSG_TYPES;i++){
		received_flag[i]=0;
		new_msg_flag[i]=0;
		us_of_last_msg[i]=UINT64_MAX;
	}
	memset(&messages,i,sizeof(mavlink_message_t));
	us_of_last_msg_any=UINT64_MAX;
	msg_id_of_last_msg=-1;

	// open socket for UDP packets
	if((sock_fd=socket(AF_INET, SOCK_DGRAM, 0)) < 0){
		perror("ERROR: in rc_mav_init: ");
		return -1;
	}

	// fill out rest of sockaddr_in struct
	if(__address_init(&my_address, 0, current_port) != 0){
		fprintf(stderr, "ERROR: in rc_mav_init: couldn't set local address\n");
		return -1;
	}
	// socket timeout should be half the connection timeout detection window
	rcv_timeo.tv_sec = (connection_timeout_us/2)/1000000;
	rcv_timeo.tv_usec = (connection_timeout_us/2)%1000000;
	if(setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, (struct timeval *)&rcv_timeo, sizeof (struct timeval)) < 0){
		perror("ERROR: in rc_mav_init: ");
		return -1;
	}

	// bind address to listening port
	if(bind(sock_fd, (struct sockaddr *) &my_address, sizeof my_address) < 0){
		perror("ERROR: in rc_mav_init: ");
		return -1;
	}

	// set destination address
	if(__address_init(&dest_address, dest_ip, current_port) != 0){
		fprintf(stderr, "ERROR: in rc_mav_init: couldn't set destination address");
		return -1;
	}

	// signal initialization finished
	init_flag=1;
	system_id=sysid;

	// spawn listener thread
	if(rc_pthread_create(&listener_thread, __listen_thread_func, NULL, SCHED_OTHER, 0) < 0){
		fprintf(stderr,"ERROR: in rc_mav_init, couldn't start listening thread\n");
		return -1;
	}

	return 0;
}

int rc_mav_set_dest_ip(const char* dest_ip)
{
	return __address_init(&dest_address,dest_ip,current_port);
}


int rc_mav_set_system_id(uint8_t sys_id)
{
	system_id = sys_id;
	return 0;
}


int rc_mav_cleanup(void)
{
	int ret = 0;
	if(init_flag==0 || listening_flag==0){
		fprintf(stderr, "WARNING, trying to cleanup mavlink listener when it's not running\n");
		return -1;
	}
	shutdown_flag=1;
	listening_flag=0;

	// wait for thread to join
	ret=rc_pthread_timed_join(listener_thread, NULL, 1.5);
	if(ret==1) fprintf(stderr,"WARNING in rc_mav_cleanup, joining thread timed out\n");

	close(sock_fd);
	init_flag=0;
	return ret;
}


int rc_mav_send_msg(mavlink_message_t msg)
{
	uint8_t buf[BUFFER_LENGTH];
	int msg_len, bytes_sent;

	if(init_flag == 0){
		fprintf(stderr, "ERROR: in rc_mav_send_msg, socket not initialized\n");
		return -1;
	}

	memset(buf, 0, BUFFER_LENGTH);
	msg_len = mavlink_msg_to_send_buffer(buf, &msg);
	if(msg_len < 0){
		fprintf(stderr, "ERROR: in rc_mav_send_msg, unable to pack message for sending\n");
		return -1;
	}
	bytes_sent = sendto(sock_fd, buf, msg_len, 0, (struct sockaddr *) &dest_address,
							sizeof dest_address);
	if(bytes_sent != msg_len){
		perror("ERROR in rc_mav_send_msg failed to write to UDP socket");
		return -1;
	}
	return 0;
}

int rc_mav_is_new_msg(int msg_id)
{
	if(init_flag==0){
		fprintf(stderr, "ERROR in rc_mav_is_new_msg, call rc_mav_init first\n");
		return -1;
	}
	if(msg_id<0 || msg_id>MAX_UNIQUE_MSG_TYPES){
		fprintf(stderr,"ERROR: in rc_mav_is_new_msg, msg_id out of bounds\n");
		return -1;
	}
	return received_flag[msg_id];
}

int rc_mav_get_msg(int msg_id, mavlink_message_t* msg)
{
	if(__get_msg_common_checks(msg_id)) return -1;
	// write data back to user's pointer
	*msg=messages[msg_id];
	return 0;
}


int rc_mav_set_callback(int msg_id, void (*func)(void))
{
	if(msg_id<0 || msg_id>MAX_UNIQUE_MSG_TYPES){
		fprintf(stderr,"ERROR: in rc_mav_set_callback, msg_id out of bounds\n");
		return -1;
	}
	if(func==NULL){
		fprintf(stderr,"ERROR: in rc_mav_set_callback, received NULL pointer\n");
		return -1;
	}
	callbacks[msg_id]=func;
	return 0;
}


int rc_mav_set_callback_all(void (*func)(void))
{
	if(init_flag==0){
		fprintf(stderr, "ERROR in rc_mav_set_callback_all, call rc_mav_init first\n");
		return -1;
	}
	if(func==NULL){
		fprintf(stderr,"ERROR: in rc_mav_set_callback_all, received NULL pointer\n");
		return -1;
	}
	callback_all=func;
	return 0;
}

int rc_mav_set_callback_connection_lost(void (*func)(void))
{
	if(init_flag==0){
		fprintf(stderr, "ERROR in rc_mav_set_callback_connection_lost, call rc_mav_init first\n");
		return -1;
	}
	if(func==NULL){
		fprintf(stderr,"ERROR: in rc_mav_set_callback_connection_lost, received NULL pointer\n");
		return -1;
	}
	connection_lost_callback=func;
	return 0;
}

rc_mav_connection_state_t rc_mav_get_connection_state()
{
	return connection_state;
}


uint8_t rc_mav_get_sys_id_of_last_msg(int msg_id)
{
	if(init_flag==0){
		fprintf(stderr, "ERROR in get_sys_id_of_last_msg, call rc_mav_init first\n");
		return -1;
	}
	if(msg_id<0 || msg_id>MAX_UNIQUE_MSG_TYPES){
		fprintf(stderr,"ERROR: in rc_mav_get_sys_id_of_last_msg, msg_id out of bounds\n");
		return -1;
	}
	return messages[msg_id].sysid;
}

uint8_t rc_mav_get_sys_id_of_last_msg_any(void)
{
	if(init_flag==0){
		fprintf(stderr, "ERROR in rc_mav_get_sys_id_of_last_msg_any, call rc_mav_init first\n");
		return -1;
	}
	return sys_id_of_last_msg;
}

int64_t rc_mav_ns_since_last_msg(int msg_id)
{
	if(msg_id<0 || msg_id>MAX_UNIQUE_MSG_TYPES){
		fprintf(stderr,"ERROR: in rc_mav_ns_since_last_msg, msg_id out of bounds\n");
		return -1;
	}
	if(init_flag==0){
		fprintf(stderr, "ERROR in rc_mav_ns_since_last_msg, call rc_mav_init first\n");
		return -1;
	}
	// if no packet received
	if(us_of_last_msg[msg_id]==UINT64_MAX) return -1;
	// else get current time and subtract;
	return __us_since_boot()-us_of_last_msg[msg_id];
}

int64_t rc_mav_ns_since_last_msg_any(void)
{
	if(init_flag==0){
		fprintf(stderr, "ERROR in rc_mav_ns_since_last_msg_any, call rc_mav_init first\n");
		return -1;
	}
	// if no packet received
	if(us_of_last_msg_any==UINT64_MAX) return -1;
	// else get current time and subtract;
	return __us_since_boot()-us_of_last_msg_any;
}

int rc_mav_msg_id_of_last_msg(void)
{
	if(init_flag==0){
		fprintf(stderr, "ERROR in rc_mav_msg_id_of_last_msg, call rc_mav_init first\n");
		return -1;
	}
	return msg_id_of_last_msg;
}




int rc_mav_send_heartbeat_abbreviated(void)
{
	// sanity check
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(system_id, MAV_COMP_ID_ALL, &msg, 0, 0, 0, 0, 0);
	if(rc_mav_send_msg(msg)){
		fprintf(stderr, "ERROR: in rc_mav_send_heartbeat_abbreviated, failed to send\n");
		return -1;
	}
	return 0;
}


int rc_mav_send_heartbeat(uint32_t custom_mode, uint8_t type, uint8_t autopilot, uint8_t base_mode, uint8_t system_status)
{
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(system_id, MAV_COMP_ID_ALL, &msg, type, autopilot, base_mode, custom_mode, system_status);
	return rc_mav_send_msg(msg);
}

int rc_mav_get_heartbeat(mavlink_heartbeat_t* data)
{
	if(__get_msg_common_checks(MAVLINK_MSG_ID_HEARTBEAT)) return -1;
	mavlink_msg_heartbeat_decode(&messages[MAVLINK_MSG_ID_HEARTBEAT], data);
	return 0;
}


int rc_mav_send_attitude(
	float roll,
	float pitch,
	float yaw,
	float rollspeed,
	float pitchspeed,
	float yawspeed)
{
	mavlink_message_t msg;
	mavlink_msg_attitude_pack(system_id, MAV_COMP_ID_ALL, &msg, __us_since_boot()/1000, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed);
	return rc_mav_send_msg(msg);
}

int rc_mav_get_attitude(mavlink_attitude_t* data)
{
	if(__get_msg_common_checks(MAVLINK_MSG_ID_ATTITUDE)) return -1;
	mavlink_msg_attitude_decode(&messages[MAVLINK_MSG_ID_ATTITUDE], data);
	return 0;
}


int rc_mav_send_attitude_quaternion(
	float q1,
	float q2,
	float q3,
	float q4,
	float rollspeed,
	float pitchspeed,
	float yawspeed)
{
	mavlink_message_t msg;
	mavlink_msg_attitude_quaternion_pack(system_id, MAV_COMP_ID_ALL, &msg, __us_since_boot()/1000, q1, q2, q3, q4, rollspeed, pitchspeed, yawspeed);
	return rc_mav_send_msg(msg);
}

int rc_mav_get_attitude_quaternion(mavlink_attitude_quaternion_t* data)
{
	if(__get_msg_common_checks(MAVLINK_MSG_ID_ATTITUDE_QUATERNION)) return -1;
	mavlink_msg_attitude_quaternion_decode(&messages[MAVLINK_MSG_ID_ATTITUDE_QUATERNION], data);
	return 0;
}


int rc_mav_send_local_position_ned(
	float x,
	float y,
	float z,
	float vx,
	float vy,
	float vz)
{
	mavlink_message_t msg;
	mavlink_msg_local_position_ned_pack(system_id, MAV_COMP_ID_ALL, &msg, __us_since_boot()/1000, x, y, z, vx, vy, vz);
	return rc_mav_send_msg(msg);
}

int rc_mav_get_local_position_ned(mavlink_local_position_ned_t* data)
{
	if(__get_msg_common_checks(MAVLINK_MSG_ID_LOCAL_POSITION_NED)) return -1;
	mavlink_msg_local_position_ned_decode(&messages[MAVLINK_MSG_ID_LOCAL_POSITION_NED], data);
	return 0;
}


int rc_mav_send_global_position_int(
	int32_t lat,
	int32_t lon,
	int32_t alt,
	int32_t relative_alt,
	int16_t vx,
	int16_t vy,
	int16_t vz,
	uint16_t hdg)
{
	mavlink_message_t msg;
	mavlink_msg_global_position_int_pack(system_id, MAV_COMP_ID_ALL, &msg, __us_since_boot()/1000, lat, lon, alt,relative_alt, vx, vy, vz, hdg);
	return rc_mav_send_msg(msg);
}

int rc_mav_get_global_position_int(mavlink_global_position_int_t* data)
{
	if(__get_msg_common_checks(MAVLINK_MSG_ID_GLOBAL_POSITION_INT)) return -1;
	mavlink_msg_global_position_int_decode(&messages[MAVLINK_MSG_ID_GLOBAL_POSITION_INT], data);
	return 0;
}


int rc_mav_send_set_position_target_local_ned(
	float x,
	float y,
	float z,
	float vx,
	float vy,
	float vz,
	float afx,
	float afy,
	float afz,
	float yaw,
	float yaw_rate,
	uint16_t type_mask,
	uint8_t target_system,
	uint8_t target_component,
	uint8_t coordinate_frame)
{
	mavlink_message_t msg;
	mavlink_msg_set_position_target_local_ned_pack(system_id, MAV_COMP_ID_ALL, &msg, __us_since_boot()/1000, target_system, target_component, coordinate_frame, type_mask, x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate);
	return rc_mav_send_msg(msg);
}

int rc_mav_get_set_position_target_local_ned(mavlink_set_position_target_local_ned_t* data)
{
	if(__get_msg_common_checks(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED)) return -1;
	mavlink_msg_set_position_target_local_ned_decode(&messages[MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED], data);
	return 0;
}


int rc_mav_send_set_position_target_global_int(
	int32_t lat_int,
	int32_t lon_int,
	float alt,
	float vx,
	float vy,
	float vz,
	float afx,
	float afy,
	float afz,
	float yaw,
	float yaw_rate,
	uint16_t type_mask,
	uint8_t target_system,
	uint8_t target_component,
	uint8_t coordinate_frame)
{
	mavlink_message_t msg;
	mavlink_msg_set_position_target_global_int_pack(system_id, MAV_COMP_ID_ALL, &msg, __us_since_boot()/1000, target_system, target_component, coordinate_frame, type_mask, lat_int, lon_int, alt, vx, vy, vz, afx, afy, afz, yaw, yaw_rate);
	return rc_mav_send_msg(msg);
}

int rc_mav_get_set_position_target_global_int(mavlink_set_position_target_global_int_t* data)
{
	if(__get_msg_common_checks(MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT)) return -1;
	mavlink_msg_set_position_target_global_int_decode(&messages[MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT], data);
	return 0;
}


int rc_mav_send_gps_raw_int(
	int32_t lat,
	int32_t lon,
	int32_t alt,
	uint16_t eph,
	uint16_t epv,
	uint16_t vel,
	uint16_t cog,
	uint8_t fix_type,
	uint8_t satellites_visible,
	int32_t alt_ellipsoid,
	uint32_t h_acc,
	uint32_t v_acc,
	uint32_t vel_acc,
	uint32_t hdg_acc)
{
	mavlink_message_t msg;
	mavlink_msg_gps_raw_int_pack(system_id, MAV_COMP_ID_ALL, &msg, __us_since_boot(), fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible, alt_ellipsoid, h_acc, v_acc, vel_acc, hdg_acc);
	return rc_mav_send_msg(msg);
}

int rc_mav_get_gps_raw_int(mavlink_gps_raw_int_t* data)
{
	if(__get_msg_common_checks(MAVLINK_MSG_ID_GPS_RAW_INT)) return -1;
	mavlink_msg_gps_raw_int_decode(&messages[MAVLINK_MSG_ID_GPS_RAW_INT], data);
	return 0;

}



int rc_mav_send_scaled_pressure(
	float press_abs,
	float press_diff,
	int16_t temperature)
{
	mavlink_message_t msg;
	mavlink_msg_scaled_pressure_pack(system_id, MAV_COMP_ID_ALL, &msg, __us_since_boot()/1000, press_abs, press_diff, temperature);
	return rc_mav_send_msg(msg);
}


int rc_mav_get_scaled_pressure(mavlink_scaled_pressure_t* data)
{
	if(__get_msg_common_checks(MAVLINK_MSG_ID_SCALED_PRESSURE)) return -1;
	mavlink_msg_scaled_pressure_decode(&messages[MAVLINK_MSG_ID_SCALED_PRESSURE], data);
	return 0;
}


int rc_mav_send_servo_output_raw(
	uint16_t servo1_raw,
	uint16_t servo2_raw,
	uint16_t servo3_raw,
	uint16_t servo4_raw,
	uint16_t servo5_raw,
	uint16_t servo6_raw,
	uint16_t servo7_raw,
	uint16_t servo8_raw,
	uint8_t port,
	uint16_t servo9_raw,
	uint16_t servo10_raw,
	uint16_t servo11_raw,
	uint16_t servo12_raw,
	uint16_t servo13_raw,
	uint16_t servo14_raw,
	uint16_t servo15_raw,
	uint16_t servo16_raw)
{
	mavlink_message_t msg;
	mavlink_msg_servo_output_raw_pack(system_id, MAV_COMP_ID_ALL, &msg, __us_since_boot(), port, servo1_raw, servo2_raw, servo3_raw, servo4_raw, servo5_raw, servo6_raw, servo7_raw, servo8_raw, servo9_raw, servo10_raw, servo11_raw, servo12_raw, servo13_raw, servo14_raw, servo15_raw, servo16_raw);
	return rc_mav_send_msg(msg);
}


int rc_mav_get_servo_output_raw(mavlink_servo_output_raw_t* data)
{
	if(__get_msg_common_checks(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW)) return -1;
	mavlink_msg_servo_output_raw_decode(&messages[MAVLINK_MSG_ID_SERVO_OUTPUT_RAW], data);
	return 0;
}


int rc_mav_send_sys_status(
	uint32_t onboard_control_sensors_present,
	uint32_t onboard_control_sensors_enabled,
	uint32_t onboard_control_sensors_health,
	uint16_t load,
	uint16_t voltage_battery,
	int16_t current_battery,
	uint16_t drop_rate_comm,
	uint16_t errors_comm,
	uint16_t errors_count1,
	uint16_t errors_count2,
	uint16_t errors_count3,
	uint16_t errors_count4,
	int8_t battery_remaining)
{
	mavlink_message_t msg;
	mavlink_msg_sys_status_pack(system_id, MAV_COMP_ID_ALL, &msg, onboard_control_sensors_present, onboard_control_sensors_enabled, onboard_control_sensors_health, load, voltage_battery, current_battery, battery_remaining, drop_rate_comm, errors_comm, errors_count1, errors_count2, errors_count3, errors_count4);
	return rc_mav_send_msg(msg);
}


int rc_mav_get_sys_status(mavlink_sys_status_t* data)
{
	if(__get_msg_common_checks(MAVLINK_MSG_ID_SYS_STATUS)) return -1;
	mavlink_msg_sys_status_decode(&messages[MAVLINK_MSG_ID_SYS_STATUS], data);
	return 0;
}


int rc_mav_send_manual_control(
	int16_t x,
	int16_t y,
	int16_t z,
	int16_t r,
	uint16_t buttons,
	uint8_t target)
{
	mavlink_message_t msg;
	mavlink_msg_manual_control_pack(system_id, MAV_COMP_ID_ALL, &msg,
					target, x, y, z, r, buttons);

	return rc_mav_send_msg(msg);
}

int rc_mav_get_manual_control(mavlink_manual_control_t* data)
{
	if(__get_msg_common_checks(MAVLINK_MSG_ID_MANUAL_CONTROL)) return -1;
	mavlink_msg_manual_control_decode(&messages[MAVLINK_MSG_ID_MANUAL_CONTROL], data);
	return 0;
}


int rc_mav_send_att_pos_mocap(float q[4], float x, float y, float z)
{
	mavlink_message_t msg;
	mavlink_msg_att_pos_mocap_pack(system_id, MAV_COMP_ID_ALL, &msg, __us_since_boot(), q, x, y, z);
	return rc_mav_send_msg(msg);
}

int rc_mav_get_att_pos_mocap(mavlink_att_pos_mocap_t* data)
{
	if(__get_msg_common_checks(MAVLINK_MSG_ID_ATT_POS_MOCAP)) return -1;
	mavlink_msg_att_pos_mocap_decode(&messages[MAVLINK_MSG_ID_ATT_POS_MOCAP], data);
	return 0;
}
