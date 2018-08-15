/**
 * <rc/mavlink_udp_helpers.h>
 *
 * @brief      helper functions for the most common mavlink packets.
 *
 * For use with the functions in mavlink_udp.h
 *
 * @author     James Strawson & Henry Gaudet
 * @date       1/24/2018
 *
 * @addtogroup Mavlink_Helpers
 * @ingroup    Mavlink
 * @{
 */

#ifndef RC_MAVLINK_UDP_HELPERS_H
#define RC_MAVLINK_UDP_HELPERS_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>	// for specific integer types

// these are directly from the mavlink source
#include <rc/mavlink/common/mavlink.h>
#include <rc/mavlink/mavlink_types.h>



/**
 * @brief      Constructs and sends a heartbeat packet of type
 * MAVLINK_MSG_ID_HEARTBEAT
 *
 * This is a shortcut for rc_mav_send_heartbeat for those which don't wish to
 * populate all of the availbale fields in the mavlink heartbeat packet. It
 * still sends a complete heartbeat packet but with all 0's in the packet
 * payload fields. The receiver can still see the system_id of the sender and
 * this still serves the primary purpose to the heartbeat packet which is to
 * indicate that a communication channel is open and functioning.
 *
 * @return     0 on success or -1 on failure.
 */
int rc_mav_send_heartbeat_abbreviated(void);

/**
 * @brief      Constructs and sends a heartbeat packet of type
 * MAVLINK_MSG_ID_HEARTBEAT
 *
 * Constructs and sends a heartbeat packet to the previously set destination ip
 * address. The arguments encompass all available parameters in the heartbeat
 * packet. However, many users will not be bothered to populate these parameters
 * and may opt to use rc_mav_send_heartbeat_abbreviated() instead.
 *
 * @param[in]  custom_mode    A bitfield for use for autopilot-specific flags.
 * @param[in]  type           Type of the MAV (quadrotor, helicopter, etc., up
 * to 15 types, defined in MAV_TYPE ENUM)
 * @param[in]  autopilot      Autopilot type / class. defined in MAV_AUTOPILOT
 * ENUM
 * @param[in]  base_mode      System mode bitfield, see MAV_MODE_FLAGS ENUM in
 * mavlink/include/mavlink_types.h
 * @param[in]  system_status  System status flag, see MAV_STATE ENUM
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_send_heartbeat(
	uint32_t custom_mode,
	uint8_t type,
	uint8_t autopilot,
	uint8_t base_mode,
	uint8_t system_status);


/**
 * @brief      fetches the most recently received heartbeat packet of type
 * MAVLINK_MSG_ID_HEARTBEAT
 *
 * @param[out] data  pointer to put heartbeat packet data
 *
 * @return     0 on success, -1 on failure for example if no message has been
 * received of type msg_id.
 */
int rc_mav_get_heartbeat(mavlink_heartbeat_t* data);


/**
 * @brief      Send and attitude packet of type MAVLINK_MSG_ID_ATTITUDE
 *
 * @param[in]  roll        Roll angle (rad, -pi..+pi)
 * @param[in]  pitch       Pitch angle (rad, -pi..+pi)
 * @param[in]  yaw         Yaw angle (rad, -pi..+pi)
 * @param[in]  rollspeed   Roll angular speed (rad/s)
 * @param[in]  pitchspeed  Pitch angular speed (rad/s)
 * @param[in]  yawspeed    Yaw angular speed (rad/s)
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_send_attitude(
	float roll,
	float pitch,
	float yaw,
	float rollspeed,
	float pitchspeed,
	float yawspeed);

/**
 * @brief      Fetches the last attitude packet of type MAVLINK_MSG_ID_ATTITUDE
 *
 * @param[out] data  Pointer to user's packet struct to be populated with new
 * data
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_get_attitude(mavlink_attitude_t* data);


/**
 * @brief      Packs and sends an attitude quaternion packet of type
 * MAVLINK_MSG_ID_ATTITUDE_QUATERNION
 *
 * @param[in]  q1          Quaternion component 1
 * @param[in]  q2          Quaternion component 2
 * @param[in]  q3          Quaternion component 3
 * @param[in]  q4          Quaternion component 4
 * @param[in]  rollspeed   Roll angular speed (rad/s)
 * @param[in]  pitchspeed  Pitch angular speed (rad/s)
 * @param[in]  yawspeed    Yaw angular speed (rad/s)
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_send_attitude_quaternion(
	float q1,
	float q2,
	float q3,
	float q4,
	float rollspeed,
	float pitchspeed,
	float yawspeed);


/**
 * @brief      Fetches and unpacks the last received
 * MAVLINK_MSG_ID_ATTITUDE_QUATERNION
 *
 * @param[out] data  Pointer to user's packet struct to be populated with new
 * data
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_get_attitude_quaternion(mavlink_attitude_quaternion_t* data);


/**
 * @brief      Packs and sends a local position NED packet of type
 * MAVLINK_MSG_ID_LOCAL_POSITION_NED
 *
 * @param[in]  x     X Position (meters)
 * @param[in]  y     Y Position (meters)
 * @param[in]  z     Z Position (meters)
 * @param[in]  vx    X Speed (m/s)
 * @param[in]  vy    Y Speed (m/s)
 * @param[in]  vz    Z Speed (m/s)
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_send_local_position_ned(
	float x,
	float y,
	float z,
	float vx,
	float vy,
	float vz);


/**
 * @brief      Fetches and unpacks the last received
 * MAVLINK_MSG_ID_LOCAL_POSITION_NED
 *
 * @param[out] data  Pointer to user's packet struct to be populated with new
 * data
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_get_local_position_ned(mavlink_local_position_ned_t* data);


/**
 * @brief      Packs and sends a packet of type
 * MAVLINK_MSG_ID_GLOBAL_POSITION_INT
 *
 * @param[in]  lat           Latitude, expressed as * 1E7
 * @param[in]  lon           Longitude, expressed as * 1E7
 * @param[in]  alt           Altitude in meters, expressed as * 1000
 * (millimeters), above MSL
 * @param[in]  relative_alt  Altitude above ground in meters, expressed as *
 * 1000 (millimeters)
 * @param[in]  vx            Ground X Speed (Latitude), expressed as m/s * 100
 * @param[in]  vy            Ground Y Speed (Longitude), expressed as m/s * 100
 * @param[in]  vz            Ground Z Speed (Altitude), expressed as m/s * 100
 * @param[in]  hdg           Compass heading in degrees * 100, 0.0..359.99
 * degrees. If unknown, set to: UINT16_MAX
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_send_global_position_int(
	int32_t lat,
	int32_t lon,
	int32_t alt,
	int32_t relative_alt,
	int16_t vx,
	int16_t vy,
	int16_t vz,
	uint16_t hdg);


/**
 * @brief      Fetches and unpacks the last received
 * MAVLINK_MSG_ID_GLOBAL_POSITION_INT
 *
 * @param[out] data  Pointer to user's packet struct to be populated with new
 * data
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_get_global_position_int(mavlink_global_position_int_t* data);


/**
 * @brief      Packs and sends a packet of type
 * MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED
 *
 * @param[in]  x                 X Position in NED frame in m
 * @param[in]  y                 Y Position in NED frame in m
 * @param[in]  z                 Z Position in NED frame in meters (note,
 * altitude is negative in NED)
 * @param[in]  vx                X velocity in NED frame in m/s
 * @param[in]  vy                Y velocity in NED frame in m/s
 * @param[in]  vz                Z velocity in NED frame in m/s
 * @param[in]  afx               X acceleration or force (if bit 10 of type_mask
 * is set) in NED frame in meter/s^2 or N
 * @param[in]  afy               Y acceleration or force (if bit 10 of type_mask
 * is set) in NED frame in meter/s^2 or N
 * @param[in]  afz               Z acceleration or force (if bit 10 of type_mask
 * is set) in NED frame in meter/s^2 or N
 * @param[in]  yaw               yaw setpoint in rad
 * @param[in]  yaw_rate          yaw rate setpoint in rad/s
 * @param[in]  type_mask         Bitmask to indicate which dimensions should be
 * ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000
 * indicates that none of the setpoint dimensions should be ignored. If bit 10
 * is set the floats afx afy afz should be interpreted as force instead of
 * acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy,
 * bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit
 * 11: yaw, bit 12: yaw rate
 * @param[in]  target_system     System ID of the target system
 * @param[in]  target_component  Component ID
 * @param[in]  coordinate_frame  Valid options are: MAV_FRAME_LOCAL_NED = 1,
 * MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8,
 * MAV_FRAME_BODY_OFFSET_NED = 9
 *
 * @return     0 on success, -1 on failure
 */
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
	uint8_t coordinate_frame);


/**
 * @brief      fetches and unpacks the most recently received packet of type
 * MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED
 *
 * @param[out] data  Pointer to user's packet struct to be populated with new
 * data
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_get_set_position_target_local_ned(mavlink_set_position_target_local_ned_t* data);


/**
 * @brief      Packs and sends a packet of type
 * MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT
 *
 * @param[in]  lat_int           X Position in WGS84 frame in 1e7 * degrees
 * @param[in]  lon_int           Y Position in WGS84	frame in 1e7 * degrees
 * @param[in]  alt               Altitude in meters in AMSL altitude, not WGS84
 * if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
 * @param[in]  vx                X velocity in NED frame in m/s
 * @param[in]  vy                Y velocity in NED frame in m/s
 * @param[in]  vz                Z velocity in NED frame in m/s
 * @param[in]  afx               X acceleration or force (if bit 10 of type_mask
 * is set) in NED frame in meter/s^2 or N
 * @param[in]  afy               Y acceleration or force (if bit 10 of type_mask
 * is set) in NED frame in meter/s^2 or N
 * @param[in]  afz               Z acceleration or force (if bit 10 of type_mask
 * is set) in NED frame in meter/s^2 or N
 * @param[in]  yaw               yaw setpoint in rad
 * @param[in]  yaw_rate          yaw rate setpoint in rad/s
 * @param[in]  type_mask         Bitmask to indicate which dimensions should be
 * ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000
 * indicates that none of the setpoint dimensions should be ignored. If bit 10
 * is set the	floats afx afy afz should be interpreted as	force instead of
 * acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy,
 * bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit
 * 11: yaw, bit 12: yaw rate
 * @param[in]  target_system     System ID
 * @param[in]  target_component  Component ID
 * @param[in]  coordinate_frame  Valid options are: MAV_FRAME_GLOBAL_INT = 5,
 * MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
 *
 * @return     0 on success, -1 on failure
 */
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
	uint8_t coordinate_frame);


/**
 * @brief      Fetches and unpacks the last received packet of type
 * MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT
 *
 * @param[out] data  Pointer to user's packet struct to be populated with new
 * data
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_get_set_position_target_global_int(mavlink_set_position_target_global_int_t* data);


/**
 * @brief      Packs and sends a packet of type MAVLINK_MSG_ID_GPS_RAW_INT
 *
 * @param[in]  lat                 Latitude (WGS84, EGM96 ellipsoid), in degrees *
 * 1E7
 * @param[in]  lon                 Longitude (WGS84, EGM96 ellipsoid), in
 * degrees * 1E7
 * @param[in]  alt                 Altitude (AMSL, NOT WGS84), in meters * 1000
 * (positive for up). Note that virtually all GPS modules provide the AMSL
 * altitude in addition to the WGS84 altitude.
 * @param[in]  eph                 GPS HDOP horizontal dilution of position
 * (unitless). If unknown, set to: UINT16_MAX
 * @param[in]  epv                 GPS VDOP vertical dilution of position
 * (unitless). If unknown, set to: UINT16_MAX
 * @param[in]  vel                 GPS ground speed (m/s * 100). If unknown, set
 * to: UINT16_MAX
 * @param[in]  cog                 Course over ground (NOT heading, but
 * direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set
 * to: UINT16_MAX
 * @param[in]  fix_type            See the GPS_FIX_TYPE enum.
 * @param[in]  satellites_visible  Number of satellites visible. If unknown, set
 * to 255
 * @param[in]  alt_ellipsoid       Altitude (above WGS84, EGM96 ellipsoid), in
 * meters * 1000 (positive for up).
 * @param[in]  h_acc               Position uncertainty in meters * 1000
 * (positive for up).
 * @param[in]  v_acc               Altitude uncertainty in meters * 1000
 * (positive for up).
 * @param[in]  vel_acc             Speed uncertainty in meters * 1000 (positive
 * for up).
 * @param[in]  hdg_acc             Heading / track uncertainty in degrees * 1e5.
 *
 * @return     0 on success, -1 on failure
 */
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
	uint32_t hdg_acc);


/**
 * @brief      Fetches and unpacks a packet of type MAVLINK_MSG_ID_GPS_RAW_INT
 *
 * @param[out] data  Pointer to user's packet struct to be populated with new
 * data
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_get_gps_raw_int(mavlink_gps_raw_int_t* data);


/**
 * @brief      Packs and sends a packet of type MAVLINK_MSG_ID_SCALED_PRESSURE
 *
 * @param[in]  press_abs    Absolute pressure (hectopascal)
 * @param[in]  press_diff   Differential pressure 1 (hectopascal)
 * @param[in]  temperature  Temperature measurement (0.01 degrees celsius)
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_send_scaled_pressure(
	float press_abs,
	float press_diff,
	int16_t temperature);

/**
 * @brief      Fetches and unpacks last received packet of type
 * MAVLINK_MSG_ID_SCALED_PRESSURE
 *
 * @param[out] data  Pointer to user's packet struct to be populated with new
 * data
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_get_scaled_pressure(mavlink_scaled_pressure_t* data);


/**
 * @brief      Packs and sends packet of type MAVLINK_MSG_ID_SERVO_OUTPUT_RAW
 *
 * @param[in]  servo1_raw   Servo output 1 value, in microseconds
 * @param[in]  servo2_raw   Servo output 2 value, in microseconds
 * @param[in]  servo3_raw   Servo output 3 value, in microseconds
 * @param[in]  servo4_raw   Servo output 4 value, in microseconds
 * @param[in]  servo5_raw   Servo output 5 value, in microseconds
 * @param[in]  servo6_raw   Servo output 6 value, in microseconds
 * @param[in]  servo7_raw   Servo output 7 value, in microseconds
 * @param[in]  servo8_raw   Servo output 8 value, in microseconds
 * @param[in]  port         Servo output port (set of 8 outputs = 1 port). Most
 * MAVs will just use one, but this allows to encode more than 8 servos.
 * @param[in]  servo9_raw   Servo output 9 value, in microseconds
 * @param[in]  servo10_raw  Servo output 10 value, in microseconds
 * @param[in]  servo11_raw  Servo output 11 value, in microseconds
 * @param[in]  servo12_raw  Servo output 12 value, in microseconds
 * @param[in]  servo13_raw  Servo output 13 value, in microseconds
 * @param[in]  servo14_raw  Servo output 14 value, in microseconds
 * @param[in]  servo15_raw  Servo output 15 value, in microseconds
 * @param[in]  servo16_raw  Servo output 16 value, in microseconds
 *
 * @return     0 on success, -1 on failure
 */
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
	uint16_t servo16_raw);


/**
 * @brief      Fetch and unpack message of type MAVLINK_MSG_ID_SERVO_OUTPUT_RAW
 *
 * @param[out] data  Pointer to user's packet struct to be populated with new
 * data
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_get_servo_output_raw(mavlink_servo_output_raw_t* data);


/**
 * @brief      { function_description }
 *
 * @param[in]  onboard_control_sensors_present  Bitmask showing which onboard
 * controllers and sensors are present. Value of 0: not present. Value of 1:
 * present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
 * @param[in]  onboard_control_sensors_enabled  Bitmask showing which onboard
 * controllers and sensors are enabled:  Value of 0: not enabled. Value of 1:
 * enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
 * @param[in]  onboard_control_sensors_health   Bitmask showing which onboard
 * controllers and sensors are operational or have an error: Value of 0: not
 * enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
 * @param[in]  load                             Maximum usage in percent of the
 * mainloop time, (0%: 0, 100%: 1000) should be always below 1000
 * @param[in]  voltage_battery                  Battery voltage, in millivolts
 * (1 = 1 millivolt)
 * @param[in]  current_battery                  Battery current, in
 * 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the
 * current
 * @param[in]  drop_rate_comm                   Communication drops in percent,
 * (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links
 * (packets that were corrupted on reception on the MAV)
 * @param[in]  errors_comm                      Communication errors (UART, I2C,
 * SPI, CAN), dropped packets on all links (packets that were corrupted on
 * reception on the MAV)
 * @param[in]  errors_count1                    Autopilot-specific errors
 * @param[in]  errors_count2                    Autopilot-specific errors
 * @param[in]  errors_count3                    Autopilot-specific errors
 * @param[in]  errors_count4                    Autopilot-specific errors
 * @param[in]  battery_remaining                Remaining battery energy: (0%:
 * 0, 100%: 100), -1: autopilot estimate the remaining battery
 *
 * @return     0 on success, -1 on failure
 */
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
	int8_t battery_remaining);


/**
 * @brief      Fetch and unpack packet of type MAVLINK_MSG_ID_SERVO_OUTPUT_RAW
 *
 * @param[out] data  Pointer to user's packet struct to be populated with new
 * data
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_get_sys_status(mavlink_sys_status_t* data);


/**
 * @brief      Pack and send message of type MAVLINK_MSG_ID_MANUAL_CONTROL
 *
 * @param[in]  x        X-axis, normalized to the range [-1000,1000]. A value of
 * INT16_MAX indicates that this axis is invalid. Generally corresponds to
 * forward(1000)-backward(-1000) movement on a joystick and the pitch of a
 * vehicle.
 * @param[in]  y        Y-axis, normalized to the range [-1000,1000]. A value of
 * INT16_MAX indicates that this axis is invalid. Generally corresponds to
 * left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
 * @param[in]  z        Z-axis, normalized to the range [-1000,1000]. A value of
 * INT16_MAX indicates that this axis is invalid. Generally corresponds to a
 * separate slider movement with maximum being 1000 and minimum being -1000 on a
 * joystick and the thrust of a vehicle.
 * @param[in]  r        R-axis, normalized to the range [-1000,1000]. A value of
 * INT16_MAX indicates that this axis is invalid. Generally corresponds to a
 * twisting of the joystick, with counter-clockwise being 1000 and clockwise
 * being -1000, and the yaw of a vehicle.
 * @param[in]  buttons  A bitfield corresponding to the joystick buttons'
 * current state, 1 for pressed, 0 for released. The lowest bit corresponds to
 * Button 1.
 * @param[in]  target   The system to be controlled.
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_send_manual_control(
	int16_t x,
	int16_t y,
	int16_t z,
	int16_t r,
	uint16_t buttons,
	uint8_t target);


/**
 * @brief      Fetch and unpack the last received message of type
 * MAVLINK_MSG_ID_MANUAL_CONTROL
 *
 * @param[out] data  Pointer to user's packet struct to be populated with new
 * data
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_get_manual_control(mavlink_manual_control_t* data);


/**
 * @brief      Packs and send a message of type MAVLINK_MSG_ID_ATT_POS_MOCAP
 *
 * @param      q     Attitude quaternion, w, x, y, z order, zero-rotation is
 * (1,0,0,0)
 * @param[in]  x     X position in meters (NED)
 * @param[in]  y     Y position in meters (NED)
 * @param[in]  z     Z position in meters (NED)
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_send_att_pos_mocap(
	float q[4],
	float x,
	float y,
	float z);


/**
 * @brief      Fetche and unpacks last received packet of type
 * MAVLINK_MSG_ID_ATT_POS_MOCAP
 *
 * @param[out] data  Pointer to user's packet struct to be populated with new
 * data
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_get_att_pos_mocap(mavlink_att_pos_mocap_t* data);


#ifdef __cplusplus
}
#endif

#endif // RC_MAVLINK_UDP_HELPERS_H

///@} end group Mavlink_Helpers
