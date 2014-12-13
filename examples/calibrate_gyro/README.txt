calibrate_gyro
James Strawson - 2014

Project Description:
The IMU's gyroscopes have steady state error from the factory. To zero this out, call the calibrate_gyro function while the beaglebone sits very still on a hard surface. This saves steady state offsets to "/root/robot_config/gyro.cal"

Any time you call initialize_imu() in your robot project, these offsets are loaded into the IMU so you don't have to calibrate your gyro each time your program starts.