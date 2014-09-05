complementary_filter.c
James Strawson 2014

This is an example for how to measure the pitch angle
of the robotics cape using two different methods.

The first is using a first order complementary filter which
low-pass filters an angle estimate based on the inverse-tangent 
of the X and Z accelerometers and high-pass filters the integral
of the Y-gyroscope derived angular velocity. 

The second is by reading the angle off of the MPU-9150 IMU's
Digital Motion Processor or DMP. This performs a very similar
filter but with all 3 accels and gyros, resulting in greater
accuracy when the sensor rolls and yaws as well.