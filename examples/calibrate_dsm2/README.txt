calibrate_dsm2
James Strawson - 2014

Project Description:
Running the calibrate_dsm2 example will print out raw data to the console and record the min and max values for each channel. These limits will be saved to "/root/robot_config/dsm2.cal".

Make sure the transmitter and receiver are paired before testing. Use the pair_dsm2 example if you haven't already used a bind plug and standard receiver. The satellite receiver remembers which transmitter it is paired to, not your BeagleBone. Spektrum and Orange brand DSM2 satellite receivers communicate at 115200 baud over a single serial datalink. Your Robotics Cape reads this data through the UART4 port and can be tested here. 