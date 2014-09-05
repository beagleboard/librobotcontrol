Spektrum DSM2 Radio testing and Calibration Function
James Strawson - 2013

Project Description:
	Spektrum DSM2 satellite recievers and 3rd party equivalents 
communicate at 115200 baud over a single serial datalink. Your 
Robotics Cape reads this data through the UART4 port and can be 
tested here. 
	Running the calibrate_spektrum executable will print out raw 
data to the terminal along with min and max recorded values. 
These limits will be saved to a calibration file in 
/home/root/calibration to be used with your projects.
Make sure the transmitter and reciever are 
paired before testing. Use the pair_spektrum example if you don't 
have a bind plug and reciever. The reciever remembers which 
transmitter it is paired to, not your BeagleBone.

Compile Instructions:
make clean - Removes compiled object and executable files.
make - Links and compiles your source file into an executable. Run with "./project_name"
make install - Adds a copy of your executable to /usr/bin. Use this to keep a copy of your stable programs that can be executed from any directory.