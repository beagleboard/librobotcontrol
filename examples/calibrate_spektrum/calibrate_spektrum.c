/*
Spektrum DSM2 Radio testing and Calibration Function
Running the calibrate_spektrum executable will print out raw 
data to the terminal along with min and max recorded values. 
These limits will be saved to a calibration file in 
/home/root/calibration to be used with your projects.

James Strawson - 2013
*/


#include <robotics_cape.h>

int main(){
	initialize_cape();
	
	initialize_spektrum();
	calibrate_spektrum();
	return 0;
}