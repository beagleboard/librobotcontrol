// calibrate_gyro.c
// James Strawson - 2014

// This routine samples the gyro for a second and saves a .cal file containing the offsets

#include <robotics_cape.h>
#include <useful_includes.h>

int main(){
	if(initialize_cape()<0){
		printf("Failed to initialize cape, exiting\n");
		return -1;
	}
	
	printf("\nThis program will generate a new gyro calibration file\n");
	printf("keep your beaglebone very still and hit enter to calibrate\n");
	while( getchar() != '\n' );
	
	if(calibrate_gyro_routine()<0){
		printf("Failed to complete gyro calibration\n");
		return -1;
	}
	
	printf("\ngyro calibration file written\n");
	printf("run test_imu to check performance\n");
		
	cleanup_cape();
	return 0;
}
