// Sample Code for testing MPU-9250 IMU operation
// James Strawson - 2013

#include <robotics_cape.h>
#include <useful_includes.h>

int enable_accel=0;
int enable_gyro=0;
int enable_mag=0;


    
int main(int argc, char *argv[]){
	imu_data_t data; //struct to hold new data
	
	if(initialize_cape()){
		printf("ERROR: failed to initialize_cape\n");
		return -1;
	}
	
	// use defaults for now
	imu_config_t conf = get_default_imu_config();
	conf.enable_magnetometer=0;
	
	if(initialize_imu(&data, conf)){
		printf("initialize_imu_failed\n");
		return -1;
	}
	
	//now just wait, print_data will run
	while (get_state() != EXITING) {
		printf("\r");
		
		if(read_accel_data(&data))
			printf("read accel data failed\n");
		else printf("A:%6.2f %6.2f %6.2f  ",data.accel[0],\
								data.accel[1],data.accel[2]);
								
		if(read_gyro_data(&data))
			printf("read gyro data failed\n");
		else printf("G:%6.1f %6.1f %6.1f  ",data.gyro[0],\
								 data.gyro[1],data.gyro[2]);
								 
		// if(read_mag_data(&data)){
			// printf("read mag data failed\n");
		// }
		// printf("M:%5.1f %5.1f %5.1f  ",data.mag[0],\
								 // data.mag[1],data.mag[2]);
								
		if(read_imu_temp(&data)){
			printf("read temp data failed\n");
		}
		printf("T:%4.1f ", data.temp);
														
		fflush(stdout);
		usleep(100000);
	}
	cleanup_cape();
	return 0;
}

