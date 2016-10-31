/*******************************************************************************
* test_imu.c
*
* James Strawson 2016
* This serves as an example of how to read the IMU with direct reads to the
* sensor registers. To use the DMP or interrupt-driven timing see test_dmp.c
*******************************************************************************/

#include "../../libraries/roboticscape-usefulincludes.h"
#include "../../libraries/roboticscape.h"

int main(){
	imu_data_t data; //struct to hold new data
	
	if(initialize_cape()){
		printf("ERROR: failed to initialize_cape\n");
		return -1;
	}
	
	// use defaults for now, except also enable magnetometer.
	imu_config_t conf = get_default_imu_config();
	conf.enable_magnetometer=1;
	
	if(initialize_imu(&data, conf)){
		printf("initialize_imu_failed\n");
		return -1;
	}
	
	// print a header
	printf("\n");
	printf("   Accel XYZ(m/s^2)  |");
	printf("   Gyro XYZ (deg/s)  |");
	printf("  Mag Field XYZ(uT)  |");
	printf(" Temp (C)");
	printf("\n");
	
	//now just wait, print_data will run
	while (get_state() != EXITING) {
		printf("\r");
		
		if(read_accel_data(&data)<0)
			printf("read accel data failed\n");
		else printf("%6.2f %6.2f %6.2f |",	data.accel[0],\
											data.accel[1],\
											data.accel[2]);
								
		if(read_gyro_data(&data)<0)
			printf("read gyro data failed\n");
		else printf("%6.1f %6.1f %6.1f |",	data.gyro[0],\
											data.gyro[1],\
											data.gyro[2]);
								 
		if(read_mag_data(&data)<0){
			printf("read mag data failed\n");
		}
		else printf("%6.1f %6.1f %6.1f |",	data.mag[0],\
											data.mag[1],\
											data.mag[2]);
								
		if(read_imu_temp(&data)<0){
			printf("read temp data failed\n");
		}
		else printf(" %4.1f ", data.temp);
														
		fflush(stdout);
		usleep(100000);
	}

	power_off_imu();
	cleanup_cape();
	return 0;
}

