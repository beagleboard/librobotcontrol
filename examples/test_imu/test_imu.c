/*******************************************************************************
* test_imu.c
*
* James Strawson 2016
* This serves as an example of how to read the IMU with direct reads to the
* sensor registers. To use the DMP or interrupt-driven timing see test_dmp.c
*******************************************************************************/

#include "../../libraries/roboticscape-usefulincludes.h"
#include "../../libraries/roboticscape.h"

// possible modes, user selected with command line arguments
typedef enum m_mode_t{
	RAD,
	DEG,
	RAW
} m_mode_t;

// printed if some invalid argument was given
void print_usage(){
	printf("\n");
	printf("-r		print raw values instead of radians\n");
	printf("-d		print gyro in degrees/s instead of radians\n");
	printf("-h		print this help message\n");
	printf("\n");
}

int main(int argc, char *argv[]){
	imu_data_t data; //struct to hold new data
	int c;
	m_mode_t mode = RAD; // default to radian mode.


	// parse arguments
	opterr = 0;
	while ((c = getopt(argc, argv, "m:d:fbs:h")) != -1){
		switch (c){
		case 'r':
			if(mode!=RAD) print_usage();
			mode = RAW;
			printf("\nRaw values are from 16-bit ADC\n");
			break;
		case 'd':
			if(mode!=RAD) print_usage();
			mode = DEG;
			break;
		case 'h':
			print_usage();
			return 0;
			break;
		default:
			print_usage();
			return -1;
			break;
		}
	}

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
	switch(mode){
	case RAD:
		printf("   Accel XYZ(m/s^2)  |");
		printf("   Gyro XYZ (rad/s)  |");
		break;
	case DEG:
		printf("   Accel XYZ(m/s^2)  |");
		printf("   Gyro XYZ (deg/s)  |");
		break;
	case RAW:
		printf("   Accel XYZ(raw adc)  |");
		printf("   Gyro XYZ (raw adc)  |");
		break;
	default:
		printf("ERROR: invalid mode\n");
		return -1;
	}
	printf("  Mag Field XYZ(uT)  |");
	printf(" Temp (C)");
	printf("\n");
	
	//now just wait, print_data will run
	while (get_state() != EXITING) {
		printf("\r");
		
		// print accel
		if(read_accel_data(&data)<0){
			printf("read accel data failed\n");
		}
		if(mode==RAW){
			printf("%7d %7d %7d |",			data.raw_accel[0],\
											data.raw_accel[1],\
											data.raw_accel[2]);
		}
		else{
			printf("%6.2f %6.2f %6.2f |",	data.accel[0],\
											data.accel[1],\
											data.accel[2]);
		}
		
		// print gyro data
		if(read_gyro_data(&data)<0){
			printf("read gyro data failed\n");
		}
		switch(mode){
		case RAD:
			printf("%6.1f %6.1f %6.1f |",	data.gyro[0],\
											data.gyro[1],\
											data.gyro[2]);
			break;
		case DEG:
			printf("%6.1f %6.1f %6.1f |",	data.gyro[0]*RAD_TO_DEG,\
											data.gyro[1]*RAD_TO_DEG,\
											data.gyro[2]*RAD_TO_DEG);
			break;
		case RAW:
			printf("%7d %7d %7d |",			data.raw_gyro[0],\
											data.raw_gyro[1],\
											data.raw_gyro[2]);
			break;
		default:
			printf("ERROR: invalid mode\n");
			return -1;
		}

		// read magnetometer
		if(read_mag_data(&data)<0){
			printf("read mag data failed\n");
		}
		else printf("%6.1f %6.1f %6.1f |",	data.mag[0],\
											data.mag[1],\
											data.mag[2]);

		// read temperature
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

