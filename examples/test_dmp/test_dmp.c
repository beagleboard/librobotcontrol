// Sample Code for testing MPU-9250 IMU operation with DMP and interrupts
// James Strawson - 2016


#include <useful_includes.h>
#include <robotics_cape.h>

int show_accel=0;
int show_gyro=0;
int enable_mag=0;
int read_temp=0;
int show_raw=0;
int show_quat=0;
int sample_rate=100;
imu_data_t data; //struct to hold new data

// printed if some invalid argument was given
void print_usage(){
	printf("\n Options\n");
	printf("-s {rate}	Set sample rate in HZ (default 200)\n");
	printf("-m		Enable Magnetometer\n");
	printf("-a		Enable printing of Accelerometer data\n");
	printf("-g		Enable printing of Gyro Data\n");
	printf("-r		Print raw ADC measurements instead of read units\n");
	printf("-q		Print quaternion instead of Euler angles\n");
	printf("-t		Read thermometer data too\n");
	printf("-h		Print this help message\n");
	printf("Sample rate must be a divisor of 200\n\n");
	return;
}


int print_data(){
	set_led(GREEN,1);
	printf("\r");
	
	if(show_quat){
		// print quaternion
		printf("Quat:%6.2f %6.2f %6.2f %6.2f ",	data.dmp_quat[QUAT_W], \
												data.dmp_quat[QUAT_X], \
												data.dmp_quat[QUAT_Y], \
												data.dmp_quat[QUAT_Z]);
	} else{
		// print Euler angles
		printf("Euler:%6.2f %6.2f %6.2f ",		data.dmp_euler[0], \
												data.dmp_euler[1], \
												data.dmp_euler[2]);
	}
	
	if(show_accel){
		if(show_raw) printf("A:%6d %6d %6d  ",	data.raw_accel[0],\
												data.raw_accel[1],\
												data.raw_accel[2]);
		else printf("A:%6.2f %6.2f %6.2f  ",	data.accel[0],\
												data.accel[1],\
												data.accel[2]);
	}
	
	if(show_gyro){
		if(show_raw) printf("A:%6d %6d %6d  ",	data.raw_gyro[0],\
												data.raw_gyro[1],\
												data.raw_gyro[2]);
		else printf("G:%6.1f %6.1f %6.1f  ",	data.gyro[0],\
												data.gyro[1],\
												data.gyro[2]);
	}
	
							 
	// if(read_mag_data(&data)){
		// printf("read mag data failed\n");
	// }
	// printf("M:%5.1f %5.1f %5.1f  ",data.mag[0],\
							 // data.mag[1],data.mag[2]);
	
	if(read_temp){
		if(read_imu_temp(&data)<0){
			printf("read temp data failed\n");
		}
		printf("T:%4.1f ", data.temp);
	}
													
	fflush(stdout);
	set_led(GREEN,0);
	return 0;
}
    
int main(int argc, char *argv[]){
	int c;
	
	// parse arguments
	opterr = 0;
	while ((c=getopt(argc, argv, "magrqths:"))!=-1 && argc>1){
		switch (c){
		case 's': // sample rate option
			sample_rate = atoi(optarg);
			if(sample_rate>200 || sample_rate<4){
				printf("sample_rate but be between 4 & 200");
				return -1;
			}
			break;
		case 'm': // magnetometer option
			enable_mag = 1;
			break;
		case 'a': // show accelerometer option
			show_accel = 1;
			break;
		case 'g': // show gyro option
			show_gyro = 1;
			break;
		case 'r': // show raw option
			show_raw = 1;
			break;
		case 'q': // show quaternion option
			show_quat = 1;
			break;
		case 't': // read thermometer option
			read_temp = 1;
			break;
		case 'h': // show help option
			print_usage();
			return -1;
			break;
		default:
			printf("opt: %c\n",c);
			printf("invalid argument\n");
			print_usage();
			return -1;
			break;
		}
    }
	
	// start by initializing cape as always
	if(initialize_cape()){
		printf("ERROR: failed to initialize_cape\n");
		return -1;
	}
	
	// start with defaults and modify magnetometer and sample rate
	imu_config_t conf = get_default_imu_config();
	conf.enable_magnetometer = enable_mag;
	conf.dmp_sample_rate = sample_rate;
	if(initialize_imu_dmp(&data, conf)){
		printf("initialize_imu_failed\n");
		return -1;
	}
	
	// set the interrupt function
	set_imu_interrupt_func(&print_data);
	
	//now just wait, print_data will be called by the interrupt
	while (get_state()!=EXITING) {
		usleep(1000);
	}
	
	// shut things down
	power_off_imu();
	cleanup_cape();
	return 0;
}

