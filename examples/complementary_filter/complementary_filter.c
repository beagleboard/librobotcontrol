// Complementary Filter for pitch angle estimation about Y axis
// James Strawson 2014

#include <robotics_cape.h>

#define SAMPLE_RATE_HZ 200	// 200hz
#define DT 0.005       		// 5ms
#define THETA_MIX_TC  2   // time constant on filter

// Complementary Filter for pitch angle theta about Y axis
const float HP_CONST = THETA_MIX_TC/(THETA_MIX_TC + DT);
const float LP_CONST = DT/(THETA_MIX_TC + DT);
int xAccel, zAccel, yGyroOffset;
float yGyro, accLP, gyroHP, theta, theta_DMP;
unsigned short gyro_fsr; //full scale range of gyro
float gyro_to_rad_per_sec;
mpudata_t mpu; //struct to read IMU data into

int filter_loop(){
	if (mpu9150_read(&mpu) == 0) {
		//raw integer accelerometer values
		xAccel = mpu.rawAccel[VEC3_X]; // DMP reverses x and Z
		zAccel = mpu.rawAccel[VEC3_Z]; 
		yGyro = -(mpu.rawGyro[VEC3_Y]-yGyroOffset);
		theta_DMP = mpu.fusedEuler[VEC3_Y];
	}
	//first order filters
	accLP = accLP + LP_CONST * (atan2(zAccel,-xAccel) - accLP);
	gyroHP = HP_CONST*(gyroHP + (DT*yGyro*gyro_to_rad_per_sec));
	theta = gyroHP + accLP;
	//printf("%0.2f	%0.2f\n", theta, theta_DMP);
	return 0;
}


// Filter Initialization sampling
float sum_ax, sum_az, sum_gy;
int warmup_samples;

int sample_imu(){
	if (mpu9150_read(&mpu) == 0) {
		sum_ax += mpu.rawAccel[VEC3_X];
		sum_az += mpu.rawAccel[VEC3_Z]; 
		sum_gy += mpu.rawGyro[VEC3_Y];
		warmup_samples ++;
	}
	return 0; 
}


// 10hz Loop printing data
void* slow_loop_func(void* ptr){
	do{
		printf("\r");
		// printf("xAccel: %d ",		xAccel);
		// printf("zAccel: %d ",		zAccel);
		// printf("ygyro: %0.2f ",		yGyro);
		// printf("gyroFSR %d ",		gyro_fsr);
		printf("gyroHP: %0.2f ",	gyroHP);
		printf("accLP: %0.2f ", 	accLP);
		printf("theta: %0.2f ", 	theta);
		printf("DMP: %0.2f",		theta_DMP);
		printf("     ");
		fflush(stdout);
		usleep(100000); //print at roughly 10 hz, not very accurate
	}while(get_state() != EXITING);
	return NULL;
}

// If the user holds the start button for a second, exit cleanly
int on_start_press(){
	int i=0;
	do{
		usleep(100000);
		if(get_start_button() == LOW){
			return 0; //user let go before time-out
		}
		i++;
	}while(i<10);
	//user held the button down long enough, exit cleanly
	set_state(EXITING);
	return 0;
}

int main(){
	initialize_cape();
	set_start_pressed_func(&on_start_press);
	signed char orientation[9] = ORIENTATION_UPRIGHT; //could also use ORIENTATION_FLAT
	initialize_imu(SAMPLE_RATE_HZ, orientation);
	
	// read the gyro full-scale range
	mpu_get_gyro_fsr(&gyro_fsr);
	gyro_to_rad_per_sec = gyro_fsr*DEG_TO_RAD/32768;
	
	// now start initiating filter by sampling sensors for a second
	set_imu_interrupt_func(&sample_imu);
	sleep(1);
	set_imu_interrupt_func(&null_func); //stop interrupt routine
	yGyroOffset = sum_gy/warmup_samples;	// offset to correct for gyro bias
    accLP = atan2(sum_az, -sum_ax); 	// initialize accLP at current theta
	theta = accLP;								// start theta based on accel
	printf("yGyroOffset = %d\n", yGyroOffset);

	// start the filter
	set_imu_interrupt_func(&filter_loop); 
	
	// start slow thread printing data
	pthread_t slow_thread;
	pthread_create(&slow_thread, NULL, slow_loop_func, (void*) NULL);
	
	setGRN(1);
	while(get_state()!=EXITING){
		sleep(1);	// chill
	}
	cleanup_cape();
	return 0;
}