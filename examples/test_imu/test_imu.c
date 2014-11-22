// Sample Code for testing MPU-9150 operation
// James Strawson - 2013

#include <robotics_cape.h>
#define DEFAULT_SAMPLE_RATE	200  // This is also the fastest speed the DMP will do

int print_imu_data(){
	mpudata_t mpu; //struct to read IMU data into
	memset(&mpu, 0, sizeof(mpudata_t)); //make sure it's clean before starting
	if (mpu9150_read(&mpu) == 0) {
		printf("\r");
		
		printf("X: %0.1f Y: %0.1f Z: %0.1f ",
		mpu.fusedEuler[VEC3_X] * RAD_TO_DEGREE, 
		mpu.fusedEuler[VEC3_Y] * RAD_TO_DEGREE, 
		mpu.fusedEuler[VEC3_Z] * RAD_TO_DEGREE);
		
		printf("Xg: %05d Yg: %05d Zg: %05d ",
		mpu.rawGyro[VEC3_X], 
		mpu.rawGyro[VEC3_Y], 
		mpu.rawGyro[VEC3_Z]);
		
		// printf("Xa: %05d Ya: %05d Za: %05d ",
		// mpu.calibratedAccel[VEC3_X], 
		// mpu.calibratedAccel[VEC3_Y], 
		// mpu.calibratedAccel[VEC3_Z]);
		
		// printf("Xm: %03d Ym: %03d Zm: %03d ",
		// mpu.calibratedMag[VEC3_X], 
		// mpu.calibratedMag[VEC3_Y], 
		// mpu.calibratedMag[VEC3_Z]);

		// printf("W: %0.2f X: %0.2f Y: %0.2f Z: %0.2f ",
		// mpu.fusedQuat[QUAT_W],
		// mpu.fusedQuat[QUAT_X],
		// mpu.fusedQuat[QUAT_Y],
		// mpu.fusedQuat[QUAT_Z]);
		
			
		fflush(stdout);
	}
	return 0; 
}

    
int main(int argc, char *argv[]){
	int sample_rate;
	signed char orientation[9] = ORIENTATION_FLAT; 
	//signed char orientation[9] = ORIENTATION_UPRIGHT;
	
	if (argc==1){
		sample_rate = DEFAULT_SAMPLE_RATE;
    }
	else{
		sample_rate = atoi(argv[1]);
		if((sample_rate>MAX_SAMPLE_RATE)||(sample_rate<MIN_SAMPLE_RATE)){
			printf("sample rate should be between %d and %d\n", MIN_SAMPLE_RATE,MAX_SAMPLE_RATE);
			return -1;
		}
	}
	
	initialize_cape();
	initialize_imu(sample_rate, orientation);
	set_imu_interrupt_func(&print_imu_data); //start the interrupt handler
	
	//now just wait, print_imu_data will run
	while (get_state() != EXITING) {
		sleep(1);
	}
	cleanup_cape();
	return 0;
}

