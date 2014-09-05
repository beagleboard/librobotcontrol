// test_imu for Robotics Cape Project
// James Strawson - 2013

#include <robotics_cape.h>

#define SAMPLE_RATE_HZ	20

int main(){
	initialize_cape();
	
	if(initialize_imu(SAMPLE_RATE_HZ)){
		exit(1);}
		
	mpudata_t mpu;
	memset(&mpu, 0, sizeof(mpudata_t));

	printf("\nEntering read loop (ctrl-c to exit)\n\n");
	unsigned long loop_delay;
	loop_delay = (1000 / SAMPLE_RATE_HZ) - 2;
	linux_delay_ms(loop_delay);

	while (get_state() != EXITING) {
		if (mpu9150_read(&mpu) == 0) {
			 printf("\rX: %0.1f Y: %0.1f Z: %0.1f        ",
			 mpu.fusedEuler[VEC3_X] * RAD_TO_DEGREE, 
			 mpu.fusedEuler[VEC3_Y] * RAD_TO_DEGREE, 
			 mpu.fusedEuler[VEC3_Z] * RAD_TO_DEGREE);
			// printf("\nX: %0.1f  ",
			// mpu.fusedEuler[VEC3_X] * RAD_TO_DEGREE);
			fflush(stdout);
			
			// printf("\rW: %0.2f X: %0.2f Y: %0.2f Z: %0.2f        ",
			// mpu->fusedQuat[QUAT_W],
			// mpu->fusedQuat[QUAT_X],
			// mpu->fusedQuat[QUAT_Y],
			// mpu->fusedQuat[QUAT_Z]);
			// fflush(stdout);
			
			// printf("\rX: %05d Y: %05d Z: %05d        ",
			// mpu->calibratedAccel[VEC3_X], 
			// mpu->calibratedAccel[VEC3_Y], 
			// mpu->calibratedAccel[VEC3_Z]),
			// fflush(stdout);

			// printf("\rX: %03d Y: %03d Z: %03d        ",
			// mpu->calibratedMag[VEC3_X], 
			// mpu->calibratedMag[VEC3_Y], 
			// mpu->calibratedMag[VEC3_Z]);
			// fflush(stdout);
		}

		linux_delay_ms(loop_delay);
	}
	cleanup_cape();
	return 0;
}

