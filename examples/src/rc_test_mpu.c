/**
 * @file rc_test_mpu.c
 * @example    rc_test_mpu
 *
 * @brief      serves as an example of how to use the MPU in normal mode.
 *
 *             This does not use the DMP, for a demonstration of DMP
 *             functionality see rc_test_dmp.
 *
 * @author     James Strawson
 * @date       1/29/2018
 */


#include <stdio.h>
#include <signal.h>
#include <getopt.h>
#include <rc/mpu.h>
#include <rc/time.h>

// bus for Robotics Cape and BeagleboneBlue is 2
// change this for your platform
#define I2C_BUS 2

// possible modes, user selected with command line arguments
typedef enum g_mode_t{
	G_MODE_RAD,
	G_MODE_DEG,
	G_MODE_RAW
} g_mode_t;

typedef enum a_mode_t{
	A_MODE_MS2,
	A_MODE_G,
	A_MODE_RAW
} a_mode_t;

static int enable_magnetometer = 0;
static int enable_thermometer = 0;
static int enable_warnings = 0;
static int running = 0;

// printed if some invalid argument was given
static void __print_usage(void)
{
	printf("\n");
	printf("-a	print raw adc values instead of radians\n");
	printf("-r	print gyro in radians/s instead of degrees/s\n");
	printf("-g	print acceleration in G instead of m/s^2\n");
	printf("-m	print magnetometer data as well as accel/gyro\n");
	printf("-t	print thermometer data as well as accel/gyro\n");
	printf("-w	print i2c warnings\n");
	printf("-h	print this help message\n");
	printf("\n");
}

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}

int main(int argc, char *argv[])
{
	rc_mpu_data_t data; //struct to hold new data
	int c;
	g_mode_t g_mode = G_MODE_DEG; // gyro default to degree mode.
	a_mode_t a_mode = A_MODE_MS2; // accel default to m/s^2

	// parse arguments
	opterr = 0;
	while ((c = getopt(argc, argv, "argmtwh")) != -1){
		switch (c){
		case 'a':
			g_mode = G_MODE_RAW;
			a_mode = A_MODE_RAW;
			printf("\nRaw values are from 16-bit ADC\n");
			break;
		case 'r':
			g_mode = G_MODE_RAD;
			break;
		case 'g':
			a_mode = A_MODE_G;
			break;
		case 'm':
			enable_magnetometer = 1;
			break;
		case 't':
			enable_thermometer = 1;
			break;
		case 'w':
			enable_warnings = 1;
			break;
		case 'h':
			__print_usage();
			return 0;
		default:
			__print_usage();
			return -1;
		}
	}

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running = 1;

	// use defaults for now, except also enable magnetometer.
	rc_mpu_config_t conf = rc_mpu_default_config();
	conf.i2c_bus = I2C_BUS;
	conf.enable_magnetometer = enable_magnetometer;
	conf.show_warnings = enable_warnings;

	if(rc_mpu_initialize(&data, conf)){
		fprintf(stderr,"rc_mpu_initialize_failed\n");
		return -1;
	}

	// print the header
	printf("\ntry 'rc_test_mpu -h' to see other options\n\n");
	switch(a_mode){
	case A_MODE_MS2:
		printf("   Accel XYZ(m/s^2)  |");
		break;
	case A_MODE_G:
		printf("     Accel XYZ(G)    |");
		break;
	case A_MODE_RAW:
		printf("  Accel XYZ(raw ADC) |");
		break;
	default:
		printf("ERROR: invalid mode\n");
		return -1;
	}
	switch(g_mode){
	case G_MODE_RAD:
		printf("   Gyro XYZ (rad/s)  |");
		break;
	case G_MODE_DEG:
		printf("   Gyro XYZ (deg/s)  |");
		break;
	case G_MODE_RAW:
		printf("  Gyro XYZ (raw ADC) |");
		break;
	default:
		printf("ERROR: invalid mode\n");
		return -1;
	}

	if(enable_magnetometer)	printf("  Mag Field XYZ(uT)  |");
	if(enable_thermometer) printf(" Temp (C)");
	printf("\n");

	//now just wait, print_data will run
	while (running) {
		printf("\r");

		// read sensor data
		if(rc_mpu_read_accel(&data)<0){
			printf("read accel data failed\n");
		}
		if(rc_mpu_read_gyro(&data)<0){
			printf("read gyro data failed\n");
		}
		if(enable_magnetometer && rc_mpu_read_mag(&data)){
			printf("read mag data failed\n");
		}
		if(enable_thermometer && rc_mpu_read_temp(&data)){
			printf("read imu thermometer failed\n");
		}


		switch(a_mode){
		case A_MODE_MS2:
			printf("%6.2f %6.2f %6.2f |",	data.accel[0],\
							data.accel[1],\
							data.accel[2]);
			break;
		case A_MODE_G:
			printf("%6.2f %6.2f %6.2f |",	data.accel[0]*MS2_TO_G,\
							data.accel[1]*MS2_TO_G,\
							data.accel[2]*MS2_TO_G);
			break;
		case A_MODE_RAW:
			printf("%6d %6d %6d |",		data.raw_accel[0],\
							data.raw_accel[1],\
							data.raw_accel[2]);
		}

		switch(g_mode){
		case G_MODE_RAD:
			printf("%6.1f %6.1f %6.1f |",	data.gyro[0]*DEG_TO_RAD,\
							data.gyro[1]*DEG_TO_RAD,\
							data.gyro[2]*DEG_TO_RAD);
			break;
		case G_MODE_DEG:
			printf("%6.1f %6.1f %6.1f |",	data.gyro[0],\
							data.gyro[1],\
							data.gyro[2]);
			break;
		case G_MODE_RAW:
			printf("%6d %6d %6d |",		data.raw_gyro[0],\
							data.raw_gyro[1],\
							data.raw_gyro[2]);
		}

		// read magnetometer
		if(enable_magnetometer){
			printf("%6.1f %6.1f %6.1f |",	data.mag[0],\
							data.mag[1],\
							data.mag[2]);
		}
		// read temperature
		if(enable_thermometer){
			printf(" %4.1f    ", data.temp);
		}

		fflush(stdout);
		rc_usleep(100000);
	}
	printf("\n");
	rc_mpu_power_off();
	return 0;
}

