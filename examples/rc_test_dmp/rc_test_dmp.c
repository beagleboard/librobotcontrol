/*******************************************************************************
* rc_test_dmp.c
*
* James Strawson - 2016
* Sample Code for testing MPU-9250 IMU operation with DMP and interrupts.
* Instructions are printed to the screen when called.
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

// Global Variables
int show_accel = 0;
int show_gyro  = 0;
int enable_mag = 0;
int show_compass = 0;
int show_temp  = 0;
int show_quat  = 0;
int show_tb = 0;
int orientation_menu = 0;
//struct to hold new data
rc_imu_data_t data; 

// local functions
rc_imu_orientation_t orientation_prompt();
void print_usage();
void print_data(); // imu interrupt function
void print_header();


/*******************************************************************************
* void print_usage()
*
* Printed if some invalid argument was given, or -h option given.
*******************************************************************************/
void print_usage(){
	printf("\n Options\n");
	printf("-s {rate}	Set sample rate in HZ (default 100)\n");
	printf("		Sample rate must be a divisor of 200\n");
	printf("-m		Enable Magnetometer\n");
	printf("-c		Show raw compass angle\n");
	printf("-a		Print Accelerometer Data\n");
	printf("-g		Print Gyro Data\n");
	printf("-t		Print TaitBryan Angles\n");
	printf("-q		Print Quaternion Vector\n");
	printf("-p {prio}	Set Interrupt Priority (default 98)\n");
	printf("-w		Print I2C bus warnings\n");
	printf("-o		Show a menu to select IMU orientation\n");
	printf("-h		Print this help message\n\n");
	
	return;
}

/*******************************************************************************
* void print_data()
*
* This is the IMU interrupt function.  
*******************************************************************************/
void print_data(){
	printf("\r");
	printf(" ");
	
	if(show_compass){
		printf("   %6.1f   |", data.compass_heading_raw*RAD_TO_DEG);
		printf("   %6.1f   |", data.compass_heading*RAD_TO_DEG);
	}
	if(show_quat && enable_mag){
		// print fused quaternion
		printf(" %4.1f %4.1f %4.1f %4.1f |", 	data.fused_quat[QUAT_W], \
												data.fused_quat[QUAT_X], \
												data.fused_quat[QUAT_Y], \
												data.fused_quat[QUAT_Z]);
	}
	else if(show_quat){
		// print quaternion
		printf(" %4.1f %4.1f %4.1f %4.1f |",	data.dmp_quat[QUAT_W], \
												data.dmp_quat[QUAT_X], \
												data.dmp_quat[QUAT_Y], \
												data.dmp_quat[QUAT_Z]);
	}
	if(show_tb && enable_mag){
		// print fused TaitBryan Angles
		printf("%6.1f %6.1f %6.1f |",	data.fused_TaitBryan[TB_PITCH_X]*RAD_TO_DEG,\
										data.fused_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG,\
										data.fused_TaitBryan[TB_YAW_Z]*RAD_TO_DEG);
	}
	else if(show_tb){
		// print TaitBryan angles
		printf("%6.1f %6.1f %6.1f |",	data.dmp_TaitBryan[TB_PITCH_X]*RAD_TO_DEG,\
										data.dmp_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG,\
										data.dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG);
	}
	if(show_accel){
		printf(" %5.2f %5.2f %5.2f |",	data.accel[0],\
										data.accel[1],\
										data.accel[2]);
	}
	if(show_gyro){
		printf(" %5.1f %5.1f %5.1f |",	data.gyro[0],\
										data.gyro[1],\
										data.gyro[2]);
	}
													
	fflush(stdout);
	return;
}

/*******************************************************************************
* void print_header()
*
* Based on which data is marked to be printed, print the correct labels.
* this is printed only once and the actual data is updated on the next line.
*******************************************************************************/
void print_header(){
	printf(" ");
	if(show_compass){
		printf("Raw Compass |");
		printf("FilteredComp|");
	}
	if(enable_mag){
		if(show_quat) printf("   Fused Quaternion  |");
		if(show_tb) printf(" FusedTaitBryan(deg) |");
	} else{
		if(show_quat) printf("    DMP Quaternion   |");
		if(show_tb) printf(" DMP TaitBryan (deg) |");
	}
	if(show_accel) printf("   Accel XYZ (g)   |");
	if(show_gyro) printf("  Gyro XYZ (deg/s) |");
	if(show_temp) printf(" Temp(C)");
	
	printf("\n");
}

/*******************************************************************************
* rc_imu_orientation_t orientation_prompt()
*
* If the user selects the -o option for orientation selection, this menu will
* displayed to prompt the user for which orientation to use. It will return 
* a valid rc_imu_orientation_t when a number 1-6 is given or quit when 'q' is 
* pressed. On other inputs the user will be allowed to enter again.
*******************************************************************************/
rc_imu_orientation_t orientation_prompt(){
	char c;
	
	printf("\n");
	printf("Please select a number 1-6 corresponding to the\n");
	printf("orientation you wish to use. Press 'q' to exit.\n\n");
	printf(" 1: ORIENTATION_Z_UP\n");
	printf(" 2: ORIENTATION_Z_DOWN\n");
	printf(" 3: ORIENTATION_X_UP\n");
	printf(" 4: ORIENTATION_X_DOWN\n");
	printf(" 5: ORIENTATION_Y_UP\n");
	printf(" 6: ORIENTATION_Y_DOWN\n");
	printf(" 7: ORIENTATION_X_FORWARD\n");
	printf(" 8: ORIENTATION_X_BACK\n");

    while ((c = getchar()) != EOF){
        switch(c){
        case '1':
            return ORIENTATION_Z_UP;
            break;
		case '2':
            return ORIENTATION_Z_DOWN;
            break;
		case '3':
            return ORIENTATION_X_UP;
            break;
		case '4':
            return ORIENTATION_X_DOWN;
            break;
		case '5':
            return ORIENTATION_Y_UP;
            break;
		case '6':
            return ORIENTATION_Y_DOWN;
            break;
        case '7':
            return ORIENTATION_X_FORWARD;
            break;
		case '8':
            return ORIENTATION_X_BACK;
            break;
        case 'q':
            printf("Quitting\n");
            exit(0);
            break;
		case '\n':
			break;
        default:
            printf("invalid input\n");
            break;
        }
    }
    return 0;
}

/*******************************************************************************
* int main(int argc, char *argv[])
*
* main() serves to parse user options, initialize the imu and interrupt handler,
* and wait for the rc_get_state()==EXITING condition before exiting cleanly.
* The imu_interrupt function print_data() is what actually prints new imu data
* to the screen after being set with rc_set_imu_interrupt_func().
*******************************************************************************/
int main(int argc, char *argv[]){
	int c, sample_rate, priority;
	int show_something = 0; // set to 1 when any show data option is given.
	
	// start with default config and modify based on options
	rc_imu_config_t conf = rc_default_imu_config();
	
	// parse arguments
	opterr = 0;
	while ((c=getopt(argc, argv, "s:magrqtcp:hwo"))!=-1 && argc>1){
		switch (c){
		case 's': // sample rate option
			sample_rate = atoi(optarg);
			if(sample_rate>200 || sample_rate<4){
				printf("sample_rate must be between 4 & 200");
				return -1;
			}
			conf.dmp_sample_rate = sample_rate;
			break;
		case 'p': // priority option
			priority = atoi(optarg);
			const int max_pri = sched_get_priority_max(SCHED_FIFO);
			if(priority>max_pri || priority<0){
				printf("priority must be between 0 & %d\n",max_pri);
				return -1;
			}
			conf.dmp_interrupt_priority = priority;
			break;
		case 'm': // magnetometer option
			show_something = 1;
			enable_mag = 1;
			conf.enable_magnetometer = 1;
			break;
		case 'c': // compass option
			show_something = 1;
			enable_mag = 1;
			show_compass = 1;
			conf.enable_magnetometer = 1;
			break;
		case 'a': // show accelerometer option
			show_something = 1;
			show_accel = 1;
			break;
		case 'g': // show gyro option
			show_something = 1;
			show_gyro = 1;
			break;
		case 'q': // show quaternion option
			show_something = 1;
			show_quat = 1;
			break;
		case 't': // show TaitBryan angle option
			show_something = 1;
			show_tb = 1;
			break;
		case 'T': // read thermometer option
			show_something = 1;
			show_temp = 1;
			break;
		case 'w': // print warnings
			conf.show_warnings=1;
			break;
		case 'o': // let user select imu orientation
			orientation_menu=1;
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
	// user didn't give an option to show anything. Print warning and return.
	if(show_something==0){
		print_usage();
		printf("please enable an option to print some data\n");
		return -1;
	}
	// If the user gave the -o option to select an orientation then prompt them
	if(orientation_menu){
		conf.orientation=orientation_prompt();
	}
	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}
	// now set up the imu for dmp interrupt operation
	if(rc_initialize_imu_dmp(&data, conf)){
		printf("rc_initialize_imu_failed\n");
		return -1;
	}
	// write labels for what data will be printed and associate the interrupt
	// function to print data immediately after the header.
	print_header();
	rc_set_imu_interrupt_func(&print_data);
	//now just wait, print_data() will be called by the interrupt
	while (rc_get_state()!=EXITING) {
		usleep(10000);
	}
	// shut things down
	rc_power_off_imu();
	rc_cleanup();
	return 0;
}

