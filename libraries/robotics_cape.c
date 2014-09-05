/*
Supporting library for Robotics Cape Features
Strawson Design - 2013
*/

#include "robotics_cape.h"

///////////////////////////////////////////////////
//////////////////  STATE VARIABLE  ///////////////
///////////////////////////////////////////////////
//control the flow of your loops and threads with this

enum state_t get_state(){
	return state;
}

int set_state(enum state_t new_state){
	state = new_state;
	return 0;
}


///////////////////////////////////////////////////
////////////////////  OUTPUT PINS   ///////////////
///////////////////////////////////////////////////
// gpio number is first digit *32 plus second digit
#define MDIR1A    20	//gpio0.20
#define MDIR1B    112	//gpio3.16
#define MDIR2A    113	//gpio3.17
#define MDIR2B    61	//gpio1.29
#define MDIR3A    49	//gpio1.17
#define MDIR3B    48	//gpio1.16
#define MDIR4A    65	//gpio2.1 
#define MDIR4B    27	//gpio0.27 
#define MDIR5A    26	//gpio0.26
#define MDIR5B    68	//gpio1.28
#define MDIR6A    68	//gpio2.4
#define MDIR6B    66	//gpio2.2
#define GRN_LED 47	// gpio1.15 "P8_15"
#define RED_LED 46	// gpio1.14 "P8_16"
//Spektrum UART4 RX must be remuxed to gpio output temporarily for pairing
#define PAIRING_PIN 30 //P9.11 gpio0.30
#define NUM_OUT_PINS 15

unsigned int out_gpio_pins[] = {MDIR1A, MDIR1B, MDIR2A, MDIR2B, 
								 MDIR3A, MDIR3B, MDIR4A, MDIR4B, 
								 MDIR5A, MDIR5B, MDIR5A, MDIR5B,
								 GRN_LED, RED_LED, PAIRING_PIN};

								 
	

	
///////////////////////////////////////////////////
////////////////////  INPUT PINS   ////////////////
///////////////////////////////////////////////////
#define START_BTN 67	//gpio2.3 P8_8
#define SELECT_BTN 69	//gpio2.6 P8_9
#define INTERRUPT_PIN 121  //gpio3.21 P9.25

int start_btn_state, select_btn_state;
int (*imu_interrupt_func)();
int (*start_unpressed_func)();
int (*start_pressed_func)();
int (*select_unpressed_func)();
int (*select_pressed_func)();

//function pointers for events initialized to null_fun
//instead of containing a null pointer
int null_func(){
	return 0;
}

int set_imu_interrupt_func(int (*func)(void)){
	imu_interrupt_func = func;
	return 0;
}
int set_start_pressed_func(int (*func)(void)){
	start_pressed_func = func;
	return 0;
}
int set_start_unpressed_func(int (*func)(void)){
	start_unpressed_func = func;
	return 0;
}
int set_select_pressed_func(int (*func)(void)){
	select_pressed_func = func;
	return 0;
}
int set_select_unpressed_func(int (*func)(void)){
	select_unpressed_func = func;
	return 0;
}

int get_start_button(){
	return start_btn_state;
}
int get_select_button(){
	return select_btn_state;
}



///////////////////////////////////////////////////
////////////////////  PWM PINS   //////////////////
///////////////////////////////////////////////////
//#define DEFAULT_PERIOD_NS    500000  //pwm period nanoseconds: 20khz
char pwm_files[][MAX_BUF] = {"/sys/devices/ocp.3/pwm_test_P9_31.12/",
							 "/sys/devices/ocp.3/pwm_test_P9_29.13/",
							 "/sys/devices/ocp.3/pwm_test_P9_14.15/",
							 "/sys/devices/ocp.3/pwm_test_P9_16.14/",
							 "/sys/devices/ocp.3/pwm_test_P8_19.16/",
							 "/sys/devices/ocp.3/pwm_test_P8_13.17/"
};
FILE *pwm_duty_pointers[6]; //store pointers to 6 pwm channels for frequent writes
int pwm_period_ns=0; //stores current pwm period in nanoseconds
#define MOTOR_DEFAULT_PERIOD_NS 25000 //40khz
#define ESC_DEFAULT_PERIOD_NS 10000000 //10ms, want to be longer than update interval

///////////////////////////////////////////////////
/////////////   eQEP Encoder Stuff   //////////////
///////////////////////////////////////////////////
char encoder_files[][MAX_BUF] = {"/sys/devices/ocp.3/48300000.epwmss/48300180.eqep/position",
								 "/sys/devices/ocp.3/48302000.epwmss/48302180.eqep/position",
								 "/sys/devices/ocp.3/48304000.epwmss/48304180.eqep/position"					
};
FILE *encoder_pointers[3]; //store pointers to 3 encoder channels for frequency reads
/*	
FILE *eqep0;
FILE *eqep1;
FILE *eqep2;
*/



///////////////////////////////////////////////////
//////  ADC AIN6 for Battery Monitoring    ////////
///////////////////////////////////////////////////
FILE *AIN6_file;
int millivolts;	


///////////////////////////////////////////////////
//////////   UART4 Spektrum Stuff   ///////////////
///////////////////////////////////////////////////

int rc_channels[RC_CHANNELS];
int rc_maxes[RC_CHANNELS];
int rc_mins[RC_CHANNELS];
int tty4_fd;
int rc_new_flag;






int initialize_cape(){
	set_state(RUNNING);
	
	printf("\n\nEnabling exit signal handler\n");
	signal(SIGINT, ctrl_c);	
	
	printf("Exporting GPIO pins\n");
	FILE *fd; //repeatedly used for different files
	char path[MAX_BUF]; //also repeatedly used to store file path string
	int i = 0; //general use counter
	for(i=0; i<NUM_OUT_PINS; i++){
		gpio_export(out_gpio_pins[i]);
	}
	printf("Setting GPIO Direction\n");
	for(i=0; i<NUM_OUT_PINS; i++){
		gpio_set_dir(out_gpio_pins[i], OUTPUT_PIN);
	}
	
	//set up function pointers for button and interrupt events
	set_imu_interrupt_func(&null_func);
	set_start_pressed_func(&null_func);
	set_start_unpressed_func(&null_func);
	set_select_pressed_func(&null_func);
	set_select_unpressed_func(&null_func);
    
	
	//Set up PWM
	//set correct polarity such that 'duty' is time spent HIGH
	printf("Initializing PWM\n");
	i=0;
	for(i=0; i<6; i++){
		strcpy(path, pwm_files[i]);
		strcat(path, "polarity");
		fd = fopen(path, "a");
		fprintf(fd,"%c",'0');
		fflush(fd);
		fclose(fd);
	}

	//set the pwm period in nanoseconds
	//set_pwm_period_ns(DEFAULT_PERIOD_NS);
	
	//leave duty open for future writes
	for(i=0; i<6; i++){
		strcpy(path, pwm_files[i]);
		strcat(path, "duty");
		pwm_duty_pointers[i] = fopen(path, "a");
		//set_motor(i+1,0); //set motor to free-spin
	}
	printf("Initializing eQep Encoders\n");
	//open encoder file pointers to make sure they work
	for(i=0; i<3; i++){
		strcpy(path, encoder_files[i]);
		encoder_pointers[i] = fopen(path, "r");
		if(encoder_pointers[i] <0){
			printf("Encoder Driver not loaded\n");
			printf("check is tieqep.ko is in the bootscript or load it manually.\n");
			return -1;
		}
		fclose(encoder_pointers[i]);
	}
	
	//  Spektrum RC setup on uart4
	// initialize_spektrum();
	
	printf("Starting Event Handler\n");
	pthread_t event_thread;
	pthread_create(&event_thread, NULL, read_events, (void*) NULL);
	

	printf("Battery Voltage = %fV\n", getBattVoltage());
	
	printf("Cape Initialized\n");
	printf("Pressing Ctrl-C will exit cleanly\n");
	return 0;
}


int set_motor(int motor, float duty){
	PIN_VALUE a;
	PIN_VALUE b;
	if(state == UNINITIALIZED){
		initialize_cape();
	}
	if(pwm_period_ns == 0){
		//printf("pwm period not set, using default");
		set_pwm_period_ns(MOTOR_DEFAULT_PERIOD_NS);
	}
	
	if(motor>6 || motor<1){
		printf("enter a motor value between 1 and 6\n");
		return -1;
	}
	
	//check that the duty cycle is within +-1
	if (duty>1){
		duty = 1;
	}
	else if(duty<-1){
		duty=-1;
	}

	//switch the direction pins to H-bridge
	if (duty>0){
	 	a=HIGH;
		b=LOW;
	}
	else{
		a=LOW;
		b=HIGH;
		duty=-duty;
	}
	gpio_set_value(out_gpio_pins[(motor-1)*2],a);
	gpio_set_value(out_gpio_pins[(motor-1)*2+1],b);

	fprintf(pwm_duty_pointers[motor-1], "%d", (int)(duty*pwm_period_ns));	
	fflush(pwm_duty_pointers[motor-1]);

	return 0;
}

int set_pwm_period_ns(int period){
	if(period <1){
		//printf("please use PWM period >1 (nanoseconds)\n");
		return -1;
	}
	int i=0;
	FILE *fd;
	char path[MAX_BUF];
	//printf("setting pwm run to 0\n");
	for(i=0; i<6; i++){
		strcpy(path, pwm_files[i]);
		strcat(path, "run");
		fd = fopen(path, "a");
		if(fd<0){
			printf("PWM run not available in /sys/class/devices/ocp.3\n");
			return -1;
		}
		fprintf(fd,"%d", 0);
		fflush(fd);
		fclose(fd);
	};
	//printf("setting period\n");
	for(i=0; i<6; i++){
		strcpy(path, pwm_files[i]);
		strcat(path, "period");
		fd = fopen(path, "a");
		if(fd<0){
			printf("PWM period not available in /sys/class/devices/ocp.3\n");
			return -1;
		}
		fprintf(fd,"%d", period);
		fflush(fd);
		fclose(fd);
	};
	//printf("setting pwm run to 1\n");
		for(i=0; i<6; i++){
		strcpy(path, pwm_files[i]);
		strcat(path, "run");
		fd = fopen(path, "a");
		if(fd<0){
			printf("PWM run not available in /sys/class/devices/ocp.3\n");
			return -1;
		}
		fprintf(fd,"%d", 1);
		fflush(fd);
		fclose(fd);
	};
	pwm_period_ns = period;
	return 0;
}

int set_esc(int esc, float normalized_duty){

	if(pwm_period_ns == 0){
		//printf("pwm period not set, using default");
		set_pwm_period_ns(ESC_DEFAULT_PERIOD_NS);
	}
	if(esc>6 || esc<1){
		printf("enter a esc value in the set {1,6}\n");
		return -1;
	}
	
	//check that the duty cycle is within +-1
	if (normalized_duty>1){
		normalized_duty = 1;
	}
	else if(normalized_duty<0){
		normalized_duty=0;
	}
	
	int duty_range_ns = DEFAULT_MAX_PULSE-DEFAULT_MIN_PULSE;
	int duty_ns = DEFAULT_MIN_PULSE + normalized_duty*duty_range_ns;
	fprintf(pwm_duty_pointers[esc-1], "%d", duty_ns);	
	fflush(pwm_duty_pointers[esc-1]);

	return 0;
}

int set_all_esc(float duty){
	int i;
	if(duty<0 || duty >1){
		printf("please set esc to duty in {0,1}\n");
		return -1;
	}
	for(i=0;i<6;i++){
		set_esc(i+1,duty);
	}
	return 0;
}
			
// kill_pwm() stops all communication to ECS by setting pulse duty to 0
// Unlike set_esc which keeps pulsing at DEFAULT_MIN_PULSE when power is
// set to zero to keep escs awake.
int kill_pwm(){
	int ch;
	for(ch=0;ch<6;ch++){
		fprintf(pwm_duty_pointers[ch], "%d", 0);	
		fflush(pwm_duty_pointers[ch]);
	}
	return 0;
}




long int get_encoder(int encoder){
	char path[MAX_BUF]; //repeatedly used to store file path string
	if((encoder<1)|(encoder >3)){
		printf("Enter encoder number between 1 & 3\n");
		return -1;
	}
	long int j;
	//rewind(encoder_pointers[encoder-1]);
	strcpy(path, encoder_files[encoder-1]);
	encoder_pointers[encoder-1] = fopen(path, "r");
	fscanf(encoder_pointers[encoder-1], "%li", &j);
	fclose(encoder_pointers[encoder-1]);
	return j;
}


int setGRN(PIN_VALUE i){
	return gpio_set_value(GRN_LED, i);
}

int setRED(PIN_VALUE i){
	return gpio_set_value(RED_LED, i);
}

float getBattVoltage(){
	int raw_adc;
	FILE *AIN6_fd;
	AIN6_fd = fopen("/sys/devices/ocp.3/helper.18/AIN6", "r");
	if(AIN6_fd < 0){
		printf("error reading adc\n");
		return -1;
	}
	fscanf(AIN6_fd, "%i", &raw_adc);
	fclose(AIN6_fd);
	return (float)raw_adc*11.0/1000.0; // time 11 for the voltage divider, divide by 1000 to go from mv to V
}


void* read_events(void* ptr){
	int fd;
    fd = open("/dev/input/event1", O_RDONLY);
    struct input_event ev;
	
	while (state != EXITING){
        read(fd, &ev, sizeof(struct input_event));
		//printf("type %i key %i state %i\n", ev.type, ev.code, ev.value);
        if(ev.type == 1){ //only new data
			switch(ev.code){
				case 0:
					if(ev.value == 1){
						(*imu_interrupt_func)();
					}
				break;
				//start button
				case 1:
					if(ev.value == 1){
						start_btn_state = 0; //unpressed
						(*start_unpressed_func)();
					}
					else{
						start_btn_state = 1; //pressed
						(*start_pressed_func)();
					}
				break;
				//select button
				case 2:	
					if(ev.value == 1){
						select_btn_state = 0; //unpressed
						(*select_unpressed_func)();
					}
					else{
						select_btn_state = 1; //pressed
						(*select_pressed_func)();
					}
				break;
			}
		}
		usleep(100000);
    }
	return NULL;
}

///////////////////////////////////////////////////
//////   Spektrum DSM2 RC Stuff   /////////////////
///////////////////////////////////////////////////

float get_rc_channel(int ch){
	if(ch<1 || ch > RC_CHANNELS){
		printf("please enter a channel between 1 & RC_CHANNELS");
		return -1;
	}
	float range = rc_maxes[ch-1]-rc_mins[ch-1];
	if(range!=0) {
		rc_new_flag = 0;
		float center = (rc_maxes[ch-1]+rc_mins[ch-1])/2;
		return 2*(rc_channels[ch-1]-center)/range;
	}
	else{
		return 0;
	}
	//rc_new_flag = 0;

}

int get_rc_new_flag(){
	return rc_new_flag;
}

void* uart4_checker(void *ptr){
	//set up sart/stop bit and 115200 baud
	struct termios config;
	//memset(&config,0,sizeof(config));
	config.c_iflag &= ~(BRKINT | ICRNL | IGNPAR);
	config.c_iflag=0;
    config.c_oflag=0;
    config.c_cflag= CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more info
    config.c_lflag=0;
    config.c_cc[VTIME]=5;
	if ((tty4_fd = open ("/dev/ttyO4", O_RDWR | O_NOCTTY)) < 0) {
		printf("error opening uart4\n");
	}
	if(cfsetispeed(&config, B115200) < 0) {
		printf("cannot set uart4 baud rate\n");
		return NULL;
	}
	
	if(tcsetattr(tty4_fd, TCSAFLUSH, &config) < 0) { 
		printf("cannot set uart4 attributes\n");
		return NULL;
	}
	char buf[2];
	int i;
	while(get_state()!=EXITING){
		read(tty4_fd, &buf, 2);
		if((buf[1]==0xFF) && (buf[0]==0xFF)){ //check if end of packet
			i=0;
			
			//printf(" 0xFF 0xFF");
			read(tty4_fd, &buf, 2); //clear next two useless packets
			if(buf[1] != 0xFF){
				//printf(" 0xFF");
				//printf(byte_to_binary(buf[1]));
			}
			else{//out of sync, end packet 0xFF FF FF not recognized
				read(tty4_fd, &buf, 1);
				//i=RC_CHANNELS+1;
			}
			//printf("\n");	
		}
		else if(i>=RC_CHANNELS){//something went wrong get back in sync
			//printf("\nuart4 packet sync error\n");
			do{
				read(tty4_fd, &buf, 1);
				if(buf[0]==0xFF){
					read(tty4_fd, &buf, 1);
				}
				read(tty4_fd, &buf, 2);
				i=0;
			} while (buf[0]!=0xFF);//congrats, two 0xFF packets in a row, new packet begins
		}
		else{
			//every pair of bytes is the raw integer for each channel
			rc_channels[i] = buf[0]<<8 ^ buf[1];
			rc_new_flag = 1; //new data to be read!!
			i++;
			
			/*
			printf("%d  ",rc_channels[i]);
			//printf("  ");
			printf(byte_to_binary(buf[0]));
			//printf(" ");
			printf(byte_to_binary(buf[1]));
			printf("   ");
			*/
		}
		usleep(1000);
		
	}
	printf("closing uart4 thread\n");
	close(tty4_fd);
	return 0;
}

int calibrate_spektrum(){
	printf("\n\nTurn on your Transmitter and connect receiver.\n");
	printf("Move all sticks and switches through their range of motion.\n");
	printf("Measured raw min/max positions will display below.\n");
	printf("Press the start button when done.\n");
	printf("\nWaiting on Radio Connection. Satellite receiver LED should illuminate.\n");
	printf("Ch: current min max\n");
	
	//start the signal handler to stop calibrating and write to file
	//register_sig_handler();
	
	//fire up the reading thread
	set_state(RUNNING);
	
	while(get_rc_new_flag()==0){
		usleep(100000); //wait for data to start
		if (get_state()==EXITING){
			//return 0;
			break;
		}
	}
	usleep(100000); //wait for packet to finish
	//start mins at current value
	int j,k;
	for(j=0;j<RC_CHANNELS;j++){
		rc_mins[j]=rc_channels[j];
		rc_maxes[j]=rc_channels[j];
	}
	if(get_start_button()==HIGH){
		setGRN(HIGH);
		
	}
	while(get_state()!=EXITING){
		printf("\r");
		if (get_rc_new_flag() == 0){
			if (k==1){
				set_state(EXITING); 
				printf("\nRadio Disconnected\n");
			}
			else k=1;
		}
		else{
			for(j=0;j<RC_CHANNELS;j++){
				if(get_start_button()==HIGH){
					set_state(EXITING);
					break;
				}
				else if(get_state()==EXITING){
					break; //since we are in a for loop, check state frequently
				}
				else if(rc_channels[j]>rc_maxes[j])
					rc_maxes[j]=rc_channels[j];
				else if(rc_channels[j]<rc_mins[j])
					rc_mins[j]=rc_channels[j];
				
				if(rc_channels[j] != 0){ //show only non-zero channels
					//printf("ch%d %d %d %d  ", j+1, rc_channels[j], rc_mins[j], rc_maxes[j]);
					printf("%d %d  ", rc_mins[j], rc_maxes[j]);
				}
				rc_new_flag = 0;
			}
		}
		usleep(100000);
	}
	//if it looks like new data came in, write calibration file
	if((rc_mins[0]!=0)&&(rc_mins[0]!=rc_maxes[0])){ 
		int fd;
		fd = open(SPEKTRUM_CAL_FILE, O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);

		if (fd < 0) {
			printf("\n error opening calibration file for writing\n");
			return -1;
		}
		char buff[64];
		for(j=0;j<RC_CHANNELS;j++){
				sprintf(buff, "%d %d\n", rc_mins[j], rc_maxes[j]);
				write(fd, buff, strlen(buff));
		}
		close(fd);
		printf("\n new calibration file written\n");
	}
	
	return 0;
}

int initialize_spektrum(){
	//signal(SIGINT, ctrl_c);	//signal catcher calls cleanup function on exit
	//if calibration file exists, load it and start spektrum thread
	FILE *cal;
	cal = fopen(SPEKTRUM_CAL_FILE, "r");
	if (cal < 0) {
		printf("\nSpektrum Calibration File Doesn't Exist Yet\n");
		printf("Use calibrate_spektrum example to create one\n");
		return -1;
	}
	else{
		int i;
		for(i=0;i<RC_CHANNELS;i++){
			fscanf(cal,"%d %d", &rc_mins[i],&rc_maxes[i]);
			//printf("%d %d\n", rc_mins[i],rc_maxes[i]);
		}
		printf("Spektum Calibration Loaded\n");
	}
	fclose(cal);
	pthread_t uart4_thread;
	pthread_create(&uart4_thread, NULL, uart4_checker, (void*) NULL);
	printf("Spektrum Thread Started\n");
	return 0;
}



//////////////////////////////////////////////
/////// Exiting and closing handlers  ////////
//////////////////////////////////////////////

void ctrl_c(int signo){
	if (signo == SIGINT){
		set_state(EXITING);
		printf("\nreceived SIGINT Ctrl-C\n");
		//sleep(1);
		// in case of ctrl-c signal, shut down cleanly
		//cleanup_cape();
		//exit(EXIT_SUCCESS);
 	}
}

int cleanup_cape(){
	set_state(EXITING); 

	setGRN(0);
	setRED(0);	

	//printf("Closing PWM\n");
	kill_pwm();
	
	printf("\nExiting Cleanly\n");
	return 0;
}

//// MPU9150 IMU ////
int initialize_imu(int sample_rate){

	signed char gyro_orientation[9] = { 1, 0, 0,
                                        0, 1, 0,
                                        0, 0, 1 };

	linux_set_i2c_bus(1);

	printf("Initializing IMU .");
	fflush(stdout);

	if (mpu_init(NULL)) {
		printf("\nmpu_init() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);

	if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS)) {
		printf("\nmpu_set_sensors() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);

	if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {
		printf("\nmpu_configure_fifo() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);
	
	if (mpu_set_sample_rate(sample_rate)) {
		printf("\nmpu_set_sample_rate() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);
	
	if(sample_rate > 100){
		if(mpu_set_compass_sample_rate(100)){
			printf("\nmpu_set_compass_sample_rate() failed\n");
			return -1;}
	}		
	else{
		if(mpu_set_compass_sample_rate(sample_rate)){
			printf("\nmpu_set_compass_sample_rate() failed\n");
			return -1;}
	}

	printf(".");
	fflush(stdout);

	if (dmp_load_motion_driver_firmware()) {
		printf("\ndmp_load_motion_driver_firmware() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);

	if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) {
		printf("\ndmp_set_orientation() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);

  	if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL 
						| DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL)) {
		printf("\ndmp_enable_feature() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);
 
	if (dmp_set_fifo_rate(sample_rate)) {
		printf("\ndmp_set_fifo_rate() failed\n");
		return -1;
	}

	printf(".\n");
	fflush(stdout);

	if (mpu_set_dmp_state(1)) {
		printf("\nmpu_set_dmp_state(1) failed\n");
		return -1;
	}
	return 0;
}

/////////////////////////////////////
//// Timing and support Functions ///
/////////////////////////////////////

const char *byte_to_binary(int x){
    static char b[9];
    b[0] = '\0';

    int z;
    for (z = 128; z > 0; z >>= 1)
    {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }

    return b;
}

timespec diff(timespec start, timespec end){
	timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000L+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}
