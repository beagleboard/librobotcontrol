/*
Supporting library for Robotics Cape Features
Strawson Design - 2013
*/

#include "robotics_cape.h"

// state variable for loop and thread control
enum state_t state = UNINITIALIZED;

enum state_t get_state(){
	return state;
}

int set_state(enum state_t new_state){
	state = new_state;
	return 0;
}



// gpio stuff
unsigned int out_gpio_pins[] = {MDIR1A, MDIR1B, MDIR2A, MDIR2B, 
								 MDIR3A, MDIR3B, MDIR4A, MDIR4B,
								 GRN_LED, RED_LED, PAIRING_PIN};
								 
// buttons
int start_btn_state, select_btn_state;
int (*imu_interrupt_func)();
int (*start_unpressed_func)();
int (*start_pressed_func)();
int (*select_unpressed_func)();
int (*select_pressed_func)();

//function pointers for events initialized to null_func()
//instead of containing a null pointer
int null_func(){
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


// pwm stuff
char pwm_files[][MAX_BUF] = {"/sys/devices/ocp.3/pwm_test_P8_34.12/",
							 "/sys/devices/ocp.3/pwm_test_P8_36.13/",
							 "/sys/devices/ocp.3/pwm_test_P8_19.16/",
							 "/sys/devices/ocp.3/pwm_test_P8_13.17/"
};
FILE *pwm_duty_pointers[6]; //store pointers to 6 pwm channels for frequent writes
int pwm_period_ns=0; //stores current pwm period in nanoseconds


// eQEP Encoder mmap arrays
volatile char *pwm_map_base[3];

// ADC AIN6 for Battery Monitoring
FILE *AIN6_file;
int millivolts;	

// DSM2 Spektrum radio & UART4
int rc_channels[RC_CHANNELS];
int rc_maxes[RC_CHANNELS];
int rc_mins[RC_CHANNELS];
int tty4_fd;
int new_dsm2_flag;


int initialize_cape(){
	FILE *fd; 			// opened and closed for each file
	char path[MAX_BUF]; // buffer to store file path string
	int i = 0; 			// general use counter
	
	printf("\nInitializing GPIO\n");
	for(i=0; i<NUM_OUT_PINS; i++){
		gpio_export(out_gpio_pins[i]);
		gpio_set_dir(out_gpio_pins[i], OUTPUT_PIN);
	}

	//Set up PWM
	printf("Initializing PWM\n");
	i=0;
	for(i=0; i<4; i++){
		strcpy(path, pwm_files[i]);
		strcat(path, "polarity");
		fd = fopen(path, "a");
		if(fd<0){
			printf("PWM polarity not available in /sys/class/devices/ocp.3\n");
			return -1;
		}
		//set correct polarity such that 'duty' is time spent HIGH
		fprintf(fd,"%c",'0');
		fflush(fd);
		fclose(fd);
	}
	
	//leave duty cycle file open for future writes
	for(i=0; i<4; i++){
		strcpy(path, pwm_files[i]);
		strcat(path, "duty");
		pwm_duty_pointers[i] = fopen(path, "a");
	}
	
	//read in the pwm period defined in device tree overlay .dts
	strcpy(path, pwm_files[0]);
	strcat(path, "period");
	fd = fopen(path, "r");
	if(fd<0){
		printf("PWM period not available in /sys/class/devices/ocp.3\n");
		return -1;
	}
	fscanf(fd,"%i", &pwm_period_ns);
	fclose(fd);
	
	// mmap pwm modules to get fast access to eQep encoder position
	// see mmap_eqep example program for more mmap and encoder info
	printf("Initializing eQep Encoders\n");
	int dev_mem;
	if ((dev_mem = open("/dev/mem", O_RDWR | O_SYNC))==-1){
	  printf("Could not open /dev/mem \n");
	  return -1;
	}
	pwm_map_base[0] = mmap(0,getpagesize(),PROT_READ|PROT_WRITE,MAP_SHARED,dev_mem,PWM0_BASE);
	pwm_map_base[1] = mmap(0,getpagesize(),PROT_READ|PROT_WRITE,MAP_SHARED,dev_mem,PWM1_BASE);
	pwm_map_base[2] = mmap(0,getpagesize(),PROT_READ|PROT_WRITE,MAP_SHARED,dev_mem,PWM2_BASE);
	if(pwm_map_base[0] == (void *) -1) {
		printf("Unable to mmap pwm \n");
		return(-1);
	}
	close(dev_mem);
	
	// Test eqep and reset position
	for(i=1;i<3;i++){
		if(set_encoder_pos(i,0)){
			printf("failed to access eQep register\n");
			printf("eQep driver not loaded\n");
			return -1;
		}
	}
	
	//set up function pointers for button press events
	set_start_pressed_func(&null_func);
	set_start_unpressed_func(&null_func);
	set_select_pressed_func(&null_func);
	set_select_unpressed_func(&null_func);
	
	//event handler thread for buttons
	printf("Starting Event Handler\n");
	pthread_t event_thread;
	pthread_create(&event_thread, NULL, read_events, (void*) NULL);
	
	// Print current battery voltage
	printf("Battery Voltage = %fV\n", getBattVoltage());
	
	// Start Signal Handler
	printf("Enabling exit signal handler\n");
	signal(SIGINT, ctrl_c);	
	
	// all done
	set_state(PAUSED);
	printf("\nRobotics Cape Initialized\n");

	return 0;
}

// set a motor direction and power
// motor is from 1 to 6
// duty is from -1 to +1
int set_motor(int motor, float duty){
	PIN_VALUE a;
	PIN_VALUE b;
	if(state == UNINITIALIZED){
		initialize_cape();
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
	if (duty>=0){
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

int kill_pwm(){
	int ch;
	for(ch=0;ch<6;ch++){
		fprintf(pwm_duty_pointers[ch], "%d", 0);	
		fflush(pwm_duty_pointers[ch]);
	}
	return 0;
}


//// eQep Encoder read/write
long int get_encoder_pos(int ch){
	if(ch<1 || ch>3){
		printf("Encoder Channel must be in 1, 2, or 3\n");
		return -1;
	}
	return  *(unsigned long*)(pwm_map_base[ch-1] + EQEP_OFFSET +QPOSCNT);
}

int set_encoder_pos(int ch, long value){
	if(ch<1 || ch>3){
		printf("Encoder Channel must be 1, 2 or 3\n");
		return -1;
	}
	*(unsigned long*)(pwm_map_base[ch-1] + EQEP_OFFSET +QPOSCNT) = value;
	return 0;
}


//// LED functions
// PIN_VALUE and be HIGH or LOW
int setGRN(PIN_VALUE i){
	return gpio_set_value(GRN_LED, i);
}
int setRED(PIN_VALUE i){
	return gpio_set_value(RED_LED, i);
}

//// Read battery voltage
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
	// times 11 for the voltage divider, divide by 1000 to go from mv to V
	return (float)raw_adc*11.0/1000.0; 
}

//// Button event handler thread
void* read_events(void* ptr){
	int fd;
    fd = open("/dev/input/event1", O_RDONLY);
    struct input_event ev;
	while (state != EXITING){
        read(fd, &ev, sizeof(struct input_event));
		// uncomment printf to see how event codes work
		// printf("type %i key %i state %i\n", ev.type, ev.code, ev.value); 
        if(ev.type == 1){ //only new data
			switch(ev.code){
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
		usleep(5000);
    }
	return NULL;
}


////  DSM2  Spektrum  RC Stuff  
int initialize_dsm2(){
	//if calibration file exists, load it and start spektrum thread
	FILE *cal;
	cal = fopen(DSM2_CAL_FILE, "a+");
	if (cal < 0) {
		printf("\nDSM2 Calibration File Doesn't Exist Yet\n");
		printf("Use calibrate_dsm2 example to create one\n");
		return -1;
	}
	else{
		int i;
		for(i=0;i<RC_CHANNELS;i++){
			fscanf(cal,"%d %d", &rc_mins[i],&rc_maxes[i]);
			//printf("%d %d\n", rc_mins[i],rc_maxes[i]);
		}
		printf("DSM2 Calibration Loaded\n");
	}
	fclose(cal);
	pthread_t uart4_thread;
	pthread_create(&uart4_thread, NULL, uart4_checker, (void*) NULL);
	printf("DSM2 Thread Started\n");
	return 0;
}

float get_dsm2_ch_normalized(int ch){
	if(ch<1 || ch > RC_CHANNELS){
		printf("please enter a channel between 1 & %d",RC_CHANNELS);
		return -1;
	}
	float range = rc_maxes[ch-1]-rc_mins[ch-1];
	if(range!=0) {
		new_dsm2_flag = 0;
		float center = (rc_maxes[ch-1]+rc_mins[ch-1])/2;
		return 2*(rc_channels[ch-1]-center)/range;
	}
	else{
		return 0;
	}
}

int get_dsm2_ch_raw(int ch){
	if(ch<1 || ch > RC_CHANNELS){
		printf("please enter a channel between 1 & %d",RC_CHANNELS);
		return -1;
	}
	else{
		new_dsm2_flag = 0;
		return rc_channels[ch-1];
	}
}

int is_new_dsm2_data(){
	return new_dsm2_flag;
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
	if ((tty4_fd = open (UART4_PATH, O_RDWR | O_NOCTTY)) < 0) {
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
			new_dsm2_flag = 1; //new data to be read!!
			i++;
			// printf("%d  ",rc_channels[i]);
			// printf("  ");
			// printf(byte_to_binary(buf[0]));
			// printf(" ");
			// printf(byte_to_binary(buf[1]));
			// printf("   ");
		}
		usleep(1000);
		
	}
	printf("closing uart4 thread\n");
	close(tty4_fd);
	return 0;
}



//// MPU9150 IMU ////
int initialize_imu(int sample_rate, signed char orientation[9]){
	printf("Initializing IMU\n");
	//set up gpio interrupt pin connected to imu
	if(gpio_export(INTERRUPT_PIN)){
		printf("can't export gpio %d \n", INTERRUPT_PIN);
		return (-1);
	}
	gpio_set_dir(INTERRUPT_PIN, INPUT_PIN);
	gpio_set_edge(INTERRUPT_PIN, "falling");  // Can be rising, falling or both
		
	linux_set_i2c_bus(1);

	
	if (mpu_init(NULL)) {
		printf("\nmpu_init() failed\n");
		return -1;
	}
 
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
	mpu_set_sample_rate(sample_rate);
	
	// compass run at 100hz max. 
	if(sample_rate > 100){
		// best to sample at a fraction of the gyro/accel
		mpu_set_compass_sample_rate(sample_rate/2);
	}
	else{
		mpu_set_compass_sample_rate(sample_rate);
	}
	mpu_set_lpf(188); //as little filtering as possible
	dmp_load_motion_driver_firmware(sample_rate);

	dmp_set_orientation(inv_orientation_matrix_to_scalar(orientation));
  	dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL 
						| DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);

	dmp_set_fifo_rate(sample_rate);
	
	if (mpu_set_dmp_state(1)) {
		printf("\nmpu_set_dmp_state(1) failed\n");
		return -1;
	}
	if(loadGyroCalibration()){
		printf("\nGyro Calibration File Doesn't Exist Yet\n");
		printf("Use calibrate_gyro example to create one\n");
		printf("Using 0 offset for now\n");
	};
	
	// set the imu_interrupt_thread to highest priority since this is used
	// for real-time control with time-sensitive IMU data
	set_imu_interrupt_func(&null_func);
	pthread_t imu_interrupt_thread;
	struct sched_param params;
	pthread_create(&imu_interrupt_thread, NULL, imu_interrupt_handler, (void*) NULL);
	params.sched_priority = sched_get_priority_max(SCHED_FIFO);
	pthread_setschedparam(imu_interrupt_thread, SCHED_FIFO, &params);
	
	return 0;
}

int setXGyroOffset(int16_t offset) {
	uint16_t new = offset;
	const unsigned char msb = (unsigned char)((new&0xff00)>>8);
	const unsigned char lsb = (new&0x00ff);//get LSB
	//printf("writing: 0x%x 0x%x 0x%x \n", new, msb, lsb);
	linux_i2c_write(MPU_ADDR, MPU6050_RA_XG_OFFS_USRH, 1, &msb);
	return linux_i2c_write(MPU_ADDR, MPU6050_RA_XG_OFFS_USRL, 1, &lsb);
}

int setYGyroOffset(int16_t offset) {
	uint16_t new = offset;
	const unsigned char msb = (unsigned char)((new&0xff00)>>8);
	const unsigned char lsb = (new&0x00ff);//get LSB
	//printf("writing: 0x%x 0x%x 0x%x \n", new, msb, lsb);
	linux_i2c_write(MPU_ADDR, MPU6050_RA_YG_OFFS_USRH, 1, &msb);
	return linux_i2c_write(MPU_ADDR, MPU6050_RA_YG_OFFS_USRL, 1, &lsb);
}

int setZGyroOffset(int16_t offset) {
	uint16_t new = offset;
	const unsigned char msb = (unsigned char)((new&0xff00)>>8);
	const unsigned char lsb = (new&0x00ff);//get LSB
	//printf("writing: 0x%x 0x%x 0x%x \n", new, msb, lsb);
	linux_i2c_write(MPU_ADDR, MPU6050_RA_ZG_OFFS_USRH, 1, &msb);
	return linux_i2c_write(MPU_ADDR, MPU6050_RA_ZG_OFFS_USRL, 1, &lsb);
}

int loadGyroCalibration(){
	FILE *cal;
	cal = fopen(GYRO_CAL_FILE, "r");
	if (cal == 0) {
		// calibration file doesn't exist yet
		return -1;
	}
	else{
		int xoffset, yoffset, zoffset;
		fscanf(cal,"%d\n%d\n%d\n", &xoffset, &yoffset, &zoffset);
		if(setXGyroOffset((int16_t)xoffset)){
			printf("problem setting gyro offset\n");
			return -1;
		}
		if(setYGyroOffset((int16_t)yoffset)){
			printf("problem setting gyro offset\n");
			return -1;
		}
		if(setZGyroOffset((int16_t)zoffset)){
			printf("problem setting gyro offset\n");
			return -1;
		}
	}
	fclose(cal);
	return 0;
}

// IMU interrupt stuff
// see test_imu and calibrate_gyro for examples
int set_imu_interrupt_func(int (*func)(void)){
	imu_interrupt_func = func;
	return 0;
}

void* imu_interrupt_handler(void* ptr){
	struct pollfd fdset[1];
	char buf[MAX_BUF];
	int imu_gpio_fd = gpio_fd_open(INTERRUPT_PIN);
	fdset[0].fd = imu_gpio_fd;
	fdset[0].events = POLLPRI; // high-priority interrupt
	// keep running until the program closes
	while(get_state() != EXITING) {
		// system hangs here until IMU FIFO interrupt
		poll(fdset, 1, POLL_TIMEOUT);        
		if (fdset[0].revents & POLLPRI) {
			lseek(fdset[0].fd, 0, SEEK_SET);  
			read(fdset[0].fd, buf, MAX_BUF);
			// user selectable with set_inu_interrupt_func() defined above
			imu_interrupt_func(); 
		}
	}
	gpio_fd_close(imu_gpio_fd);
	return 0;
}



//// helpful functions
char *byte_to_binary(unsigned char x){
    static char b[9];
    b[0] = '\0';
    unsigned char z;
    for (z = 128; z > 0; z >>= 1){
        if(x&z)
			strcat(b, "1");
		else
			strcat(b, "0");
    }
    return b;
}

// find the difference between two timespec structs
// used with sleep_ns() function and accurately timed loops
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

uint64_t microsSinceEpoch(){
	struct timeval tv;
	uint64_t micros = 0;
	gettimeofday(&tv, NULL);  
	micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	return micros;
}


// Mavlink easy setup for UDP
// This function mostly taken from Bryan Godbolt's mavlink_udp example
struct sockaddr_in initialize_mavlink_udp(char gc_ip_addr[],  int *udp_sock){
	struct sockaddr_in gcAddr;
	char target_ip[100];
	struct sockaddr_in locAddr;
	int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	
	strcpy(target_ip, gc_ip_addr);
	memset(&locAddr, 0, sizeof(locAddr));
	locAddr.sin_family = AF_INET;
	locAddr.sin_addr.s_addr = INADDR_ANY;
	locAddr.sin_port = htons(14551); //uav listening port for mavlink
	/* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */ 
	if (-1 == bind(sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr))){
		perror("error bind failed");
		close(sock);
		exit(EXIT_FAILURE);
    }

	memset(&gcAddr, 0, sizeof(&gcAddr));
	gcAddr.sin_family = AF_INET;
	gcAddr.sin_addr.s_addr = inet_addr(target_ip);
	gcAddr.sin_port = htons(14550);
	*udp_sock = sock;
	printf("Initialized Mavlink with Ground Control address ");
	printf(target_ip);
	printf("\n");
	return gcAddr;
}


//// SPI0   see test_adns9800 example
int initialize_spi0(){ // returns a file descriptor to spi device
	int spi0_fd;
	spi0_fd = open("/dev/spidev1.0", O_RDWR); // actually spi0
    if (spi0_fd<=0) {  
        printf("/dev/spidev1.0 not found\n"); 
        return -1; 
    } 
	if(gpio_export(SPI0_SS0_GPIO_PIN)){
		printf("failed to export gpio0[5] p9.17\n"); 
        return -1; 
	}
	gpio_set_dir(SPI0_SS0_GPIO_PIN, OUTPUT_PIN);
	gpio_set_value(SPI0_SS0_GPIO_PIN, HIGH);
	
	return spi0_fd;
}
// TODO: support multiple slaves
int select_spi0_slave(int slave){
	return gpio_set_value(SPI0_SS0_GPIO_PIN, LOW);
}	
int deselect_spi0_slave(int slave){
	return gpio_set_value(SPI0_SS0_GPIO_PIN, HIGH);
}	

// catch Ctrl-C signal and change system state
// all threads should watch for get_state()==EXITING and shut down cleanly
void ctrl_c(int signo){
	if (signo == SIGINT){
		set_state(EXITING);
		printf("\nreceived SIGINT Ctrl-C\n");
 	}
}

// cleanup_cape() should be at the end of every main() function
int cleanup_cape(){
	set_state(EXITING); 
	setGRN(0);
	setRED(0);	
	kill_pwm();
	printf("\nExiting Cleanly\n");
	return 0;
}