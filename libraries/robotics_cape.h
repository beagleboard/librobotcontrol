/*******************************************************************************
* robotics_cape.h
*
* This contains the complete Robotics Cape API. All functions declared here can 
* be executed by linking to /usr/lib/robotics_cape.so
*
* All functions return 0 on success or -1 on failure unless otherwise stated.
*
* James Strawson - 2016
*******************************************************************************/

#ifndef ROBOTICS_CAPE
#define ROBOTICS_CAPE

#include <stdint.h> // for uint8_t types etc
typedef struct timespec	timespec;
typedef struct timeval timeval;

/*******************************************************************************
* INITIALIZATION AND CLEANUP
*
* @ int initialize_cape() 
*
* The user MUST call initialize_cape at beginning of your program. 
* In addition to setting up hardware, it also places a process id file
* in the file system at "/run/robotics_cape.pid" to indicate your program is
* using the robotics cape library and hardware. If initialize_cape() is called
* with an existing pid file in /run/ then it will try to shut down the existing 
* program before continuing with your own to avoid conflicts.
* 
* @ int cleanup_cape() 
*
* This removes the pid file and closes file pointers cleanly. 
* 
* * @ int kill_robot()
*
* This function is used by initialize_cape to make sure any existing program
* using the robotics cape lib is stopped. The user doesn't need to integrate
* this in their own program as initialize_cape calls it. However, the user may
* call the kill_robot example program from the command line to close whatever
* program is running in the background.
* return values:
* -2 : invalid contents in PID_FILE
* -1 : existing project failed to close cleanly and had to be killed
*  0 : No existing program is running
*  1 : An existing program was running but it shut down cleanly.
*
* All example programs use these functions. See the bare_minimum example 
* for a skeleton outline.
*******************************************************************************/
int initialize_cape(); 	// call at the beginning of main()
int cleanup_cape();		// call at the end of main()
int kill_robot();		// not usually necessary, use kill_robot example instead


/*******************************************************************************
* FLOW STATE FOR HIGH LEVEL PROGRAM CONTROL
*
* The user is encouraged to manage the initialization,running, and closing of 
* their threads with the globally accessible program flow state. This state
* can be set and accessed with get_state() and set_state() declared here.
* Calling initialize_cape() will set the state to UNINITIALIZED. When the
* user's own initialization sequence is complete they should set the flow
* state to RUNNING to indicate to other threads that the program should now
* behave in normal ongoing operational mode. Threads and loops should also
* independently check for an EXITING state to know when to close and exit
* cleanly when prompted by another thread. You may also call print_state()
* to print the textual name of the state to the screen.
*
* All example programs use these functions. See the bare_minimum example 
* for a skeleton outline.
*******************************************************************************/
typedef enum state_t {
	UNINITIALIZED,
	RUNNING,
	PAUSED,
	EXITING
} state_t;

state_t get_state();
int set_state(state_t new_state);
int print_state();


/*******************************************************************************
* LEDs
*
* Since the 4 blue USR LEDs on the Beaglebone are normally used by the OS,
* the Robotics Cape provides two LEDs for sole use by the user. One is red
* and one is green. The included examples use the red LED to indicate a paused
* or stopped state, and the green LED to indicate a running state. However
* they are not tied to any other robotics cape library functions and can be 
* used for whatever the user desires.
*
* @ typedef enum led_t
* 
* Two LEDs are available and defined as an enumerated type: RED or GREEN. 
*
* @ int set_led(led_t led, int state)
* 
* If state is 0 the LED will be turned off. If int state is non-zero then the 
* LED will be turned on. Returns 0 on success, -1 on failure.
*
* @ int get_led_state(led_t led)
*
* returns 1 if the LED is on or 0 if it is off.
* This function is typically used in multithreded applications where multiple
* threads may wish to use the same LED.
*
* @ int blink_led(led_t led, float hz, float period)
* 
* Flash an LED at a set frequency for a finite period of time.
* This is a blocking call and only returns after flashing.
*
* See the blink example for sample use case of all of these functions.
*******************************************************************************/
#define ON 	1
#define OFF	0
typedef enum led_t {
	GREEN,
	RED
} led_t;
int set_led(led_t led, int state);
int get_led_state(led_t led);
int blink_led(led_t led, float hz, float period);


/*******************************************************************************
* BUTTONS
*
* The Robotics Cape includes two buttons labeled PAUSE and MODE. Like the LEDs,
* they are not used by any background library functions and the user can assign
* them to any function they wish. However, the user is encouraged to use the
* pause button to toggle the program flow state between PAUSED and RUNNING
* using the previously described set_state(state_t new_state) function.
*
* @ typedef enum button_state_t
* 
* A button state can be either RELEASED or PRESSED as defined by this enum.
*
* @ int set_pause_pressed_func(int (*func)(void))
* @ int set_pause_released_func(int (*func)(void))
* @ int set_mode_pressed_func(int (*func)(void))
* @ int set_mode_released_func(int (*func)(void))
*
* initialize_cape() sets up interrupt handlers that run in the background to
* handle changes in button state in a way that uses minimal resources. The 
* user can assign which function should be called when either button is pressed
* or released. Functions can also be assigned under both conditions.
* for example, a timer could be started when a button is pressed and stopped
* when the button is released. Pass
*
* For simple tasks like pausing the robot, the user is encouraged to assign
* their function to be called when the button is released as this provides 
* a more natural user experience aligning with consumer product functionality.
* 
* The user can also just do a basic call to get_pause_button_state() or
* get_mode_buttom_state() which returns the enumerated type RELEASED or 
* PRESSED.
*
* See the blink example program for sample use case.
******************************************************************************/
typedef enum button_state_t {
	RELEASED,
	PRESSED
} button_state_t;
int set_pause_pressed_func(int (*func)(void));
int set_pause_released_func(int (*func)(void));
int set_mode_pressed_func(int (*func)(void));
int set_mode_released_func(int (*func)(void));
button_state_t get_pause_button();
button_state_t get_mode_button();


/******************************************************************************
* DC MOTOR CONTROL
*
* The robotics cape can drive 4 DC motors bidirectionally from a 2-cell lithium
* battery pack. The motors can not draw power from USB or a 12V charger as this
* would likely draw too much current. Each channel can support 1.2A continuous.
* 
* @ int enable_motors()
* @ int disable_motors()
*
* The motor drivers are initially in a low-power standby state and must be
* woken up with enable_motors() before use. The user can optionally put the 
* motor drivers back into low power state with disable_motors().
* 
* @ int set_motor(int motor, float duty)
* @ int set_motor_all(float duty)
*
* These will take in a motor index from 1 to 4 and a duty between -1 & 1
* corresponding to full power reverse to full power forward.
* set_motor_all() applies the same duty cycle to all 4 motor channels.
*
* @ int set_motor_free_spin(int motor)
* @ int set motor_free_spin_all()
*
* This puts one or all motor outputs in high-impedance state which lets the 
* motor spin freely as if it wasn't connected to anything.
*
* @ int set_motor_brake(int motor)
* @ int set_motor_brake_all()
*
* These will connect one or all motor terminal pairs together which
* makes the motor fight against its own back EMF turning it into a brake.
*
* See the test_motors example for sample use case.
******************************************************************************/
int enable_motors();
int disable_motors();
int set_motor(int motor, float duty);
int set_motor_all(float duty);
int set_motor_free_spin(int motor);
int set_motor_free_spin_all();
int set_motor_brake(int motor);
int set_motor_brake_all();


/******************************************************************************
* QUADRATURE ENCODER
*
* @ int get_encoder_pos(int ch)
* @ int set_encoder_pos(int ch, int value)
*
* The Robotics Cape includes 4 JST-SH sockets {E1 E2 E3 E4} for connecting
* quadrature encoders. The pins for each are assigned as follows:
*
* 1 - Ground
* 2 - 3.3V
* 3 - Signal A
* 4 - Signal B
*
* The first 3 channels are counted by the Sitara's eQEP hardware encoder
* counters. The 4th channel is counted by the PRU. As a result, no CPU cycles
* are wasted counting encoders and the user only needs to read the channel
* at any point in their code to get the current position. All channels are 
* reset to 0 when initialize_cape() is called. However, the user can reset
* the counter to zero or any other signed 32 bit value with set_encoder_pos().
*
* See the test_encoders example for sample use case.
******************************************************************************/
int get_encoder_pos(int ch);
int set_encoder_pos(int ch, int value);
 
 
/******************************************************************************
* ANALOG VOLTAGE SIGNALS
*
*
* @ float get_battery_voltage()
* @ float get_dc_jack_voltage()
* The Robotics cape includes two voltage dividers for safe measurement of the
* 2-cell lithium battery voltage and the voltage of any power source connected
* to the 6-16V DC power jack. These can be read with get_battery_voltage()
* and get_dc_jack_voltage()
* 
* @ int get_adc_raw(int ch)
* @ float get_adc_volt(int ch)
* 
* There is also a 6-pin JST-SH socket on the Cape for connecting up to 4
* potentiometers or general use analog signals. The pinout of this socket is
* as follows:
*
* 1 - Ground
* 2 - VDD_ADC (1.8V)
* 3 - AIN0
* 4 - AIN1
* 5 - AIN2
* 6 - AIN3
*
* All 7 ADC channels on the Sitara including the 4 listed above can be read
* with get_adc_raw(int ch) which returns the raw integer output of the 
* 12-bit ADC. get_adc_volt(int ch) additionally converts this raw value to 
* a voltage. ch must be from 0 to 6.
*
* See the test_adc example for sample use case.
******************************************************************************/
float get_battery_voltage();
float get_dc_jack_voltage();
int   get_adc_raw(int ch);
float get_adc_volt(int ch);


/******************************************************************************
* SERVO AND ESC 
*
* The Robotics Cape has 8 3-pin headers for connecting hobby servos and ESCs.
* The connectors are not polarized so pay close attention to the symbols
* printed in white silkscreen on the cape before plugging anything in.
* The standard pinnout for these 3 pin connectors is as follows.
*
* 1 - Ground
* 2 - 6V Power
* 3 - Pulse width signal
*
* @ int enable_servo_power_rail()
* @ int disable_servo_power_rail()
*
* The user must call enable_servo_power_rail() to enable the 6V voltage 
* regulator and send power to servos. This can be ignored if using ESCs or 
* servos that are driven by an external power source.
* disable_servo_power_rail() can be called to turn off power to the servos for
* example when the robot is in a paused state to save power or prevent noisy
* servos from buzzing.
*
* @ int send_servo_pulse_normalized(int ch, float input)
* @ int send_servo_pulse_normalized_all(float input)
*
* The normal operating range of hobby servos is usually +- 60 degrees of 
* rotation from the neutral position but they often work up to +- 90 degrees.
* send_servo_pulse_normalized(int ch, float input) will send a single pulse to
* the selected channel. the normalized input should be between -1.5 and 1.5
* corresponding to the following pulse width range and angle.
*
* input     width   angle  
* -1.5		600us	90 deg anticlockwise
* -1.0		900us	60 deg anticlockwise
*  0.0		1500us	0 deg neutral
*  1.0		2100us	60 deg clockwise
*  1.5		2400us	90 deg clockwise
*
* Note that all servos are different and do not necessarily allow the full
* range of motion past +-1.0. DO NOT STALL SERVOS.
*
* @ int send_esc_pulse_normalized(int ch, float input)
* @ int send_esc_pulse_normalized_all(float input)
*
* Brushless motor controllers (ESCs) for planes and multirotors are
* unidirectional and lend themselves better to a normalized range from 0 to 1.
* send_esc_pulse_normalized(int ch, float input) also sends a single pulse
* but the range is scaled as follows:
*
* input     width   power  
* 0.0		900us	0%   off
* 0.5		1500us	50%  half-throttle
* 1.0		2100us	100% full-throttle
*
* This assumes the ESCs have been calibrated for the 900-2100us range. Use the
* calibrate_escs example program to be sure.
*
* @ int send_servo_pulse_us(int ch, int us)
* @ int send_servo_pulse_us_all(int us)
*
* The user may also elect to manually specify the exact pulse width in
* in microseconds with send_servo_pulse_us(int ch, int us). When using any of
* these functions, be aware that they only send a single pulse to the servo
* or ESC. Servos and ESCs typically require an update frequency of at least 
* 10hz to prevent timing out. The timing accuracy of this loop is not critical
* and the user can choose to update at whatever frequency they wish.
*
* See the test_servos, sweep_servos, and calibrate_escs examples.
******************************************************************************/
int enable_servo_power_rail();
int disable_servo_power_rail();
int send_servo_pulse_normalized(int ch, float input);
int send_servo_pulse_normalized_all(float input);
int send_esc_pulse_normalized(int ch, float input);
int send_esc_pulse_normalized_all(float input);
int send_servo_pulse_us(int ch, int us);
int send_servo_pulse_us_all(int us);


/******************************************************************************
* DSM2/DSMX RC radio functions
*
* The Robotics Cape features a 3-pin JST ZH socket for connecting a DSM2/DSMX
* compatible satellite receiver. See the online manual for more details.
*
* @ int initialize_dsm2()
* Starts the background service.
*
* @ is_new_dsm2_data()
* 
* Returns 1 when new data is available. 
*
* @ int is_dsm2_active()
* 
* Returns 1 if packets are arriving in good health without timeouts.
* Returns 0 otherwise.
*
* @ int set_new_dsm2_data_func(int (*func)(void));
*
* Much like the button handlers, this assigns a user function to be called when
* new data arrives. Be careful as you should still check for radio disconnects 
*
* @ int get_dsm2_ch_raw(int channel) 
* 
* Returns the pulse width in microseconds commanded by the transmitter for a
* particular channel. The user can specify channels 1 through 9 but non-zero 
* values will only be returned for channels the transmitter is actually using. 
* The raw values in microseconds typically range from 900-2100us for a standard
* radio with default settings.
*
* @ get_dsm2_ch_normalized(int channel) 
*
* Returns a scaled value from -1 to 1 corresponding to the min and max values
* recorded during calibration. The user
* MUST run the clalibrate_dsm2 example to ensure the normalized values returned
* by this function are correct.
*
* @ int ms_since_last_dsm2_packet()
* 
* returns the number of milliseconds since the last dsm2 packet was received.
* if no packet has ever been received, return -1;
*
* @ int stop_dsm2_service()
*
* stops the background thread. Not necessary to be called by the user as
* cleanup_cape() calls this anyway.
*
* @ int bind_dsm2()
*
* Puts a satellite receiver in bind mode. Use the bind_dsm2 example program
* instead of calling this in your own program.
*
* int calibrate_dsm2_routine()
*
* Starts a calibration routine. 
*
* see test_dsm2, calibrate_dsm2, and dsm2_passthroguh examples for use cases.
******************************************************************************/
int   initialize_dsm2();
int   is_new_dsm2_data();
int   is_dsm2_active();
int   set_new_dsm2_data_func(int (*func)(void));
int   get_dsm2_ch_raw(int channel);
float get_dsm2_ch_normalized(int channel);
int   ms_since_last_dsm2_packet();
int   get_dsm2_frame_resolution();
int   get_num_dsm2_channels();
int   stop_dsm2_service();
int   bind_dsm2();
int   calibrate_dsm2_routine();


/******************************************************************************
* 9-AXIS IMU
*
* The Robotics Cape features an Invensense MPU9250 9-axis IMU. This API allows
* the user to configure this IMU in two modes: RANDOM and DMP
*
* RANDOM: The accelerometer, gyroscope, magnetometer, and thermometer can be
* read directly at any time. To use this mode, call initialize_imu() with your
* imu_config and imu_data structs as arguments as defined below. You can then
* call read_accel_data, read_gyro_data, read_mag_data, or read_imu_temp
* at any time to get the latest sensor values.
*
* DMP: Stands for Digital Motion Processor which is a feature of the MPU9250.
* in this mode, the DMP will sample the sensors internally and fill a FIFO
* buffer with the data at a fixed rate. Furthermore, the DMP will also calculate
* a filtered orientation quaternion which is placed in the same buffer. When
* new data is ready in the buffer, the IMU sends an interrupt to the BeagleBone
* triggering the buffer read followed by the execution of a function of your
* choosing set with the set_imu_interrupt_func() function.
*
* @ enum accel_fsr_t gyro_fsr_t
* 
* The user may choose from 4 full scale ranges of the accelerometer and
* gyroscope. They have units of gravity (G) and degrees per second (DPS)
* The defaults values are A_FSR_4G and G_FSR_1000DPS respectively.
*
* enum accel_dlpf_t gyro_dlpf_t 
*
* The user may choose from 7 digital low pass filter constants for the 
* accelerometer and gyroscope. The filter runs at 1kz and helps to reduce sensor
* noise when sampling more slowly. The default values are ACCEL_DLPF_184
* GYRO_DLPF_250. Lower cut-off frequencies incur phase-loss in measurements.
*
* @ struct imu_config_t
*
* Configuration struct passed to initialize_imu and initialize_imu_dmp. It is 
* best to get the default config with get_default_imu_config() function and
* modify from there.
*
* @ struct imu_data_t 
*
* This is the container for holding the sensor data from the IMU.
* The user is intended to make their own instance of this struct and pass
* its pointer to imu read functions.
*
* @ imu_config_t get_default_imu_config()
* 
* Returns an imu_config_t struct with default settings. Use this as a starting
* point and modify as you wish.
*
* @ int initialize_imu(imu_data_t *data, imu_config_t conf)
*
* Sets up the IMU in random-read mode. First create an instance of the imu_data
* struct to point to as initialize_imu will put useful data in it.
* initialize_imu only reads from the config struct. After this, you may read
* sensor data.
*
* @ int read_accel_data(imu_data_t *data)
* @ int read_gyro_data(imu_data_t *data)
* @ int read_mag_data(imu_data_t *data)
* @ int read_imu_temp(imu_data_t* data)
*
* These are the functions for random sensor sampling at any time. Note that
* if you wish to read the magnetometer then it must be enabled in the
* configuration struct. Since the magnetometer requires additional setup and
* is slower to read, it is disabled by default.
*
******************************************************************************/
typedef enum accel_fsr_t {
  A_FSR_2G,
  A_FSR_4G,
  A_FSR_8G,
  A_FSR_16G
} accel_fsr_t;

typedef enum gyro_fsr_t {
  G_FSR_250DPS,
  G_FSR_500DPS,
  G_FSR_1000DPS,
  G_FSR_2000DPS 
} gyro_fsr_t;

typedef enum accel_dlpf_t {
	ACCEL_DLPF_OFF,
	ACCEL_DLPF_184,
	ACCEL_DLPF_92,
	ACCEL_DLPF_41,
	ACCEL_DLPF_20,
	ACCEL_DLPF_10,
	ACCEL_DLPF_5
} accel_dlpf_t;

typedef enum gyro_dlpf_t {
	GYRO_DLPF_OFF,
	GYRO_DLPF_184,
	GYRO_DLPF_92,
	GYRO_DLPF_41,
	GYRO_DLPF_20,
	GYRO_DLPF_10,
	GYRO_DLPF_5
} gyro_dlpf_t;

typedef enum imu_orientation_t {
	ORIENTATION_Z_UP 	= 136,
	ORIENTATION_Z_DOWN 	= 396,
	ORIENTATION_X_UP 	= 14,
	ORIENTATION_X_DOWN 	= 266,
	ORIENTATION_Y_UP 	= 112,
	ORIENTATION_Y_DOWN 	= 336
} imu_orientation_t;

typedef struct imu_config_t {
	// full scale ranges for sensors
	accel_fsr_t accel_fsr; // AFS_2G, AFS_4G, AFS_8G, AFS_16G
	gyro_fsr_t gyro_fsr;  // GFS_250,GFS_500,GFS_1000,GFS_2000
	
	// internal low pass filter constants
	gyro_dlpf_t gyro_dlpf; // 
	accel_dlpf_t accel_dlpf;
	
	// magnetometer use is optional 
	int enable_magnetometer; // 0 or 1
	
	// DMP settings, only used with DMP interrupt
	int dmp_sample_rate;
	imu_orientation_t orientation; //orientation matrix
	// higher mix_factor means less weight the compass has on fused_TaitBryan
	int compass_time_constant; 	// time constant for filtering fused yaw
	int dmp_interrupt_priority; // scheduler priority for handler
	int show_warnings;	// set to 1 to enable showing of i2c_bus warnings

} imu_config_t;

typedef struct imu_data_t {
	// last read sensor values in real units
	float accel[3]; // units of m/s^2
	float gyro[3];	// units of degrees/s
	float mag[3];	// units of uT
	float temp;		// units of degrees Celsius
	
	// 16 bit raw adc readings from each sensor
	int16_t raw_gyro[3];	
	int16_t raw_accel[3];
	
	// FSR-derived conversion ratios from raw to real units
	float accel_to_ms2; // to m/s^2
	float gyro_to_degs; // to degrees/s
	
	// everything below this line is available in DMP mode only
	// quaternion and TaitBryan angles from DMP based on ONLY Accel/Gyro
	float dmp_quat[4]; 	// normalized quaternion
	float dmp_TaitBryan[3];	// radians pitch/roll/yaw X/Y/Z
	
	// If magnetometer is enabled in DMP mode, the following quaternion and 
	// TaitBryan angles will be available which add magnetometer data to filter
	float fused_quat[4]; 	// normalized quaternion
	float fused_TaitBryan[3]; 	// radians pitch/roll/yaw X/Y/Z
	float compass_heading;	// heading in radians based purely on magnetometer
} imu_data_t;
 
// General functions
imu_config_t get_default_imu_config();
int set_imu_config_to_defaults(imu_config_t *conf);
int calibrate_gyro_routine();
int calibrate_mag_routine();
int power_off_imu();

// one-shot sampling mode functions
int initialize_imu(imu_data_t *data, imu_config_t conf);
int read_accel_data(imu_data_t *data);
int read_gyro_data(imu_data_t *data);
int read_mag_data(imu_data_t *data);
int read_imu_temp(imu_data_t* data);

// interrupt-driven sampling mode functions
int initialize_imu_dmp(imu_data_t *data, imu_config_t conf);
int set_imu_interrupt_func(int (*func)(void));
int stop_imu_interrupt_func();
int was_last_read_successful();
uint64_t micros_since_last_interrupt();

/*******************************************************************************
* BMP280 Barometer
*
* The robotics cape features a Bosch BMP280 barometer for measuring temperature, 
* pressure, and altitude.
*
* @ enum bmp_oversample_t 
* Setting given to initialize_barometer() which defines the oversampling done
* internally to the barometer. For example, if BMP_OVERSAMPLE_16 is used then
* the barometer will average 16 samples before updating the data registers.
* The more oversampling used, the slower the data registers will update. You
* should pick an oversample that provides an update rate slightly slower than 
* the rate at which you will be reading the barometer. 
* 
* @ enum bmp_filter_t
* Setting given to initialize_barometer() to configure the coefficient of the 
* internal first order filter. We recommend disabling the filter with
* BMP_FILTER_OFF and doing your own filtering with the discrete filter functions
* below.
*
* @ int initialize_barometer(bmp_oversample_t oversample, bmp_filter_t filter)
* powers on the barometer and initializes it with the given oversample and
* filter settings. returns 0 on success, otherwise -1.
*
* @ int power_off_barometer()
* Puts the barometer into a low power state, should be called at the end of
* your program before close. return 0 on success, otherwise -1.
*
* @ int read_barometer()
* Reads the newest temperature and pressure measurments from the barometer over
* the I2C bus. To access the data use the bmp_get_temperature_c(), 
* bmp_get_pressure_pa(), or bmp_get_altitude_m() functions. 
* returns 0 on success, otherwise -1.
*
* @ float bmp_get_temperature_c()
* This does not start an I2C transaction but simply returns the temperature in
* degrees celcius that was read by the last call to the read_barometer() 
* function.
*
* @ float bmp_get_pressure_pa()
* This does not start an I2C transaction but simply returns the pressure in
* pascals that was read by the last call to the read_barometer() function.
* 
* @ float bmp_get_altitude_m()
* This does not start an I2C transaction but simply returns the altitude in 
* meters based on the pressure received by the last call to the read_barometer()
* function. Assuming current pressure at sea level is the default 101325 Pa.
* Use set_sea_level_pressure_pa() if you know the current sea level pressure
* and desire more accuracy. 
* 
* @ int set_sea_level_pressure_pa(float pa)
* If you know the current sea level pressure for your region and weather, you 
* can use this to correct the altititude reading. This is not necessary if you
* only care about differential altitude from a starting point.
*******************************************************************************/
typedef enum bmp_oversample_t{
	BMP_OVERSAMPLE_1  =	(0x01<<2), // update rate 182 HZ
	BMP_OVERSAMPLE_2  =	(0x02<<2), // update rate 133 HZ
	BMP_OVERSAMPLE_4  =	(0x03<<2), // update rate 87 HZ
	BMP_OVERSAMPLE_8  =	(0x04<<2), // update rate 51 HZ
	BMP_OVERSAMPLE_16 =	(0x05<<2)  // update rate 28 HZ
} bmp_oversample_t;

typedef enum bmp_filter_t{
	BMP_FILTER_OFF = (0x00<<2),
	BMP_FILTER_2   = (0x01<<2),
	BMP_FILTER_4   = (0x02<<2),
	BMP_FILTER_8   = (0x03<<2),
	BMP_FILTER_16  = (0x04<<2)
}bmp_filter_t;

int initialize_barometer(bmp_oversample_t oversample, bmp_filter_t filter);
int power_off_barometer();
int read_barometer();
float bmp_get_temperature_c();
float bmp_get_pressure_pa();
float bmp_get_altitude_m();
int set_sea_level_pressure_pa(float pa);


/*******************************************************************************
* GPS
*
*
*
*******************************************************************************/
int initialize_gps(int baud);
int stop_gps_service();


/*******************************************************************************
* I2C functions
*
* I2C bus 1 is broken out on the robotics cape on socket "I2C1" and is free for 
* the user to have full authority over. Bus 0 is used internally on the cape 
* for the IMU and barometer. The user should not use bus 0 unless they know what 
* they are doing. The IMU and barometer functions 
*
* @ int i2c_init(int bus, uint8_t devAddr)
* This initializes an I2C bus (0 or 1) at 400khz as defined in the device tree.
* The bus speed cannot be modified. devAddr is the 8-bit i2c address of the 
* device you wish to communicate with. This devAddr can be changed later 
* without initializing. i2c_init only needs to be called once per bus.
* 
* @ int set_device_address(int bus, uint8_t devAddr)
* Use this to change to another device address after initialization.
* 
* @ int i2c_close(int bus) 
* Closes the bus and device file descriptors.
*
* @int i2c_claim_bus(int bus)
* @int i2c_release_bus(int bus)
* @int i2c_get_in_use_state(int bus)
* Claim and release bus are purely for the convenience of the user and are not 
* necessary. They simply set a flag indicating that the bus is in use to help 
* manage multiple device access in multithreaded applications.
*
* @ int i2c_read_byte(int bus, uint8_t regAddr, uint8_t *data)
* @ int i2c_read_bytes(int bus, uint8_t regAddr, uint8_t length, uint8_t *data)
* @ int i2c_read_word(int bus, uint8_t regAddr, uint16_t *data)
* @ int i2c_read_words(int bus, uint8_t regAddr, uint8_t length, uint16_t *data)
* @ int i2c_read_bit(int bus, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
* These i2c_read functions are for reading data from a particular register.
* This sends the device address and register address to be read from before
* reading the response. 
*
* @ int i2c_write_byte(int bus, uint8_t regAddr, uint8_t data);
* @ int i2c_write_bytes(int bus, uint8_t regAddr, uint8_t length, uint8_t* data)
* @ int i2c_write_word(int bus, uint8_t regAddr, uint16_t data);
* @ int i2c_write_words(int bus,uint8_t regAddr, uint8_t length, uint16_t* data)
* @ int i2c_write_bit(int bus, uint8_t regAddr, uint8_t bitNum, uint8_t data)
* These write values write a value to a particular register on the previously
* selected device.
*
* @ int i2c_send_bytes(int bus, uint8_t length, uint8_t* data)
* @ int i2c_send_byte(int bus, uint8_t data)
* Instead of automatically sending a device address before the data which is 
* what happens in the above read and write functions, the i2c_send functions 
* send only the data given by the data argument. This is useful for more
* complicated IO such as uploading firmware to a device.
*******************************************************************************/
int i2c_init(int bus, uint8_t devAddr);
int i2c_close(int bus);
int i2c_set_device_address(int bus, uint8_t devAddr);
 
int i2c_claim_bus(int bus);
int i2c_release_bus(int bus);
int i2c_get_in_use_state(int bus);

int i2c_read_byte(int bus, uint8_t regAddr, uint8_t *data);
int i2c_read_bytes(int bus, uint8_t regAddr, uint8_t length,  uint8_t *data);
int i2c_read_word(int bus, uint8_t regAddr, uint16_t *data);
int i2c_read_words(int bus, uint8_t regAddr, uint8_t length, uint16_t *data);
int i2c_read_bit(int bus, uint8_t regAddr, uint8_t bitNum, uint8_t *data);

int i2c_write_byte(int bus, uint8_t regAddr, uint8_t data);
int i2c_write_bytes(int bus, uint8_t regAddr, uint8_t length, uint8_t* data);
int i2c_write_word(int bus, uint8_t regAddr, uint16_t data);
int i2c_write_words(int bus, uint8_t regAddr, uint8_t length, uint16_t* data);
int i2c_write_bit(int bus, uint8_t regAddr, uint8_t bitNum, uint8_t data);

int i2c_send_bytes(int bus, uint8_t length, uint8_t* data);
int i2c_send_byte(int bus, uint8_t data);



/*******************************************************************************
* SPI - Serial Peripheral Interface
*
* The Sitara's SPI1 bus is broken out on two JST SH 6-pin sockets
* labeled SPI1.1 and SPI1.2 These share clock and serial IO signals.
* However, each socket has its own slave select line allowing two
* 
*******************************************************************************/
int initialize_spi1(int mode, int speed_hz);
int get_spi1_fd();
int close_spi1();
int select_spi1_slave(int slave);
int deselect_spi1_slave(int slave);	
int spi1_send_bytes(char* data, int bytes);
int spi1_read_bytes(char* data, int bytes);
int spi1_write_reg_byte(char reg_addr, char data);
char spi1_read_reg_byte(char reg_addr);
int spi1_read_reg_bytes(char reg_addr, char* data, int bytes);
int spi1_transfer(char* tx_data, int tx_bytes, char* rx_data);


/*******************************************************************************
* UART
*******************************************************************************/
int initialize_uart(int bus, int speed, float timeout);
int close_uart(int bus);
int get_uart_fd(int bus);
int flush_uart(int bus);
int uart_send_bytes(int bus, int bytes, char* data);
int uart_send_byte(int bus, char data);
int uart_read_bytes(int bus, int bytes, char* buf);
int uart_read_line(int bus, int max_bytes, char* buf);


/*******************************************************************************
* CPU Frequency Control
*
* @ int set_cpu_frequency(cpu_frequency_t freq)
*
* Sets the CPU frequency to either a fixed value or to onedemand automatic
* scaling mode. Returns 0 on success, -1 on failure.
*
* @ cpu_frequency_t get_cpu_frequency()
*
* Returns the current clock speed of the Beaglebone's Sitara processor in the
* form of the provided enumerated type. It will never return the FREQ_ONDEMAND
* value as the intention of this function is to see the clock speed as set by
* either the user or the ondemand governor itself.
*
* @ int print_cpu_frequency()
*
* Prints the current frequency to the screen. For example "300MHZ".
* Returns 0 on success or -1 on failure.
*******************************************************************************/
typedef enum cpu_frequency_t{
	FREQ_ONDEMAND,
	FREQ_300MHZ,
	FREQ_600MHZ,
	FREQ_800MHZ,
	FREQ_1000MHZ
} cpu_frequency_t;

int set_cpu_frequency(cpu_frequency_t freq);
cpu_frequency_t get_cpu_frequency();
int print_cpu_frequency();

/*******************************************************************************
* Useful Functions
*
* This is a collection of miscellaneous useful functions that are part of the
* robotics cape library. These do not necessarily interact with hardware.
*
* @ int null_func()
*
* A simple function that returns 0. This exists so function pointers can be 
* set to do nothing such as button and imu interrupt handlers.
*
* @ float get_random_float()
*
* Returns a random floating point value between -1 and 1. This is here because
* the rand() function from stdlib.h only returns and integer. This is an
* optimized routine using bitwise operation instead of floating point division.
*
* @ saturate_float(float* val, float min, float max)
*
* Modifies val to be bounded between between min and max. Returns 1 if 
* saturation occurred, 0 if val was already in bound, and -1 if min was falsely
* larger than max.
*
* @ char *byte_to_binary(char x)
* 
* This returns a string (char*) of '1' and '0' representing a character.
* For example, print "00101010" with printf(byte_to_binary(42));
*
* @ timespec timespec_diff(timespec start, timespec end)
* 
* Returns the time difference between two timespec structs as another timespec.
* Convenient for use with nanosleep() function and accurately timed loops.
* Unlike timespec_sub defined in time.h, timespec_diff does not care which came 
* first, A or B. A positive difference in time is always returned.
*
* @ void timespec_add(timespec* start, float seconds);
*
* Adds a floating point number of seconds to a timespec struct. This saves
* yet more tedious manipulation of timespec structs.
*
* @ uint64_t timespec_to_micros(timespec ts)
* 
* Returns a number of microseconds corresponding to a timespec struct.
* Useful because timespecs are annoying.
* 
* @ uint64_t timeval_to_micros(timeval ts)
* 
* Returns a number of microseconds corresponding to a timespec struct.
* Useful because timespecs are annoying.
*
* @ uint64_t micros_since_epoch()
* 
* handy function for getting current time in microseconds
* so you don't have to deal with timespec structs
*
* @ int suppress_stdout(int (*func)(void))
*
* Executes a functiton func with all outputs to stdout suppressed. func must
* take no arguments and must return an integer. Adapt this source to your
* liking if you need to pass arguments. For example, if you have a function
* int foo(), call it with supressed output to stdout as follows:
* int ret = suppress_stdout(&foo);
*
* @ int suppress_stderr(int (*func)(void))
* 
* executes a functiton func with all outputs to stderr suppressed. func must
* take no arguments and must return an integer. Adapt this source to your
* liking if you need to pass arguments. For example, if you have a function
* int foo(), call it with supressed output to stderr as follows:
* int ret = suppress_stderr(&foo);
*
* @ continue_or_quit()
*
* This is a blocking function which returns 1 if the user presses ENTER.
* it returns 0 on any other keypress. If ctrl-C is pressed it will
* additionally set the global state to EXITITING and return -1. 
* This is a useful function for checking if the user wishes to continue with a 
* process or quit.
*******************************************************************************/
int null_func();
float get_random_float();
int saturate_float(float* val, float min, float max);
char *byte_to_binary(unsigned char x);
timespec timespec_diff(timespec A, timespec B);
void timespec_add(timespec* start, float seconds);
uint64_t timespec_to_micros(timespec ts);
uint64_t timeval_to_micros(timeval tv);
uint64_t micros_since_epoch();
int suppress_stdout(int (*func)(void));
int suppress_stderr(int (*func)(void));
int continue_or_quit();

/*******************************************************************************
* Vector and Quaternion Math
*
* These are useful for dealing with IMU orientation data and general vector math
*******************************************************************************/
// defines for index location within TaitBryan and quaternion arrays
#define TB_PITCH_X	0
#define TB_ROLL_Y	1
#define TB_YAW_Z	2
#define QUAT_W		0
#define QUAT_X		1
#define QUAT_Y		2
#define QUAT_Z		3

float quaternionNorm(float q[4]);
void normalizeQuaternion(float q[4]);
void quaternionToTaitBryan(float q[4], float v[3]);
void TaitBryanToQuaternion(float v[3], float q[4]);
void tilt_compensate(float in[4], float tilt[4], float out[4]);
void quaternionConjugate(float in[4], float out[4]);
void quaternionMultiply(float a[4], float b[4], float out[4]);
float vector3vector_dot_product(float a[3], float b[3]);
void vector3CrossProduct(float a[3], float b[3], float d[3]);

/*******************************************************************************
* Linear Algebra
*
*
*******************************************************************************/
typedef struct matrix_t{
	int rows;
	int cols;
	float** data;
	int initialized;
} matrix_t;

typedef struct vector_t{
	int len;
	float* data;
	int initialized;
} vector_t;

// Basic Matrix creation, modification, and access
matrix_t create_matrix(int rows, int cols);
void destroy_matrix(matrix_t* A);
matrix_t create_empty_matrix();
matrix_t duplicate_matrix(matrix_t A);
matrix_t create_square_matrix(int n);
matrix_t create_random_matrix(int rows, int cols);
matrix_t create_identity_matrix(int dim);
matrix_t create_diagonal_matrix(vector_t v);
matrix_t create_matrix_of_ones(int dim);
int set_matrix_entry(matrix_t* A, int row, int col, float val);
float get_matrix_entry(matrix_t A, int row, int col);
void print_matrix(matrix_t A);
void print_matrix_sci_notation(matrix_t A);

// Basic Vector creation, modification, and access
vector_t create_vector(int n);
void destroy_vector(vector_t* v);
vector_t create_empty_vector();
vector_t duplicate_vector(vector_t v);
vector_t create_random_vector(int len);
vector_t create_vector_of_ones(int len);
vector_t create_vector_from_array(int len, float* array);
int set_vector_entry(vector_t* v, int pos, float val);
float get_vector_entry(vector_t v, int pos);
void print_vector(vector_t v);
void print_vector_sci_notation(vector_t v);

// Multiplication, Addition, and other transforms
matrix_t multiply_matrices(matrix_t A, matrix_t Bm);
int matrix_times_scalar(matrix_t* A, float s);
int vector_times_scalar(vector_t* v, float s);
vector_t matrix_times_col_vec(matrix_t A, vector_t v);
vector_t row_vec_times_matrix(vector_t v, matrix_t A);
matrix_t add_matrices(matrix_t A, matrix_t B);
int transpose_matrix(matrix_t* A);

// vector operations
float vector_norm(vector_t v);
vector_t vector_projection(vector_t v, vector_t e);
matrix_t vector_outer_product(vector_t v1, vector_t v2);
float vector_dot_product(vector_t v1, vector_t v2);
vector_t cross_product_3d(vector_t v1, vector_t v2);
vector_t poly_conv(vector_t v1, vector_t v2);
vector_t poly_power(vector_t v, int N);
vector_t poly_butter(int N, float wc);
float standard_deviation(vector_t v);
float vector_mean(vector_t v);

// Advanced matrix operations
float matrix_determinant(matrix_t A);
int LUP_decomposition(matrix_t A, matrix_t* L, matrix_t* U, matrix_t* P);
int QR_decomposition(matrix_t A, matrix_t* Q, matrix_t* R);
matrix_t invert_matrix(matrix_t A);
matrix_t householder_matrix(vector_t v);

// linear system solvers
vector_t lin_system_solve(matrix_t A, vector_t b);
vector_t lin_system_solve_qr(matrix_t A, vector_t b);
int fit_ellipsoid(matrix_t points, vector_t* center, vector_t* lengths);


/*******************************************************************************
* Ring Buffer
*
* Ring buffers are FIFO (first in first out) buffers of fixed length which
* efficiently boot out the oldest value when full. They are particularly well
* suited for storing the last n values in a discrete time filter.
*
* The user creates their own instance of a buffer and passes a pointer to the
* these ring_buf functions to perform normal operations. 

* @ int reset_ring_buf(ring_buf* buf)
*
* sets all values in the buffer to 0 and sets the buffer position back to 0
*
* @ int insert_new_ring_buf_value(ring_buf* buf, float val)
* 
* Puts a new float into the ring buffer. If the buffer was full then the oldest
* value in the buffer is automatically removed.
*
* @ float get_ring_buf_value(ring_buf* buf, int position)
*
* returns the float which is 'position' steps behind the last value placed in
* the buffer. If 'position' is given as 0 then the most recent value is
* returned. 'Position' obviously can't be larger than buffer_size minus 1
*******************************************************************************/

typedef struct ring_buf_t{
	//float data[RING_BUF_SIZE];
	float* data;
	int size;
	int index;
	int initialized;
} ring_buf_t;

ring_buf_t create_ring_buf(int size);
int reset_ring_buf(ring_buf_t* buf);
int destroy_ring_buf(ring_buf_t* buf);
int insert_new_ring_buf_value(ring_buf_t* buf, float val);
float get_ring_buf_value(ring_buf_t* buf, int position);


/*******************************************************************************
* Discrete SISO Filters
*
* This is a collection of functions for generating and implementing discrete 
* SISO filters for arbitrary transfer functions. 
*
* @ struct d_filter_t
*
* This is the heart of the library. For each implemented filter the user must
* create a single instance of a discrete_filter struct. Each instance contains
* transfer function constants and memory about previous inputs and outputs.
* You may read values directly from your own instance of the d_filter_t struct.
* To modify the contents of the filter please use the functions provided here.
*
* @ d_filter_t create_filter(int order,float dt,float num[],float den[])
*
* Allocate memory for a filter of specified order & fill with transfer
* function constants. Use enable_saturation immediately after this if you want
* to enable automatic saturation.
*
* @ float march_filter(d_filter_t* filter, float new_input)
*
* March the filter forward in time one step with new input data.
* Returns new output which could also be accessed with filter.current_output
* If saturation is enabled then the output will automatically be bound by the
* min and max values given to enable_saturation. The enable_saturation entry
* in the filter struct will also be set to 1 if saturation occurred. 
*
* @ int reset_filter(d_filter_t* filter)
*
* resets all inputs and outputs to 0
*
* @ int enable_saturation(d_filter_t* filter, float sat_min, float sat_max)
*
* If saturation is enabled for a specified filter, the filter will automatically
* bound the output between min and max. You may ignore this function if you wish
* the filter to run unbounded.
*
* @ int did_filter_t_saturate(d_filter_t* filter)
*
* Returns 1 if the filter saturated the last time step. Returns 0 otherwise.
*
* @ float previous_filter_input(d_filter_t* filter, int steps)
*
* Returns the input 'steps' back in time. Steps = 0 returns most recent input.
*
* @ float previous_filter_output(d_filter_t* filter, int steps)
*
* Returns the output 'steps' back in time. Steps = 0 returns most recent output.
*
* @ float newest_filter_output(d_filter_t* filter)
*
* Returns the most recent output from the filter. Alternatively the user could
* access the value from their d_filter_t_t struct with filter.newest_output
*
* @ float newest_filter_input(d_filter_t* filter)
*
* Returns the most recent input to the filter. Alternatively the user could
* access the value from their d_filter_t_t struct with filter.newest_input
*
* @ d_filter_t create_first_order_low_pass(float dt, float time_constant)
*
* Returns a configured and ready to use d_filter_t_t struct with a first order
* low pass transfer function. dt is in units of seconds and time_constant is 
* the number of seconds it takes to rise to 63.4% of a steady-state input.
*
* @ d_filter_t create_first_order_high_pass(float dt, float time_constant)
*
* Returns a configured and ready to use d_filter_t_t struct with a first order
* high pass transfer function. dt is in units of seconds and time_constant is 
* the number of seconds it takes to decay by 63.4% of a steady-state input.
*
* @ d_filter_t create_integrator(float dt)
*
* Returns a configured and ready to use d_filter_t_t struct with the transfer
* function for a first order time integral.
*
* @ d_filter_t create_pid(float kp, float ki, float kd, float Tf, float dt)
*
* discrete-time implementation of a parallel PID controller with rolloff.
* This is equivalent to the Matlab function: C = pid(Kp,Ki,Kd,Tf,Ts)
*
* We cannot implement a pure differentiator with a discrete transfer function
* so this filter has high frequency rolloff with time constant Tf. Smaller Tf
* results in less rolloff, but Tf must be greater than dt/2 for stability.
*
* @ int print_filter_details(d_filter_t* filter)
*
* Prints the order, numerator, and denominator coefficients for debugging.
*******************************************************************************/

typedef struct d_filter_t{
	// transfer function properties
	int order;				// transfer function order
	float dt;				// timestep in seconds
	float gain; 			// gain usually 1
	vector_t numerator;		// numerator coefficients 
	vector_t denominator;	// denominator coefficients 
	// saturation settings
	int saturation_en;		// set to 1 by enable_saturation()
	float saturation_min;
	float saturation_max;
	int saturation_flag;	// 1 if saturated on the last step
	// soft start settings
	int soft_start_en;		// set to 1 by enbale_soft_start()
	float soft_start_steps;	// steps before full output allowed
	// dynamically allocated ring buffers
	ring_buf_t in_buf;
	ring_buf_t out_buf;
	// newest input and output for quick reference
	float newest_input;
	float newest_output;
	// other
	uint64_t step;			// steps since last reset
	int initialized;		// 
} d_filter_t;

d_filter_t create_filter(int order, float dt, float* num, float* den);
int destroy_filter(d_filter_t* filter);
d_filter_t create_empty_filter();
float march_filter(d_filter_t* filter, float new_input);
int reset_filter(d_filter_t* filter);
int enable_saturation(d_filter_t* filter, float min, float max);
int did_filter_saturate(d_filter_t* filter);
int enable_soft_start(d_filter_t* filter, float seconds);
float previous_filter_input(d_filter_t* filter, int steps);
float previous_filter_output(d_filter_t* filter, int steps);
float newest_filter_output(d_filter_t* filter);
float newest_filter_input(d_filter_t* filter);
int prefill_filter_inputs(d_filter_t* filter, float in);
int prefill_filter_outputs(d_filter_t* filter, float out);
int print_filter_details(d_filter_t* filter);
d_filter_t multiply_filters(d_filter_t f1, d_filter_t f2);
d_filter_t C2DTustin(vector_t num, vector_t den, float dt, float w);
d_filter_t create_first_order_lowpass(float dt, float time_constant);
d_filter_t create_first_order_highpass(float dt, float time_constant);
d_filter_t create_butterworth_lowpass(int order, float dt, float wc);
d_filter_t create_butterworth_highpass(int order, float dt, float wc);
d_filter_t create_integrator(float dt);
d_filter_t create_double_integrator(float dt);
d_filter_t create_pid(float kp, float ki, float kd, float Tf, float dt);


	
#endif //ROBOTICS_CAPE


