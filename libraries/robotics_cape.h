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
*
* All example programs use these functions. See the bare_minimum example 
* for a skeleton outline.
*******************************************************************************/
int initialize_cape();
int cleanup_cape();		// call at the very end of main()


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
* cleanly when prompted by another thread.
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

enum state_t get_state();
int set_state(state_t new_state);


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
* int calibrate_dsm2()
*
* Starts a calibration routine. 
*
* see test_dsm2, calibrate_dsm2, and dsm2_passthroguh examples for use cases.
******************************************************************************/
int   initialize_dsm2();
int   is_new_dsm2_data();
int   set_new_dsm2_data_func(int (*func)(void));
int   get_dsm2_ch_raw(int channel);
float get_dsm2_ch_normalized(int channel);
int   ms_since_last_dsm2_packet();
int   get_dsm2_frame_resolution();
int   get_num_dsm2_channels();
int   stop_dsm2_service();
int   bind_dsm2();
int   calibrate_dsm2();


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
	ACCEL_DLPF_460,
	ACCEL_DLPF_184,
	ACCEL_DLPF_92,
	ACCEL_DLPF_41,
	ACCEL_DLPF_20,
	ACCEL_DLPF_10,
	ACCEL_DLPF_5
} accel_dlpf_t;

typedef enum gyro_dlpf_t {
	GYRO_DLPF_250,
	GYRO_DLPF_184,
	GYRO_DLPF_92,
	GYRO_DLPF_41,
	GYRO_DLPF_20,
	GYRO_DLPF_10,
	GYRO_DLPF_5
} gyro_dlpf_t;


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
	signed char orientation[9]; //orientation matrix
	int dmp_interrupt_priority; // scheduler priority for handler
} imu_config_t;

typedef struct imu_data_t {
	// last read sensor values in real units
	float accel[3]; // units of m/s^2
	float gyro[3];	// units of degrees/s
	float mag[3];	// units of uT
	float temp;		// units of degrees celcius
	
	// 16 bit raw adc readings from each sensor
	int16_t raw_gyro[3];	
	int16_t raw_accel[3];
	
	// FSR-derived ratios from raw to real units
	float accel_to_ms2; // to m/s^2
	float gyro_to_degs; // to degrees/s

	// quaternion_t DMPQuat; 	// unitless
	// quaternion_t fusedQuat; // unitless
	// vector3d_t fusedEuler;  // units of degrees

	// complementary filter constants used
	// by imu_read_dmp
	float lastDMPYaw;
	float lastYaw;
	
	// steady state offsets loaded from calibration file
	uint16_t rawGyroBias[3];
	uint16_t rawAccelBias[3];
	
	// magnetometer factory sensitivity adjustment values
	float mag_adjust[3];
} imu_data_t;
 

// one-shot sampling mode functions
imu_config_t get_default_imu_config();
int initialize_imu(imu_data_t *data, imu_config_t conf);
int read_accel_data(imu_data_t *data);
int read_gyro_data(imu_data_t *data);
int read_mag_data(imu_data_t *data);
int read_imu_temp(imu_data_t* data);

// // interrupt-driven sampling mode functions
// int initialize_imu_dmp(imu_data_t *data, imu_config_t conf);
// int set_imu_interrupt_func(int (*func)(void), imu_data_t* data);
// int stop_imu_interrupt_func();


// reset IMU, use before re-initializing with different settings



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



// /*******************************************************************************
// * SPI - Serial Peripheral Interface
// *
// * The Sitara's SPI1 bus is broken out on two JST SH 6-pin sockets
// * labeled SPI1.1 and SPI1.2 These share clock and serial IO signals.
// * However, each socket has its own slave select line allowing two
// * 
// *******************************************************************************/
// int initialize_spi1(uint8_t mode, uint32_t speed);
int select_spi1_slave(int slave);
int deselect_spi1_slave(int slave);	
// int spi1_send_bytes(uint8_t length, uint8_t* data);
// int spi1_send_byte(uint8_t data);
// unsigned char spi_read_reg(int fd, unsigned char reg_addr);
// unsigned char spi_write_reg(int fd, unsigned char reg_addr);


/*******************************************************************************
* UART
*******************************************************************************/
int initialize_uart(int bus, int speed);
int close_uart(int bus);
int get_uart_fd(int bus);
int flush_uart(int bus);
int uart_send_bytes(int bus, int bytes, char* data);
int uart_send_byte(int bus, char data);
int uart_read_bytes(int bus, int bytes, char* buf, int timeout_ms);


/*******************************************************************************
* @ int kill_robot()
*
* This function is used by initialize_cape to make sure any existing program
* using the robotics cape lib is stopped. The user doesn't need to integrate
* this in their own program as initialize_cape calls it. However, the user may
* call the kill_robot example program from the command line to close whatever
* program is running in the background.
* 
* return values:
* -2 : invalid contents in PID_FILE
* -1 : existing project failed to close cleanly and had to be killed
*  0 : No existing program is running
*  1 : An existing program was running but it shut down cleanly.
*******************************************************************************/
int kill_robot();

/*******************************************************************************
* General use Functions
*******************************************************************************/
int saturate_float(float* val, float min, float max);
int null_func();	// good for making interrupt handlers do nothing
char *byte_to_binary(unsigned char x); // for diagnostic prints
typedef struct timespec	timespec;
timespec diff(timespec start, timespec end); // subtract timespec structs 
uint64_t microsSinceEpoch();



#endif


