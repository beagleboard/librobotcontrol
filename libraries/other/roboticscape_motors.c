/*******************************************************************************
* roboticscape_buttons.c
*
* 4 threads for managing the pause and press buttons
*******************************************************************************/
#include <stdio.h>
#include "../roboticscape.h"
#include "../roboticscape-defs.h"

// global variables
int mdir1a, mdir2b; // variable gpio pin assignments
int motors_initialized = 0;


/*******************************************************************************
* int initialize_motors()
* 
* set up gpio assignments, pwm channels, and make sure motors are left off.
* GPIO exporting must be done previously with simple_init_gpio()
*******************************************************************************/
int initialize_motors(){
	// assign gpio pins for blue/black
	if(get_bb_model()==BB_BLUE){
		mdir1a = MDIR1A_BLUE;
		mdir2b = MDIR2B_BLUE;
	}
	else{
		mdir1a = MDIR1A;
		mdir2b = MDIR2B;
	}

	#ifdef DEBUG
	printf("Initializing: PWM\n");
	#endif
	if(simple_init_pwm(1,PWM_FREQ)){
		printf("ERROR: failed to initialize hrpwm1\n");
		return -1;
	}
	if(simple_init_pwm(2,PWM_FREQ)){
		printf("ERROR: failed to initialize PWMSS 2\n");
		return -1;
	}

	motors_initialized = 1;
	disable_motors();
	return 0;
}


/*******************************************************************************
* enable_motors()
* 
* turns on the standby pin to enable the h-bridge ICs
* returns 0 on success
*******************************************************************************/
int enable_motors(){
	if(motors_initialized==0){
		printf("ERROR: trying to enable motors before they have been initialized\n");
		return -1;
	}
	set_motor_free_spin_all();
	return mmap_gpio_write(MOT_STBY, HIGH);
}

/*******************************************************************************
* int disable_motors()
* 
* turns off the standby pin to disable the h-bridge ICs
* and disables PWM output signals, returns 0 on success
*******************************************************************************/
int disable_motors(){
	if(motors_initialized==0){
		printf("ERROR: trying to disable motors before they have been initialized\n");
		return -1;
	}
	mmap_gpio_write(MOT_STBY, LOW);
	set_motor_free_spin_all();
	return 0;
}

/*******************************************************************************
* int set_motor(int motor, double duty)
* 
* set a motor direction and power
* motor is from 1 to 4, duty is from -1.0 to +1.0
*******************************************************************************/
int set_motor(int motor, double duty){
	uint8_t a,b;
	if(motors_initialized==0){
		printf("ERROR: trying to set_motor before they have been initialized\n");
		return -1;
	}
	//check that the duty cycle is within +-1
	if (duty>1.0){
		duty = 1.0;
	}
	else if(duty<-1.0){
		duty=-1.0;
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
	
	// set gpio direction outputs & duty
	switch(motor){
		case 1:
			mmap_gpio_write(mdir1a, a);
			mmap_gpio_write(MDIR1B, b);
			mmap_set_pwm_duty(1, 'A', duty);
			break;
		case 2:
			mmap_gpio_write(MDIR2A, b);
			mmap_gpio_write(mdir2b, a);
			mmap_set_pwm_duty(1, 'B', duty);
			break;
		case 3:
			mmap_gpio_write(MDIR3A, b);
			mmap_gpio_write(MDIR3B, a);
			mmap_set_pwm_duty(2, 'A', duty);
			break;
		case 4:
			mmap_gpio_write(MDIR4A, a);
			mmap_gpio_write(MDIR4B, b);
			mmap_set_pwm_duty(2, 'B', duty);
			break;
		default:
			printf("enter a motor value between 1 and 4\n");
			return -1;
	}
	return 0;
}

/*******************************************************************************
* int set_motor_all(double duty)
* 
* applies the same duty cycle argument to all 4 motors
*******************************************************************************/
int set_motor_all(double duty){
	int i;
	if(motors_initialized==0){
		printf("ERROR: trying to set_motor_all before they have been initialized\n");
		return -1;
	}
	for(i=1;i<=MOTOR_CHANNELS; i++){
		set_motor(i, duty);
	}
	return 0;
}

/*******************************************************************************
* int set_motor_free_spin(int motor)
* 
* This puts one or all motor outputs in high-impedance state which lets the 
* motor spin freely as if it wasn't connected to anything.
*******************************************************************************/
int set_motor_free_spin(int motor){
	if(motors_initialized==0){
		printf("ERROR: trying to set_motor_free_spin before they have been initialized\n");
		return -1;
	}
	// set gpio direction outputs & duty
	switch(motor){
		case 1:
			mmap_gpio_write(mdir1a, 0);
			mmap_gpio_write(MDIR1B, 0);
			mmap_set_pwm_duty(1, 'A', 0.0);
			break;
		case 2:
			mmap_gpio_write(MDIR2A, 0);
			mmap_gpio_write(mdir2b, 0);
			mmap_set_pwm_duty(1, 'B', 0.0);
			break;
		case 3:
			mmap_gpio_write(MDIR3A, 0);
			mmap_gpio_write(MDIR3B, 0);
			mmap_set_pwm_duty(2, 'A', 0.0);
			break;
		case 4:
			mmap_gpio_write(MDIR4A, 0);
			mmap_gpio_write(MDIR4B, 0);
			mmap_set_pwm_duty(2, 'B', 0.0);
			break;
		default:
			printf("enter a motor value between 1 and 4\n");
			return -1;
	}
	return 0;
}

/*******************************************************************************
* @ int set_motor_free_spin_all()
*******************************************************************************/
int set_motor_free_spin_all(){
	int i;
	if(motors_initialized==0){
		printf("ERROR: trying to set_motor_free_spin_all before they have been initialized\n");
		return -1;
	}
	for(i=1;i<=MOTOR_CHANNELS; i++){
		set_motor_free_spin(i);
	}
	return 0;
}

/*******************************************************************************
* int set_motor_brake(int motor)
* 
* These will connect one or all motor terminal pairs together which
* makes the motor fight against its own back EMF turning it into a brake.
*******************************************************************************/
int set_motor_brake(int motor){
	if(motors_initialized==0){
		printf("ERROR: trying to set_motor_brake before they have been initialized\n");
		return -1;
	}
	// set gpio direction outputs & duty
	switch(motor){
		case 1:
			mmap_gpio_write(mdir1a, 1);
			mmap_gpio_write(MDIR1B, 1);
			mmap_set_pwm_duty(1, 'A', 0.0);
			break;
		case 2:
			mmap_gpio_write(MDIR2A, 1);
			mmap_gpio_write(mdir2b, 1);
			mmap_set_pwm_duty(1, 'B', 0.0);
			break;
		case 3:
			mmap_gpio_write(MDIR3A, 1);
			mmap_gpio_write(MDIR3B, 1);
			mmap_set_pwm_duty(2, 'A', 0.0);
			break;
		case 4:
			mmap_gpio_write(MDIR4A, 1);
			mmap_gpio_write(MDIR4B, 1);
			mmap_set_pwm_duty(2, 'B', 0.0);
			break;
		default:
			printf("enter a motor value between 1 and 4\n");
			return -1;
	}
	return 0;
}

/*******************************************************************************
* @ int set_motor_brake_all()
*******************************************************************************/
int set_motor_brake_all(){
	int i;
	if(motors_initialized==0){
		printf("ERROR: trying to set_motor_brake_all before they have been initialized\n");
		return -1;
	}
	for(i=1;i<=MOTOR_CHANNELS; i++){
		set_motor_brake(i);
	}
	return 0;
}
