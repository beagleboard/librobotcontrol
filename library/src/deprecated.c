/**
 * @file deprecated.c
 */

#include <stdio.h>
#include <rc/start_stop.h>
#include <rc/deprecated.h>
#include <rc/button.h>
#include <rc/motor.h>
#include <rc/encoder_eqep.h>


int rc_initialize(void)
{
	// initialize pause and mode buttons
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR in rc_initialize failed to init buttons\n");
		return -1;
	}
	if(rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR in rc_initialize failed to init buttons\n");
		return -1;
	}
	if(rc_encoder_eqep_init()){
		fprintf(stderr,"ERROR: failed to run rc_encoder_eqep_init\n");
		return -1;
	}
	if(rc_motor_init()){
		fprintf(stderr,"ERROR: failed to initialize motors\n");
		return -1;
	}
	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}
	return 0;
}

int rc_cleanup(void)
{
	rc_button_cleanup();
	rc_encoder_eqep_cleanup();
	rc_motor_cleanup();
	return 0;
}


void (*pause_pressed_func)(void)	= NULL;
void (*pause_released_func)(void)	= NULL;
void (*mode_pressed_func)(void)		= NULL;
void (*mode_released_func)(void)	= NULL;


int rc_set_pause_pressed_func(void (*func)(void)){
	if(func==NULL){
		printf("ERROR: trying to assign NULL pointer to paused_pressed_func\n");
		return -1;
	}
	pause_pressed_func = func;
	return rc_button_set_callbacks(RC_BTN_PIN_PAUSE, pause_pressed_func, pause_released_func);
}
int rc_set_pause_released_func(void (*func)(void)){
	if(func==NULL){
		printf("ERROR: trying to assign NULL pointer to paused_released_func\n");
		return -1;
	}
	pause_released_func = func;
	return rc_button_set_callbacks(RC_BTN_PIN_PAUSE, pause_pressed_func, pause_released_func);

}
int rc_set_mode_pressed_func(void (*func)(void)){
	if(func==NULL){
		printf("ERROR: trying to assign NULL pointer to mode_pressed_func\n");
		return -1;
	}
	mode_pressed_func = func;
	return rc_button_set_callbacks(RC_BTN_PIN_MODE, mode_pressed_func, mode_released_func);
}
int rc_set_mode_released_func(void (*func)(void)){
	if(func==NULL){
		printf("ERROR: trying to assign NULL pointer to mode_released_func\n");
		return -1;
	}
	mode_released_func = func;
	return rc_button_set_callbacks(RC_BTN_PIN_MODE, mode_pressed_func, mode_released_func);
}

rc_button_state_t rc_get_pause_button(void)
{
	int ret = rc_button_get_state(RC_BTN_PIN_PAUSE);
	if(ret == RC_BTN_STATE_PRESSED) return PRESSED;
	if(ret == RC_BTN_STATE_RELEASED) return RELEASED;
	return -1;
}

rc_button_state_t rc_get_mode_button(void)
{
	int ret = rc_button_get_state(RC_BTN_PIN_MODE);
	if(ret == RC_BTN_STATE_PRESSED) return PRESSED;
	if(ret == RC_BTN_STATE_RELEASED) return RELEASED;
	return -1;
}


int rc_get_encoder_pos(int ch)
{
	return rc_encoder_eqep_read(ch);
}

int rc_set_encoder_pos(int ch, int value)
{
	return rc_encoder_eqep_write(ch, value);
}



int rc_enable_motors(void)
{
	return rc_motor_standby(0);
}

int rc_disable_motors(void)
{
	return rc_motor_standby(1);
}

int rc_set_motor(int motor, float duty)
{
	return rc_motor_set(motor,duty);
}

int rc_set_motor_all(float duty)
{
	return rc_motor_set(0,duty);
}

int rc_set_motor_free_spin(int motor)
{
	return rc_motor_free_spin(motor);
}

int rc_set_motor_free_spin_all(void)
{
	return rc_motor_free_spin(0);

}

int rc_set_motor_brake(int motor)
{
	return rc_motor_brake(motor);

}

int rc_set_motor_brake_all(void)
{
	return rc_motor_brake(0);
}