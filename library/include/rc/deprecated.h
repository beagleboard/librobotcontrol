/**
 * <rc/deprecated.h>
 *
 * Deprecated functions that only exist for backwards compatability.
 *
 * @author     James Strawson
 * @date       4/26/2018
 *
 * @addtogroup Deprecated_Functions
 * @ingroup Deprecated
 * @{
 */

#ifndef RC_DEPRECATED_H
#define RC_DEPRECATED_H

#ifdef  __cplusplus
extern "C" {
#endif



int rc_initialize(void) __attribute__ ((deprecated));

int rc_cleanup(void) __attribute__ ((deprecated));

typedef enum rc_button_state_t {
	RELEASED,
	PRESSED
} rc_button_state_t;

int rc_set_pause_pressed_func(void (*func)(void)) __attribute__ ((deprecated));
int rc_set_pause_released_func(void (*func)(void)) __attribute__ ((deprecated));
int rc_set_mode_pressed_func(void (*func)(void)) __attribute__ ((deprecated));
int rc_set_mode_released_func(void (*func)(void)) __attribute__ ((deprecated));
rc_button_state_t rc_get_pause_button(void) __attribute__ ((deprecated));
rc_button_state_t rc_get_mode_button(void) __attribute__ ((deprecated));


int rc_get_encoder_pos(int ch) __attribute__ ((deprecated));
int rc_set_encoder_pos(int ch, int value) __attribute__ ((deprecated));

int rc_enable_motors(void) __attribute__ ((deprecated));
int rc_disable_motors(void) __attribute__ ((deprecated));
int rc_set_motor(int motor, float duty) __attribute__ ((deprecated));
int rc_set_motor_all(float duty) __attribute__ ((deprecated));
int rc_set_motor_free_spin(int motor) __attribute__ ((deprecated));
int rc_set_motor_free_spin_all(void) __attribute__ ((deprecated));
int rc_set_motor_brake(int motor) __attribute__ ((deprecated));
int rc_set_motor_brake_all(void) __attribute__ ((deprecated));




#ifdef __cplusplus
}
#endif

#endif // RC_DEPRECATED_H

/** @}  end group deprecated*/