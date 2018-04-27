/**
 * <rc/deprecated.h>
 *
 * @brief Deprecated functions that only exist for backwards compatability.
 *
 * @author     James Strawson
 * @date       4/26/2018
 *
 * @addtogroup deprecated
 * @{
 */

#ifndef RC_DEPRECATED_H
#define RC_DEPRECATED_H

#ifdef  __cplusplus
extern "C" {
#endif


/*
int rc_initialize() __attribute__ ((deprecated));

int rc_cleanup() __attribute__ ((deprecated));

typedef enum rc_button_state_t {
	RELEASED,
	PRESSED
} rc_button_state_t;
int rc_set_pause_pressed_func(void (*func)(void));
int rc_set_pause_released_func(void (*func)(void));
int rc_set_mode_pressed_func(void (*func)(void));
int rc_set_mode_released_func(void (*func)(void));
rc_button_state_t rc_get_pause_button();
rc_button_state_t rc_get_mode_button();

typedef enum rc_led_t {
	GREEN,
	RED
} rc_led_t;
int rc_set_led(rc_led_t led, int state);
int rc_get_led(rc_led_t led);

int rc_get_encoder_pos(int ch);
int rc_set_encoder_pos(int ch, int value);

int rc_enable_motors();
int rc_disable_motors();
int rc_set_motor(int motor, float duty);
int rc_set_motor_all(float duty);
int rc_set_motor_free_spin(int motor);
int rc_set_motor_free_spin_all();
int rc_set_motor_brake(int motor);
int rc_set_motor_brake_all();

*/






#ifdef __cplusplus
}
#endif

#endif // RC_DEPRECATED_H
/** @}  end group deprecated*/