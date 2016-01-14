// C library for interfacing with the TI PWM userspace driver
// This relies on a device tree overlay enabling hrpwm 


int simple_init_pwm(int subsystem, int frequency);
int simple_uninit_pwm(int subsystem);
int simple_set_pwm_duty(int subsystem, char ch, float duty);