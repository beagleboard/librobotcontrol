/**
 * <robotcontrol.h>
 *
 * @brief Single header that includes all modules
 *
 * This includes every single module in the Robot Control API so that users do
 * not need to include each module individually.
 *
 * Use these modules by linking to /usr/lib/librobotcontrol.so
 *
 * @author     James Strawson
 * @date       3/7/2018
 * @addtogroup Robot_Control_All
 * @{
 *
 * @include "robotcontrol.h"
 */

#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <rc/adc.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/cpu.h>
#include <rc/deprecated.h>
#ifdef RC_AUTOPILOT_EXT
#include "rc/dsm.h"
#else
#include <rc/dsm.h>
#endif
#include <rc/encoder_eqep.h>
#include <rc/encoder_pru.h>
#include <rc/encoder.h>
#include <rc/gpio.h>
#include <rc/i2c.h>
#include <rc/led.h>
#include <rc/math.h>
#include <rc/mavlink_udp.h>
#include <rc/mavlink_udp_helpers.h>
#include <rc/model.h>
#include <rc/motor.h>
#include <rc/mpu.h>
#include <rc/pinmux.h>
#include <rc/pru.h>
#include <rc/pthread.h>
#include <rc/pwm.h>
#include <rc/servo.h>
#include <rc/spi.h>
#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc/uart.h>
#include <rc/version.h>

#endif // ROBOTCONTROL_H

/** @} end group Robot_Control_All*/




