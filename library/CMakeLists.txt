############################################################################
#
#   Copyright (c) 2018 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#notice, this list of conditions and the following disclaimer in
#the documentation and/or other materials provided with the
#distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#used to endorse or promote products derived from this software
#without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-error")

add_definitions(-DRC_AUTOPILOT_EXT)

include_directories(/usr/include)

if (DEFINED __RC_V0_3)

px4_add_library(robotics_cape
	roboticscape.c
	bmp280/rc_bmp280.c
	gpio/rc_buttons.c
	gpio/rc_gpio.c
	gpio/rc_gpio_setup.c
	math/rc_filter.c
	math/rc_linear_algebra.c
	math/rc_matrix.c
	math/rc_neon_functions.c
	math/rc_polynomial.c
	math/rc_quaternion.c
	math/rc_ring_buffer.c
	math/rc_vector.c
	mmap/rc_mmap_gpio_adc.c
	mmap/rc_mmap_pwmss.c
	mpu9250/rc_mpu9250.c
	other/rc_bb_model.c
	other/rc_cpu_freq.c
	other/rc_dsm.c
	other/rc_other.c
	other/rc_pinmux.c
	other/rc_pru.c
	other/rc_time.c
	pwm/rc_motors.c
	pwm/rc_pwm.c
	serial_ports/rc_i2c.c
	serial_ports/rc_spi.c
	serial_ports/rc_uart.c
	)

else()

include_directories(./include)

px4_add_library(robotics_cape
	src/button.c
	src/cpu.c
	src/dsm.c
	src/led.c
	src/mavlink_udp.c
	src/model.c
	src/motor.c
	src/pinmux.c
	src/pthread.c
	src/start_stop.c
	src/time.c
	src/version.c
	src/bmp/bmp.c
	src/io/adc.c
	src/io/encoder_eqep.c
	src/io/gpio.c
	src/io/i2c.c
	src/io/pwm.c
	src/io/spi.c
	src/io/uart.c
	src/math/algebra.c
	src/math/algebra_common.c
	src/math/filter.c
	src/math/matrix.c
	src/math/other.c
	src/math/polynomial.c
	src/math/quaternion.c
	src/math/ring_buffer.c
	src/math/vector.c
	src/mpu/mpu.c
	src/pru/encoder_pru.c
	src/pru/pru.c
	src/pru/servo.c
	)

endif()
