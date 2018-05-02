############################################################################
#
#   Copyright (c) 2015 ECL Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name ECL nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
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

include_directories(include  /usr/include) 

px4_add_module(
	MODULE lib__robotics_cape
	COMPILE_FLAGS
		-DRC_AUTOPILOT_EXT
#		-DDEBUG
	SRCS
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
#		    src/io/gpio.c          # when cross-compile, missing <linux/gpio.h>
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
	DEPENDS
		platforms__common
        	git_robotics_cape
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
