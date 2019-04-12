
add_definitions(-DRC_AUTOPILOT_EXT)

include_directories(/usr/include)

add_compile_options(
	-Wno-cast-align
	-Wno-logical-op
	-Wno-strict-prototypes
)

if(DEFINED __RC_V0_3)

	add_library(robotics_cape
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

	target_include_directories(robotics_cape PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

else()

	include_directories(include)

	add_library(robotics_cape
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

	target_include_directories(robotics_cape PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

endif()
