/*******************************************************************************
* test_servos.c
*
* Demonstrates use of pru to control servos and ESCs with pulses.
* This program operates in 4 different modes. See the option list below
* for how to select an operational mode from the command line.
*
* SERVO: uses send_servo_pulse_normalized() to set one or all servo positions
* to a value from -1.5 to 1.5 corresponding to their extended range. 
* -1 to 1 is considered the "safe" normal range as some servos will not go 
* beyond this. Test your servos incrementally to find their safe range.
*
* ESC: For unidirectional brushless motor speed controllers specify a range from
* 0 to 1 as opposed to the bidirectional servo range. Be sure to run the
* calibrate_esc example first to make sure the ESCs are calibrated to the right
* pulse range. This mode uses the send_esc_pulse_normalized() function.
*
* MICROSECONDS: You can also specify your own pulse width in microseconds (us).
* This uses the send_servo_pulse_us() function.
*
* SWEEP: This is intended to gently sweep a servo back and forth about the
* center position. Specify a range limit as a command line argument as described
* below. This also uses the send_servo_pulse_normalized() function.
* 
* 
* SERVO POWER RAIL: The robotics cape has a software-controlled 6V power
* regulator allowing controlled steady power to drive servos. This can be
* enabled at the command line with the -v option. It will not allow you to
* enable the power rail when using the ESC mode as sending 6V into an ESC
* may damage it. It is best to physically cut the center wire on ESC connections
* as the BEC function is not needed when using the Robotics Cape.
*******************************************************************************\

 Options
 -c {channel}   Specify one channel from 1-8.
                Otherwise all channels will be driven equally
 -f {hz}        Specify pulse frequency, otherwise 50hz is used
 -v             Enable 6V servo power rail
                DO NOT use power option with ESCs
 -p {position}  Drive servos to a position between -1.5 & 1.5
 -e {throttle}  Drive ESCs at normalized throttle from 0-1
 -u {width_us}  Send pulse width in microseconds (us)
 -s {limit}     Sweep servo back/forth between +- limit
                Limit can be between 0 & 1.5
 -h             Print this help messege

sample use to center servo channel 1:
   test_servos -v -c 1 -p 0.0
