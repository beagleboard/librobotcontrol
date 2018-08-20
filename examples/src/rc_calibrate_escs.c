/**
 * @example    rc_calibrate_escs.c
 *
 * Typical brushless speed controllers (ESCs) accept a variety of pulse widths,
 * usually from 900-2100 microseconds. Before using them, you must calibrate the
 * ESC so it knows what pulse widths correspond to minimum and maximum throttle.
 * Typically ESCs go into calibration mode by applying any pulse width greater
 * than roughly 1000us when it is powered on. Once in calibration mode, the user
 * then applies max and min pulse widths briefly before turning off the signal
 * to exit calibration mode. This is typically done with the throttle stick on
 * your RC transmitter.
 *
 * The rc_calibrate_escs example assists you with this process by sending the
 * right pulse widths. Follow the instructions that are displayed in the console
 * when you execute the program.
 */

#include <stdio.h>
#include <rc/servo.h>
#include <rc/pthread.h>
#include <rc/time.h>

static int running;
static float width; // global variable for normalized pulse width to send


// background thread to send pulses at 50hz to ESCs
static void* __send_pulses(__attribute__ ((unused)) void *params)
{
	while(running){
		rc_servo_send_esc_pulse_normalized(0, width);
		rc_usleep(20000);
	}
	return 0;
}

int main()
{
	pthread_t send_pulse_thread;
	//char c;

	if(rc_servo_init()==-1) return -1;

	printf("\nDISCONNECT PROPELLERS FROM MOTORS\n");
	printf("DISCONNECT POWER FROM ESCS\n");
	printf("press enter to start sending max pulse width\n");
	getchar();

	//Send full throttle until the user hits enter
	width = 1;
	running = 1;
	if(rc_pthread_create(&send_pulse_thread, __send_pulses, (void*) NULL, SCHED_OTHER,0)==-1){
		return -1;
	}
	printf("\n");
	printf("Now apply power to the ESCs.\n");
	printf("Press enter again after the ESCs finish chirping\n");
	getchar();

	// now set lower bound
	printf("\n");
	printf("Sending minimum width pulses\n");
	printf("Press enter again after the ESCs chirp to finish calibration\n");
	width = 0;
	getchar();

	// cleanup and close
	printf("\nCalibration complete, check with rc_test_escs\n");

	running =0;
	rc_pthread_timed_join(send_pulse_thread, NULL, 1.0); // wait for it to stop
	rc_servo_cleanup();
	return 0;
}