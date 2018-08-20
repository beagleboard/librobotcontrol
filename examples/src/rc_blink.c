/**
 * @file rc_blink.c
 * @example    rc_blink
 *
 * This is an example program to demonstrate use of LEDs and button handlers in
 * the Robot Control API. Once started, blink will flash the green and red LEDs.
 * Pressing the mode button will cycle through 3 blinking speeds, slow medium,
 * and fast. Momentarily pressing the pause button will stop and start the
 * blinking by toggling the global state between PAUSED and RUNNING. If the user
 * holds the pause button for more than 1.5 seconds then the blink program will
 * flash the red LED and exit cleanly.
 *
 * This should be used as a reference for how to handle buttons and how to
 * control program flow cleanly utilizing rc_get_state() and rc_set_state().
 **/

#include <stdio.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/start_stop.h>
#include <rc/time.h>

#define QUIT_TIMEOUT_US 1500000 // quit after 1.5 seconds holding pause button
#define QUIT_CHECK_US	100000	// check every 1/10 second

// mode=0,1,2 corresponds to us_delay index for slow,medium,fast
static const int us_delay[] = {400000, 170000, 100000};
static int mode;
static int toggle; // toggles between 0&1 for led blink


/**
 * Make the Pause button toggle between paused and running states.
 */
static void __on_pause_release(void)
{
	// toggle betewen paused and running modes
	if(rc_get_state()==RUNNING){
		rc_set_state(PAUSED);
		printf("PAUSED\n");
	}
	else if(rc_get_state()==PAUSED){
		rc_set_state(RUNNING);
		printf("RUNNING\n");
	}
	fflush(stdout);
	return;
}

/**
* If the user holds the pause button for 2 seconds, set state to exiting which
* triggers the rest of the program to exit cleanly.
*/
static void __on_pause_press(void)
{
	int i=0;
	const int samples = QUIT_TIMEOUT_US/QUIT_CHECK_US;

	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(QUIT_CHECK_US);
		if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED){
			return;
		}
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}

/**
 * If the user holds the pause button for 2 seconds, set state to exiting which
 * triggers the rest of the program to exit cleanly.
 */
static void __on_mode_release(void)
{
	if(mode<2)	mode++;
	else		mode=0;
	printf("setting mode: %d\n", mode);
	return;
}

/**
 * main function sits in one while loop blinking LEDs while button handlers
 * control the blink speed and program state
 */
int main()
{
	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()<0){
		fprintf(stderr,"ERROR: failed to complete rc_enable_signal_handler\n");
		return -1;
	}

	// initialize pause and mode buttons
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to init buttons\n");
		return -1;
	}
	if(rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to init buttons\n");
		return -1;
	}

	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE, __on_pause_press, __on_pause_release);
	rc_button_set_callbacks(RC_BTN_PIN_MODE, NULL, __on_mode_release);

	// start with both LEDs off
	if(rc_led_set(RC_LED_GREEN, 0)==-1){
		fprintf(stderr, "ERROR in rc_blink, failed to set RC_LED_GREEN\n");
		return -1;
	}
	if(rc_led_set(RC_LED_RED, 0)==-1){
		fprintf(stderr, "ERROR in rc_blink, failed to set RC_LED_RED\n");
		return -1;
	}

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// prepare to run
	rc_set_state(RUNNING);
	mode = 0;
	printf("\nPress mode to change blink rate\n");
	printf("hold pause button to exit\n");

	// Run the main loop untill state is EXITING which is set by hitting ctrl-c
	// or holding down the pause button for more than the quit timeout period
	while(rc_get_state()!=EXITING){
		// if the state is RUNNING (instead of PAUSED) then blink!
		if(rc_get_state()==RUNNING){
			if(toggle){
				rc_led_set(RC_LED_GREEN,0);
				rc_led_set(RC_LED_RED,1);
				toggle = 0;
			}
			else{
				rc_led_set(RC_LED_GREEN,1);
				rc_led_set(RC_LED_RED,0);
				toggle=1;
			}
		}
		// sleep the right delay based on current mode.
		rc_usleep(us_delay[mode]);
	}

	// now that the while loop has exited, clean up neatly and exit compeltely.
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_led_cleanup();
	rc_button_cleanup();
	rc_remove_pid_file();
	return 0;
}
