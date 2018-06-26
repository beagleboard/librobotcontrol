/**
 * @file rc_test_buttons.c
 * @example    rc_test_buttons
 *
 * This is a very basic test of button functionality. It simply prints to the
 * screen when a button has been pressed or released.
 **/

#include <stdio.h>
#include <signal.h>
#include <rc/button.h>
#include <rc/time.h>

int running;

void on_pause_press()
{
	printf("Pause Pressed\n");
	return;
}

void on_pause_release()
{
	printf("Pause Released\n");
	return;
}

void on_mode_press()
{
	printf("Mode Pressed\n");
	return;
}

void on_mode_release()
{
	printf("Mode Released\n");
	return;
}

// interrupt handler to catch ctrl-c
void signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}

int main()
{
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

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, signal_handler);
	running = 1;

	// Assign callback functions
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,on_pause_press,on_pause_release);
	rc_button_set_callbacks(RC_BTN_PIN_MODE,on_mode_press,on_mode_release);


	//toggle leds till the program state changes
	printf("Press buttons to see response\n");
	while(running)	rc_usleep(500000);

	// cleanup and exit
	rc_button_cleanup();
	return 0;
}
