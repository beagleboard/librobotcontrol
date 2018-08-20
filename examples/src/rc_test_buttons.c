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

static int running = 0;

static void __on_pause_press(void)
{
	printf("Pause Pressed\n");
	return;
}

static void __on_pause_release(void)
{
	printf("Pause Released\n");
	return;
}

static void __on_mode_press(void)
{
	printf("Mode Pressed\n");
	return;
}

static void __on_mode_release(void)
{
	printf("Mode Released\n");
	return;
}

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
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
	signal(SIGINT, __signal_handler);
	running = 1;

	// Assign callback functions
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE, __on_pause_press, __on_pause_release);
	rc_button_set_callbacks(RC_BTN_PIN_MODE, __on_mode_press, __on_mode_release);


	//toggle leds till the program state changes
	printf("Press buttons to see response\n");
	while(running)	rc_usleep(500000);

	// cleanup and exit
	rc_button_cleanup();
	return 0;
}
