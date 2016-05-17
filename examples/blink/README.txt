/*******************************************************************************
* blink.c
*
* This is an example program to demonstrate use of LEDs and button handlers
* in the robotics cape API. Once started, blink will flash the green and red
* LEDs. Pressing the mode button will cycle through 3 blinking speeds, slow
* medium, and fast. Momentarily pressing the pause button will stop and start
* the blinking by toggling the global state between PAUSED and RUNNING. If the 
* user holds the pause button for more than 1.5 seconds then the blink program
* will flash the red LED and exit cleanly.
*
* This should be used as a reference for how to handle buttons and how to
* control program flow cleanly utilizing get_state() and set_state().
*
*******************************************************************************/