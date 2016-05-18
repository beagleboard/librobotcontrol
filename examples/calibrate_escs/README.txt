/*******************************************************************************
calibrate_escs.c

Project Description:
Typical brushless speed controllers (ESCs) accept a variety of pulse widths,
usually from 900-2100 microseconds. Before using them, you must calibrate the
ESC so it knows what pulse widths correspond to minimum and maximum throttle.
Typically ESCs go into calibration mode by applying any pulse width greater 
than roughly 1000us when it is powered on. Once in calibration mode, the user
then applies max and min pulse widths briefly before turning off the signal to
exit calibration mode. This is typically done with the throttle stick on your 
RC transmitter.

The calibrate_escs example assists you with this process by sending the right 
pulse widths. Follow the instructions that are displayed in the console when you
execute the program.

*******************************************************************************/