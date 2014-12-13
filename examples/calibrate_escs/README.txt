calibrate_escs
James Strawson - 2014

Project Description:
Typical brushless speed controllers (ESCs) accept a variety of pulse widths, usually from 800-2300 microseconds. Before using them, you must calibrate the ESC so it knows what pulse widths correspond to minimum and maximum throttle. One puts an ESC into calibration mode by applying any pulse width greater than roughly 1000us when it is powered on. Once in calibration mode, one applies max and min pulse widths briefly before turning off the signal to exit calibration mode. This is typically done with the throttle stick on your RC transmitter.

The calibrate_escs example assists you with this process by sending the right pulse widths. Follow the instructions that are displayed in the console when you execute the program.