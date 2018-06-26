/**
 * @example    rc_calibrate_dsm.c
 *
 * Running the rc_calibrate_dsm example will print out raw data to the console
 * and record the min and max values for each channel. These limits will be
 * saved to disk so future dsm reads will be scaled correctly.
 *
 * Make sure the transmitter and receiver are paired before testing. Use the
 * rc_bind_dsm example if you haven't already used a bind plug and standard
 * receiver to pair. The satellite receiver remembers which transmitter it is
 * paired to, not your BeagleBone.
 */

#include <stdio.h>
#include <rc/dsm.h>

int main()
{
	printf("Please connect a DSM satellite receiver and make sure\n");
	printf("your transmitter is on and paired to the receiver.\n");
	printf("\n");
	printf("Press ENTER to continue or anything else to quit\n");
	getchar();

	// run the calibration routine
	rc_dsm_calibrate_routine();

	return 0;
}