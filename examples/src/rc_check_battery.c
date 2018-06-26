/**
 * @file rc_check_battery.c
 * @example    rc_check_battery
 *
 * Simple program to display battery state
 */

#include <stdio.h>
#include <rc/adc.h>
#include <rc/start_stop.h>
#include <rc/time.h>

#define VOLTAGE_DISCONNECT	1 // Threshold for detecting disconnected battery

int main()
{
	double pack_voltage;	// 2S pack voltage on JST XH 2S balance connector
	double cell_voltage;	// cell voltage
	double jack_voltage;	// could be dc power supply or another battery

	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1) return -1;

	if(rc_adc_init()==-1) return -1;

	while(rc_get_state()!=EXITING){
		// read in the voltage of the 2S pack and DC jack
		pack_voltage = rc_adc_batt();
		jack_voltage = rc_adc_dc_jack();

		// sanity check the SDC didn't return an error
		if(pack_voltage<0.0 || jack_voltage<0.0){
			fprintf(stderr,"ERROR: can't read voltages\n");
			return -1;
		}

		// check if a pack is on the 2S balance connector
		if(pack_voltage<VOLTAGE_DISCONNECT){
			pack_voltage = 0;
		}

		if(jack_voltage<VOLTAGE_DISCONNECT){
			jack_voltage = 0;
		}

		// 2S pack, so divide by two for cell voltage
		cell_voltage = pack_voltage/2;

		// print results
		printf("\rPack: %0.2lfV   Cell: %0.2lfV   DC Jack: %0.2lfV  ", \
					pack_voltage, cell_voltage, jack_voltage);
		fflush(stdout);

		//check periodically
		rc_usleep(100000);
	}

	rc_adc_cleanup();
	return 0;
}
