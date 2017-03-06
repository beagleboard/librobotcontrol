/******************************************************************************
* rc_check_battery.c
*
* James Strawson 2016
* Simple program to display battery state
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

#define VOLTAGE_DISCONNECT	1	// Threshold for detecting disconnected battery

int main(){
	double pack_voltage;	// 2S pack voltage on JST XH 2S balance connector
	double cell_voltage;	// cell voltage
	double jack_voltage;	// could be dc power supply or another battery

	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}

	while(rc_get_state()!=EXITING){
		// read in the voltage of the 2S pack and DC jack
		pack_voltage = rc_battery_voltage();
		jack_voltage = rc_dc_jack_voltage();

		// sanity check the SDC didn't return an error
		if(pack_voltage==-1 || jack_voltage==-1){
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
		printf("\rPack: %0.2fV   Cell: %0.2fV   DC Jack: %0.2fV  ", \
					pack_voltage, cell_voltage, jack_voltage);
		fflush(stdout);

		//check periodically
		rc_usleep(1000000);
	}
	return 0;
}
