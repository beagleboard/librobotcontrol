/******************************************************************************
* check_battery.c
*
* James Strawson 2016
* Simple program to display battery state
*******************************************************************************/
#include <useful_includes.h>
#include <robotics_cape.h>

#define VOLTAGE_DISCONNECT	1	 // Threshold for detecting disconnected battery

int main(){
	float pack_voltage;	// 2S pack voltage on JST XH 2S balance connector
	float cell_voltage;	// cell voltage
	float jack_voltage;	// could be dc power supply or another battery

	if(initialize_cape()){
		printf("ERROR: failed to initialize_cape()\n");
		return -1;
	}
	
	while(get_state()!=EXITING){
		// read in the voltage of the 2S pack and DC jack
		pack_voltage = get_battery_voltage();
		jack_voltage = get_dc_jack_voltage();

		// sanity check the SDC didn't return an error
		if(pack_voltage==-1 || jack_voltage==-1){
			printf("can't read battery voltages\n");
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
		usleep(1000000);
	}
	return 0;
}