/*
	Battery Monitor Service for Robotics Cape or General BeagleBone use. This program illuminates a set of 4 LEDS on the Robotics Cape to indicate battery charge level of a 2S, 3S, or 4S Lithium Ion or Polymer Battery. Also shuts down BeagleBone when voltage dips too low to protect the battery from over-discharging.
	
	James Strawson 2014
*/
// #include <stdio.h>
// #include <stdlib.h>
// #include <unistd.h>
// #include <SimpleGPIO.h>

#include <robotics_cape.h>

//Critical Max voltages of packs used to detect number of cells in pack
#define CELL_MAX			4.25	// set higher than actual to detect num cells
#define VOLTAGE_FULL		4.0		// minimum V to consider battery full
#define VOLTAGE_75			3.8	
#define VOLTAGE_50			3.65
#define VOLTAGE_25			3.45	
#define VOLTAGE_DISCONNECT	2		// Threshold for detecting disconnected battery

// gpio # for gpio_a.b = (32*a)+b
#define LED_1	27 // P8.17
#define LED_2	47 // P8.15
#define LED_3	46 // P8.16
#define LED_4	26 // P8.14


int raw_adc;
float pack_voltage;		 	// 2S pack voltage on JST XH 2S balance connector
float cell_voltage;			// cell voltage from either 2S or external pack
float jack_voltage;			// dc barrel jack, could be dc power supply or another battery
float dc_supply_connected;  // if 12v is seen on the dc jack, assume it's a power supply
int num_cells;				// 2 if only the 2S pack is used, otherwise it's the cells in external pack
int external_pack_connected; // =1 if an external lithium pack connected to dc jack
int internal_pack_connected; // =1 if a 2S lithium pack is connected to balance connector
int toggle = 0;
int printing = 0;

int main(){
	// open the gpio channels for 4 battery indicator LEDs
	// gpio_export(LED_1);
	// gpio_export(LED_2);
	// gpio_export(LED_3);
	// gpio_export(LED_4);
	// gpio_set_dir(LED_1, OUTPUT_PIN);
	// gpio_set_dir(LED_2, OUTPUT_PIN);
	// gpio_set_dir(LED_3, OUTPUT_PIN);
	// gpio_set_dir(LED_4, OUTPUT_PIN);
	
	// enable adc
	initialize_adc();
	initialize_gpio();
	
	// first decide if the user has called this from a terminal
	// or as a startup process
	if(isatty(fileno(stdout))){
		printing = 1;
		printf("\n2S Pack   Jack   #Cells   Cell\n");
	}
	
	// usually started as a background service designed to run forever
	// so loop forever
	while(1){
		
		// read in the voltage of the 2S pack and DC jack
		pack_voltage = getBattVoltage();
		jack_voltage = getJackVoltage();

		if(pack_voltage==-1 || jack_voltage==-1){
			printf("can't read battery voltages\n");
			return -1;
		}
		
		// check if a pack is on the 2S balance connector
		if(pack_voltage>VOLTAGE_DISCONNECT){
			internal_pack_connected = 1;
		}
		else{
			internal_pack_connected = 0;
		}
		
		// decide if a dc power supply is connected to the jack
		if(jack_voltage<12.15 && jack_voltage>11.75 && internal_pack_connected){
			// probably a 12v power supply connected.
			dc_supply_connected = 1;
			external_pack_connected = 0;
			num_cells = 2;
		}
		else if(jack_voltage>5){
			// seems a pack is connected
			dc_supply_connected = 0;
			external_pack_connected = 1;
			if (jack_voltage>CELL_MAX*4){
				printf("Voltage too High, use 2S-4S pack\n");
				return -1;
			}
			else if(jack_voltage>CELL_MAX*3){
				num_cells = 4;
			}
			else if(jack_voltage>CELL_MAX*2){
				num_cells = 3;
			}
			else if(jack_voltage>CELL_MAX*1){
				num_cells = 2;
			}
			else {
				num_cells = 1;
			}
		}
		else{
			num_cells = 2;
			external_pack_connected = 0;
		}

		
		// find cell voltage
		if(external_pack_connected){
			cell_voltage = jack_voltage/num_cells;
		}
		else{
			cell_voltage = pack_voltage/2;
		}
		
		// now illuminate LEDs properly
		if(cell_voltage<VOLTAGE_DISCONNECT){
			digitalWrite(LED_1,LOW);
			digitalWrite(LED_2,LOW);
			digitalWrite(LED_3,LOW);
			digitalWrite(LED_4,LOW);
		}
		else if(cell_voltage>VOLTAGE_FULL){
			digitalWrite(LED_1,HIGH);
			digitalWrite(LED_2,HIGH);
			digitalWrite(LED_3,HIGH);
			digitalWrite(LED_4,HIGH);
		}
		else if(cell_voltage>VOLTAGE_75){
			digitalWrite(LED_1,HIGH);
			digitalWrite(LED_2,HIGH);
			digitalWrite(LED_3,HIGH);
			digitalWrite(LED_4,LOW);
		}
		else if(cell_voltage>VOLTAGE_50){
			digitalWrite(LED_1,HIGH);
			digitalWrite(LED_2,HIGH);
			digitalWrite(LED_3,LOW);
			digitalWrite(LED_4,LOW);
		}
		else if(cell_voltage>VOLTAGE_25){
			digitalWrite(LED_1,HIGH);
			digitalWrite(LED_2,LOW);
			digitalWrite(LED_3,LOW);
			digitalWrite(LED_4,LOW);
		}
		else if(dc_supply_connected!=1){
			// blink battery LEDs to warn extremely low battery
			// but only if not charging
			digitalWrite(LED_1,toggle);
			digitalWrite(LED_2,toggle);
			digitalWrite(LED_3,toggle);
			digitalWrite(LED_4,toggle);
			if(toggle){
				toggle = 0;
			}
			else{
				toggle = 1;
			}
		}
		else{
			// if we've gotten here, battery is extremely low but charging
			digitalWrite(LED_1,HIGH);
			digitalWrite(LED_2,LOW);
			digitalWrite(LED_3,LOW);
			digitalWrite(LED_4,LOW);
		}
		
		if(printing){
			printf("\r %0.2fV   %0.2fV     %d     %0.2fV   ", \
				pack_voltage, jack_voltage, num_cells, cell_voltage);
			fflush(stdout);
		}
		//check periodically
		usleep(1000000);
	}
	return 0;
}