/******************************************************************************
* battery_monitor.c
* James Strawson - 2016
*
* see README.txt for details
*******************************************************************************/

#include <useful_includes.h>
#include <robotics_cape.h>
#include <robotics_cape_defs.h>
#include <simple_gpio/simple_gpio.h>
#include <mmap/mmap_gpio_adc.h>
#include <sys/file.h>

// Critical Max voltages of packs used to detect number of cells in pack
#define LOCKFILE	"/run/battery_monitor.lock"
#define CELL_MAX			4.25 // set higher than actual to detect num cells
#define VOLTAGE_FULL		4.0	 // minimum V to consider battery full
#define VOLTAGE_75			3.8	
#define VOLTAGE_50			3.6
#define VOLTAGE_25			3.45	
#define VOLTAGE_DISCONNECT	2	 // Threshold for detecting disconnected battery

void illuminate_leds(int i){
	switch(i){
	// now illuminate LEDs properly
	case 4:
		mmap_gpio_write(BATT_LED_1,HIGH);
		mmap_gpio_write(BATT_LED_2,HIGH);
		mmap_gpio_write(BATT_LED_3,HIGH);
		mmap_gpio_write(BATT_LED_4,HIGH);
		break;
	case 3:
		mmap_gpio_write(BATT_LED_1,HIGH);
		mmap_gpio_write(BATT_LED_2,HIGH);
		mmap_gpio_write(BATT_LED_3,HIGH);
		mmap_gpio_write(BATT_LED_4,LOW);
		break;
	case 2:
		mmap_gpio_write(BATT_LED_1,HIGH);
		mmap_gpio_write(BATT_LED_2,HIGH);
		mmap_gpio_write(BATT_LED_3,LOW);
		mmap_gpio_write(BATT_LED_4,LOW);
		break;
	case 1:
		mmap_gpio_write(BATT_LED_1,HIGH);
		mmap_gpio_write(BATT_LED_2,LOW);
		mmap_gpio_write(BATT_LED_3,LOW);
		mmap_gpio_write(BATT_LED_4,LOW);
		break;
	case 0:
		mmap_gpio_write(BATT_LED_1,LOW);
		mmap_gpio_write(BATT_LED_2,LOW);
		mmap_gpio_write(BATT_LED_3,LOW);
		mmap_gpio_write(BATT_LED_4,LOW);
		break;
	default:
		printf("can only illuminate between 0 and 4 leds\n");
		break;
	}
	return;
}
	


int main(){
	float pack_voltage;	// 2S pack voltage on JST XH 2S balance connector
	float jack_voltage;	// could be dc power supply or another battery
	float cell_voltage;	// cell voltage from either 2S or external pack
	int toggle = 0;
	int printing = 0;
	int num_cells, chg_leds, charging;
	
	// we only want one instance running, so check the lockfile
	int lockfile = open(LOCKFILE, O_CREAT | O_RDWR, 0666);
	if(flock(lockfile, LOCK_EX | LOCK_NB)) {
		printf("\nBattery_monitor already running in background\n");
		printf("This program is started in the background at boot\n");
		printf("and does not need to be run by the user.\n\n");
		printf("Use check_battery instead.\n\n");
		return -1;
	}

	// open the gpio channels for 4 battery indicator LEDs
	gpio_export(BATT_LED_1);
	gpio_export(BATT_LED_2);
	gpio_export(BATT_LED_3);
	gpio_export(BATT_LED_4);
	gpio_set_dir(BATT_LED_1, OUTPUT_PIN);
	gpio_set_dir(BATT_LED_2, OUTPUT_PIN);
	gpio_set_dir(BATT_LED_3, OUTPUT_PIN);
	gpio_set_dir(BATT_LED_4, OUTPUT_PIN);
	
	// enable adc
	initialize_mmap_adc();
	initialize_mmap_gpio();
	
	// first decide if the user has called this from a terminal
	// or as a startup process
	if(isatty(fileno(stdout))){
		printing = 1;
		printf("\n2S Pack   Jack   #Cells   Cell\n");
	}
	
	// usually started as a background service designed to run forever
	while(1){
		charging = 0;
		// read in the voltage of the 2S pack and DC jack
		pack_voltage = get_battery_voltage();
		jack_voltage = get_dc_jack_voltage();

		if(pack_voltage==-1 || jack_voltage==-1){
			printf("can't read ADC voltages\n");
			return -1;
		}
		
		// check if a pack is on the 2S balance connector and if it's charging
		if(pack_voltage>(2*VOLTAGE_DISCONNECT)){
			cell_voltage = pack_voltage/2;
			if(jack_voltage>10.0 && cell_voltage<VOLTAGE_FULL){
				charging = 1;
			}
		}
		
		// no 2S pack on the White 3-pin connector, check for dc jack
		else{
			if (jack_voltage>CELL_MAX*4){
				printf("Voltage too High, use 2S-4S pack\n");
				cell_voltage = 0;
			}
			else if(jack_voltage>CELL_MAX*3){ // check for 4S condition
				cell_voltage = jack_voltage/4;
			}
			else if(jack_voltage>CELL_MAX*2){ // check for 3S condition
				num_cells = 3;
			}
			else if(jack_voltage>CELL_MAX*1){ // check for 2S condition
				num_cells = 2;
			}
			else {
				cell_voltage = 0;
			}
		}
	
		// if charging, blink in a charging pattern
		if(charging){
			chg_leds += 1;
			if(chg_leds>4) chg_leds=1;
			illuminate_leds(chg_leds);
			goto END;
		}
		
		// now illuminate LEDs properly while discharging
		if(cell_voltage<VOLTAGE_DISCONNECT) illuminate_leds(0);
		else if(cell_voltage>VOLTAGE_FULL) 	illuminate_leds(4);
		else if(cell_voltage>VOLTAGE_75) 	illuminate_leds(3);
		else if(cell_voltage>VOLTAGE_50) 	illuminate_leds(2);
		else if(cell_voltage>VOLTAGE_25) 	illuminate_leds(1);
		else{
			// if we've gotten here, battery is extremely low, blink all 4
			if(toggle) toggle=0;
			else toggle=4;
			illuminate_leds(toggle);
		}
		
		if(printing){
			printf("\r %0.2fV   %0.2fV     %d     %0.2fV   ", \
				pack_voltage, jack_voltage, num_cells, cell_voltage);
			fflush(stdout);
		}
END:		
		//check periodically
		usleep(500000);
	}
	return 0;
}
