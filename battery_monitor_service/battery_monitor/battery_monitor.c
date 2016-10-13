/******************************************************************************
* battery_monitor.c
* James Strawson - 2016
*
* see README.txt for details
*******************************************************************************/

#include "../../libraries/usefulincludes.h"
#include "../../libraries/roboticscape.h"
#include "../../libraries/roboticscape-defs.h"
#include "../../libraries/simple_gpio/simple_gpio.h"
#include "../../libraries/mmap/mmap_gpio_adc.h"
#include <sys/file.h>

#define BATTPIDFILE	"/var/run/battery_monitor.pid"

// Critical Max voltages of packs used to detect number of cells in pack
#define CELL_MAX			4.25 // set higher than actual to detect num cells
#define VOLTAGE_FULL		3.9	 // minimum V to consider battery full
#define VOLTAGE_75			3.8	
#define VOLTAGE_50			3.55
#define VOLTAGE_25			3.1
#define VOLTAGE_DISCONNECT	2	 // Threshold for detecting disconnected battery

int running;

void illuminate_leds(int i);
int kill_existing_instance();
void shutdown_signal_handler(int signo);


// gpio designation for led 2 is a global variable
// the rest are #defines but since led 2 is different on the blue
// we must make it variable
int batt_led_2;

// main() takes only one optional argument: -k (kill)
int main(int argc, char *argv[]){
	FILE* fd;
	float pack_voltage;	// 2S pack voltage on JST XH 2S balance connector
	float jack_voltage;	// could be dc power supply or another battery
	float cell_voltage;	// cell voltage from either 2S or external pack
	int toggle = 0;
	int printing = 0;
	int num_cells, chg_leds, charging;
	int c;

	// parse arguments to check for kill mode
	opterr = 0;
	while ((c = getopt(argc, argv, "k")) != -1){
		switch (c){
		case 'k':  // kill mode
			return kill_existing_instance();
			break;
			
		default:
			printf("\nInvalid Argument \n");
			return -1;
			break;
		}
    }

	// we only want one instance running, so check is a pid file already exists
	if(access(PID_FILE, F_OK ) == 0){
		// PID file exists
		printf("instance of battery_monitor already running\n");
		return -1;
	}

	// make new pid file
	fd = fopen(BATTPIDFILE, "ab");
	if (fd < 0) {
		printf("\n error opening PID file for writing\n");
		return -1;
	}
	pid_t current_pid = getpid();
	fprintf(fd,"%d",(int)current_pid);
	fflush(fd);
	fclose(fd);

	// set up signal handler
    signal(SIGINT, shutdown_signal_handler);	
	signal(SIGTERM, shutdown_signal_handler);	

	// set led 2 gpio designation depending on board
	if(get_bb_model()==BB_BLUE) batt_led_2=BATT_LED_2_BLUE;
	else batt_led_2=BATT_LED_2;

	// open the gpio channels for 4 battery indicator LEDs
	gpio_export(BATT_LED_1);
	gpio_export(batt_led_2);
	gpio_export(BATT_LED_3);
	gpio_export(BATT_LED_4);
	gpio_set_dir(BATT_LED_1, OUTPUT_PIN);
	gpio_set_dir(batt_led_2, OUTPUT_PIN);
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
	
	// run intil running==0 which is set by signal handler
	running = 1;
	while(running){
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
			if(chg_leds>4) chg_leds=0;
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

	// exit
	illuminate_leds(0);
	printf("battery_monitor exiting cleanly\n");
	remove(PID_FILE);
	return 0;
}


/*******************************************************************************
* shutdown_signal_handler(int signo)
*
* catch Ctrl-C signal and change running state to 0 indicating exiting
*******************************************************************************/
void shutdown_signal_handler(int signo){
	if (signo == SIGINT){
		running = 0;
 	}else if (signo == SIGTERM){
		running = 0;
		printf("\nreceived SIGTERM\n");
 	}
}



int kill_existing_instance(){
	FILE* fd;
	int old_pid, i;
	
	// attempt to open PID file
	fd = fopen(BATTPIDFILE, "r");
	// if the file didn't open, no proejct is runnning in the background
	// so return 0
	if (fd == NULL) {
		return 0;
	}
	
	// otherwise try to read the current process ID
	fscanf(fd,"%d", &old_pid);
	fclose(fd);
	
	// if the file didn't contain a PID number, remove it and 
	// return -1 indicating weird behavior
	if(old_pid == 0){
		remove(BATTPIDFILE);
		return 1;
	}
		
	// attempt a clean shutdown
	kill((pid_t)old_pid, SIGINT);
	
	// check every 0.1 seconds to see if it closed 
	for(i=0; i<20; i++){
		if(getpgid(old_pid) >= 0) usleep(100000);
		else{ // succcess, it shut down properly
			remove(BATTPIDFILE);
			return 1; 
		}
	}
	
	// otherwise force kill the program if the PID file never got cleaned up
	kill((pid_t)old_pid, SIGKILL);

	// close and delete the old file
	remove(BATTPIDFILE);
	
	// return -1 indicating the program had to be killed
	return -1;
}



void illuminate_leds(int i){
	switch(i){
	// now illuminate LEDs properly
	case 4:
		mmap_gpio_write(BATT_LED_1,HIGH);
		mmap_gpio_write(batt_led_2,HIGH);
		mmap_gpio_write(BATT_LED_3,HIGH);
		mmap_gpio_write(BATT_LED_4,HIGH);
		break;
	case 3:
		mmap_gpio_write(BATT_LED_1,HIGH);
		mmap_gpio_write(batt_led_2,HIGH);
		mmap_gpio_write(BATT_LED_3,HIGH);
		mmap_gpio_write(BATT_LED_4,LOW);
		break;
	case 2:
		mmap_gpio_write(BATT_LED_1,HIGH);
		mmap_gpio_write(batt_led_2,HIGH);
		mmap_gpio_write(BATT_LED_3,LOW);
		mmap_gpio_write(BATT_LED_4,LOW);
		break;
	case 1:
		mmap_gpio_write(BATT_LED_1,HIGH);
		mmap_gpio_write(batt_led_2,LOW);
		mmap_gpio_write(BATT_LED_3,LOW);
		mmap_gpio_write(BATT_LED_4,LOW);
		break;
	case 0:
		mmap_gpio_write(BATT_LED_1,LOW);
		mmap_gpio_write(batt_led_2,LOW);
		mmap_gpio_write(BATT_LED_3,LOW);
		mmap_gpio_write(BATT_LED_4,LOW);
		break;
	default:
		printf("can only illuminate between 0 and 4 leds\n");
		break;
	}
	return;
}


