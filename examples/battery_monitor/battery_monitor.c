/*
	Battery Monitor Service for Robotics Cape or General BeagleBone use. This program illuminates a set of 4 LEDS on the Robotics Cape to indicate battery charge level of a 2S, 3S, or 4S Lithium Ion or Polymer Battery. Also shuts down BeagleBone when voltage dips too low to protect the battery from over-discharging.
	
	James Strawson 2014
*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <SimpleGPIO.h>

//Critical Max voltages of packs used to detect number of cells in pack
#define CELL_MAX			4.25		//set higher than actual to detect num cells
#define VOLTAGE_FULL		4.0		//minimum V to consider battery full
#define VOLTAGE_75			3.75	
#define VOLTAGE_50			3.65
#define VOLTAGE_25			3.6	
#define VOLTAGE_SHUTDOWN 	3.4		// When to shut down to prevent over discharge.
#define VOLTAGE_DISCONNECT	2		// Threshold for detecting disconnected battery
#define SHUTDOWN_WAIT		2		// Seconds votlage must me too low before shutting down
#define SAMPLES_PER_SECOND	2		//check voltage twice a second, use integers

// gpio number is first digit *32 plus second digit
#define LED_1	86
#define LED_2	88
#define LED_3	87
#define LED_4	89

FILE *AIN6_fd;
int raw_adc;
float pack_voltage;
float cell_voltage;
int num_cells;
int toggle = 0;
int shutdown_counter;

int main(){

	gpio_export(LED_1);
	gpio_export(LED_2);
	gpio_export(LED_3);
	gpio_export(LED_4);
	gpio_set_dir(LED_1, OUTPUT_PIN);
	gpio_set_dir(LED_2, OUTPUT_PIN);
	gpio_set_dir(LED_3, OUTPUT_PIN);
	gpio_set_dir(LED_4, OUTPUT_PIN);
	
	printf("\n    Pack   Cell\n");
	while(1){
		AIN6_fd = fopen("/sys/devices/ocp.3/helper.18/AIN6", "r");
		if(AIN6_fd < 0){
			printf("error reading adc\n");
			printf("Check Cape Manager Slots for Robotics Cape\n");
			return -1;
		}
		fscanf(AIN6_fd, "%i", &raw_adc);
		fclose(AIN6_fd);
		// times 11 for the voltage divider, divide by 1000 to go from mv to V
		pack_voltage= (float)raw_adc*11.0/1000.0; 
		
		if(pack_voltage<VOLTAGE_DISCONNECT){
			gpio_set_value(LED_1,LOW);
			gpio_set_value(LED_2,LOW);
			gpio_set_value(LED_3,LOW);
			gpio_set_value(LED_4,LOW);
			shutdown_counter = 0;
			num_cells = 0;
			printf("\rbattery disconnected");
			fflush(stdout);
		}
		
		// detect number of cells in pack
		// may fail with over-discharged pack
		else{
			if (pack_voltage>CELL_MAX*4){
				printf("Voltage too High, use 2S-4S pack\n");
				return -1;
			}
			else if(pack_voltage>CELL_MAX*3){
				num_cells = 4;
			}
			else if(pack_voltage>CELL_MAX*2){
				num_cells = 3;
			}
			else {
				num_cells = 2;
			}
			
			cell_voltage = pack_voltage/num_cells;
			
			if(cell_voltage<VOLTAGE_SHUTDOWN){
				//the user left their BBB on
				//shutdown to protect battery after time period
				shutdown_counter ++;
				if(shutdown_counter>SHUTDOWN_WAIT*SAMPLES_PER_SECOND){
					shutdown_counter = 0;
					printf("going for shutdown!\n");
					system("shutdown -P now");
				}
			}
			else if(cell_voltage>VOLTAGE_FULL){
				gpio_set_value(LED_1,HIGH);
				gpio_set_value(LED_2,HIGH);
				gpio_set_value(LED_3,HIGH);
				gpio_set_value(LED_4,HIGH);
				shutdown_counter = 0;
			}
			else if(cell_voltage>VOLTAGE_75){
				gpio_set_value(LED_1,LOW);
				gpio_set_value(LED_2,HIGH);
				gpio_set_value(LED_3,HIGH);
				gpio_set_value(LED_4,HIGH);
				shutdown_counter = 0;
			}
			else if(cell_voltage>VOLTAGE_50){
				gpio_set_value(LED_1,LOW);
				gpio_set_value(LED_2,LOW);
				gpio_set_value(LED_3,HIGH);
				gpio_set_value(LED_4,HIGH);
				shutdown_counter = 0;
			}
			else if(cell_voltage>VOLTAGE_25){
				gpio_set_value(LED_1,LOW);
				gpio_set_value(LED_2,LOW);
				gpio_set_value(LED_3,LOW);
				gpio_set_value(LED_4,HIGH);
				shutdown_counter = 0;
			}
			else{
				//blink battery LEDs to warn extremely low battery
				gpio_set_value(LED_1,toggle);
				gpio_set_value(LED_2,toggle);
				gpio_set_value(LED_3,toggle);
				gpio_set_value(LED_4,toggle);
				if(toggle){
					toggle = 0;
				}
				else{
					toggle = 1;
				}
			}
			printf("\r%dS  %0.2fV  %0.2fV   ", num_cells, pack_voltage, cell_voltage);
			fflush(stdout);
		}
		//check periodically
		usleep(1000000/SAMPLES_PER_SECOND);
	}
	return 0;
}