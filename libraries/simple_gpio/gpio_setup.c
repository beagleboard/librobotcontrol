
#include <stdio.h>
#include "../roboticscape-defs.h"
#include "../roboticscape.h"


int setup_output_pin(int pin, int val){

	if(gpio_export(pin)){
		printf("ERROR: Failed to export gpio pin %d\n", pin);
		return -1;
	}
	if(gpio_set_dir(pin, OUTPUT_PIN)){
		printf("ERROR: Failed to set gpio pin %d as output\n", pin);
		return -1;
	}
	if(gpio_set_value(pin, val)){
		printf("ERROR: Failed to set gpio pin %d value\n", pin);
		return -1;
	}
	return 0;
}

int setup_input_pin(int pin){

	if(gpio_export(pin)){
		printf("ERROR: Failed to export gpio pin %d\n", pin);
		return -1;
	}
	if(gpio_set_dir(pin, INPUT_PIN)){
		printf("ERROR: Failed to set gpio pin %d as output\n", pin);
		return -1;
	}
	return 0;
}


int configure_gpio_pins(){
	int mdir1a, mdir2b;
	int ret = 0;

	// Blue-only setup
	if(get_bb_model()==BB_BLUE){
		mdir1a = MDIR1A_BLUE;
		mdir2b = MDIR2B_BLUE;
		// ret |= setup_output_pin(BLUE_SPI_PIN_6_SS1, HIGH);
		// ret |= setup_output_pin(BLUE_SPI_PIN_6_SS2, HIGH);
		// ret |= setup_input_pin(BLUE_GP0_PIN_3);
		// ret |= setup_input_pin(BLUE_GP0_PIN_4);
		// ret |= setup_input_pin(BLUE_GP0_PIN_5);
		// ret |= setup_input_pin(BLUE_GP0_PIN_6);
		// ret |= setup_input_pin(BLUE_GP1_PIN_3);
		// ret |= setup_input_pin(BLUE_GP1_PIN_4);
	}
	// Cape-Only stuff
	else{
		mdir1a = MDIR1A;
		mdir2b = MDIR2B;
		// ret |= setup_output_pin(CAPE_SPI_PIN_6_SS1, HIGH);
		// ret |= setup_output_pin(CAPE_SPI_PIN_6_SS2, HIGH);
	}

	// Shared Pins

	// LEDs
	ret |= setup_output_pin(RED_LED, LOW);
	ret |= setup_output_pin(GRN_LED, LOW);

	// MOTOR Direction and Standby pins
	ret |= setup_output_pin(mdir1a, LOW);
	ret |= setup_output_pin(MDIR1B, LOW);
	ret |= setup_output_pin(MDIR2A, LOW);
	ret |= setup_output_pin(mdir2b, LOW);
	ret |= setup_output_pin(MDIR3A, LOW);
	ret |= setup_output_pin(MDIR3B, LOW);
	ret |= setup_output_pin(MDIR4A, LOW);
	ret |= setup_output_pin(MDIR4B, LOW);
	ret |= setup_output_pin(MOT_STBY, LOW);
	
	// DSM
	ret |= setup_output_pin(DSM_PIN, LOW);
	
	// servo power
	ret |= setup_output_pin(SERVO_PWR, LOW);

	// buttons
	ret |= setup_input_pin(MODE_BTN); 
	ret |= setup_input_pin(PAUSE_BTN);
	gpio_set_edge(MODE_BTN, EDGE_BOTH);
	gpio_set_edge(PAUSE_BTN, EDGE_BOTH);

	// IMU
	ret |= setup_input_pin(IMU_INTERRUPT_PIN);

	// UART1, GPS, and SPI pins
	// ret |= setup_input_pin(GPS_HEADER_PIN_3); 
	// ret |= setup_input_pin(GPS_HEADER_PIN_4);
	// ret |= setup_input_pin(UART1_HEADER_PIN_3); 
	// ret |= setup_input_pin(UART1_HEADER_PIN_4);
	// ret |= setup_input_pin(SPI_HEADER_PIN_3);
	// ret |= setup_input_pin(SPI_HEADER_PIN_4);
	// ret |= setup_input_pin(SPI_HEADER_PIN_5);


	if(ret){
		printf("WARNING: Failed to configure all gpio pins\n");
		return -1;
	}

	return 0;
}
