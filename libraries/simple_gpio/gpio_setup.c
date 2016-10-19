

#include "../roboticscape-defs.h"
#include "simple_gpio.h"
#include "../roboticscape.h"

int configure_gpio_pins(){
	int mdir1a, mdir2b;


	// board_specific setup
	if(get_bb_model()==BB_BLUE){
		mdir1a = MDIR1A_BLUE;
		mdir2b = MDIR2B_BLUE;
	}
	else{
		mdir1a = MDIR1A;
		mdir2b = MDIR2B;
	}

	// LEDs
	if(gpio_export(RED_LED)) return -1;
	if(gpio_set_dir(RED_LED, OUTPUT_PIN)) return -1;
	if(gpio_set_value(RED_LED, LOW)) return -1;
	gpio_export(GRN_LED);
	gpio_set_dir(GRN_LED, OUTPUT_PIN);

	// motor
	gpio_export(mdir1a);
	gpio_set_dir(mdir1a, OUTPUT_PIN);
	gpio_set_value(mdir1a, LOW);
	gpio_export(MDIR1B);
	gpio_set_dir(MDIR1B, OUTPUT_PIN);
	gpio_set_value(MDIR1B, LOW);
	gpio_export(MDIR2A);
	gpio_set_dir(MDIR2A, OUTPUT_PIN);
	gpio_set_value(MDIR2A, LOW);
	gpio_export(mdir2b);
	gpio_set_dir(mdir2b, OUTPUT_PIN);
	gpio_set_value(mdir2b, LOW);
	gpio_export(MDIR3A);
	gpio_set_dir(MDIR3A, OUTPUT_PIN);
	gpio_set_value(MDIR3A, LOW);
	gpio_export(MDIR3B);
	gpio_set_dir(MDIR3B, OUTPUT_PIN);
	gpio_set_value(MDIR3B, LOW);
	gpio_export(MDIR4A);
	gpio_set_dir(MDIR4A, OUTPUT_PIN);
	gpio_set_value(MDIR4A, LOW);
	gpio_export(MDIR4B);
	gpio_set_dir(MDIR4B, OUTPUT_PIN);
	gpio_set_value(MDIR4B, LOW);
	gpio_export(MOT_STBY);
	gpio_set_dir(MOT_STBY, OUTPUT_PIN);
	gpio_set_value(MOT_STBY, LOW);

	// dsm2 pairing pin
	gpio_export(PAIRING_PIN);
	gpio_set_dir(PAIRING_PIN, OUTPUT_PIN);
	gpio_set_value(PAIRING_PIN, LOW);

	// servo power
	gpio_export(SERVO_PWR);
	gpio_set_dir(SERVO_PWR, OUTPUT_PIN);
	gpio_set_value(SERVO_PWR, LOW);

	// spi
	gpio_export(SPI1_SS1_GPIO_PIN);
	gpio_set_dir(SPI1_SS1_GPIO_PIN, OUTPUT_PIN);
	gpio_set_value(SPI1_SS1_GPIO_PIN, HIGH);
	gpio_export(SPI1_SS2_GPIO_PIN);
	gpio_set_dir(SPI1_SS2_GPIO_PIN, OUTPUT_PIN);
	gpio_set_value(SPI1_SS2_GPIO_PIN, HIGH);

	// buttons
	gpio_export(MODE_BTN);
	gpio_set_dir(MODE_BTN, INPUT_PIN);
	gpio_set_edge(MODE_BTN, "both");  // Can be rising, falling or both
	gpio_export(PAUSE_BTN);
	gpio_set_dir(PAUSE_BTN, INPUT_PIN);
	gpio_set_edge(PAUSE_BTN, "both");  // Can be rising, falling or both

	return 0;
}
