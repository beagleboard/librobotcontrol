/**
 * @file gpio.c
 * @author Ethan Hayon
 *
 * This file contains GPIO functions using high
 * performance mmap of /dev/mem
 *
 * Licensed under the MIT License (MIT)
 * See MIT-LICENSE file for more information
 */
 
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#include "mmap_gpio.h"


static volatile uint32_t *map;
static char mapped = 0;

/**
 * map /dev/mem to memory
 *
 * @returns whether or not the mapping of /dev/mem into memory was successful
 */
int init_mmap() {
	if(!mapped) {
		int fd;
		fd = open("/dev/mem", O_RDWR);
		if(fd == -1) {
			perror("Unable to open /dev/mem");
			exit(EXIT_FAILURE);
		}
		map = (uint32_t*)mmap(NULL, MMAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, MMAP_OFFSET);
		if(map == MAP_FAILED) {
			close(fd);
			perror("Unable to map /dev/mem");
			exit(EXIT_FAILURE);
		}
		mapped = TRUE;
	}
	return mapped;
}



/**
 * Set a GPIO digital output * @param p Pin to write to
 *
 * @param mode Position to set the pin, HIGH or LOW
 * @returns output was successfully written
 */
int digitalWrite(PIN p, uint8_t mode) {
	//init_mmap();
	map[(p.gpio_bank-MMAP_OFFSET+GPIO_OE)/4] &= ~(1<<p.bank_id);
	if(mode == HIGH) map[(p.gpio_bank-MMAP_OFFSET+GPIO_DATAOUT)/4] |= 1<<p.bank_id;
	else map[(p.gpio_bank-MMAP_OFFSET+GPIO_DATAOUT)/4] &= ~(1<<p.bank_id);

	return 1;
}

/**
 * Read the input from a digital input. You must set 
 * the pin as an INPUT using the pinMode function
 *
 * @param p Pin to read from
 * @returns the value of the pin
 */
int digitalRead(PIN p) {
	init_mmap();
	return (map[(p.gpio_bank-MMAP_OFFSET+GPIO_DATAIN)/4] & (1<<p.bank_id))>>p.bank_id;
}


/*
 * Initialize the Analog-Digital Converter
 * each channel is set up in software one-shot mode for general purpose reading
 * no averaging or delays are used for fastest speed
 */
int adc_init_mmap() {
	init_mmap();
	
	// disable adc
	map[(ADC_CTRL-MMAP_OFFSET)/4] &= !0x01;
	
	// enable the CM_WKUP_ADC_TSC_CLKCTRL with CM_WKUP_MODUELEMODE_ENABLE
	map[(CM_WKUP_ADC_TSC_CLKCTRL-MMAP_OFFSET)/4] |= CM_WKUP_MODULEMODE_ENABLE;

	// waiting for adc clock module to initialize
	while(!(map[(CM_WKUP_ADC_TSC_CLKCTRL-MMAP_OFFSET)/4] & CM_WKUP_MODULEMODE_ENABLE)) {
		//printf("Waiting for CM_WKUP_ADC_TSC_CLKCTRL to enable with MODULEMODE_ENABLE\n"); 
	}
	
	// make sure STEPCONFIG write protect is off
	map[(ADC_CTRL-MMAP_OFFSET)/4] |= ADC_STEPCONFIG_WRITE_PROTECT_OFF;

	// set up each ADCSTEPCONFIG for each ain pin
	map[(ADCSTEPCONFIG1-MMAP_OFFSET)/4] = 0x00<<19 | ADC_AVG8 | ADC_SW_ONESHOT;
	map[(ADCSTEPDELAY1-MMAP_OFFSET)/4]  = 0<<24;
	map[(ADCSTEPCONFIG2-MMAP_OFFSET)/4] = 0x01<<19 | ADC_AVG8 | ADC_SW_ONESHOT;
	map[(ADCSTEPDELAY2-MMAP_OFFSET)/4]  = 0<<24;
	map[(ADCSTEPCONFIG3-MMAP_OFFSET)/4] = 0x02<<19 | ADC_AVG8 | ADC_SW_ONESHOT;
	map[(ADCSTEPDELAY3-MMAP_OFFSET)/4]  = 0<<24;
	map[(ADCSTEPCONFIG4-MMAP_OFFSET)/4] = 0x03<<19 | ADC_AVG8 | ADC_SW_ONESHOT;
	map[(ADCSTEPDELAY4-MMAP_OFFSET)/4]  = 0<<24;
	map[(ADCSTEPCONFIG5-MMAP_OFFSET)/4] = 0x04<<19 | ADC_AVG8 | ADC_SW_ONESHOT;
	map[(ADCSTEPDELAY5-MMAP_OFFSET)/4]  = 0<<24;
	map[(ADCSTEPCONFIG6-MMAP_OFFSET)/4] = 0x05<<19 | ADC_AVG8 | ADC_SW_ONESHOT;
	map[(ADCSTEPDELAY6-MMAP_OFFSET)/4]  = 0<<24;
	map[(ADCSTEPCONFIG7-MMAP_OFFSET)/4] = 0x06<<19 | ADC_AVG8 | ADC_SW_ONESHOT;
	map[(ADCSTEPDELAY7-MMAP_OFFSET)/4]  = 0<<24;
	map[(ADCSTEPCONFIG8-MMAP_OFFSET)/4] = 0x07<<19 | ADC_AVG8 | ADC_SW_ONESHOT;
	map[(ADCSTEPDELAY8-MMAP_OFFSET)/4]  = 0<<24;
	
	// enable the ADC
	map[(ADC_CTRL-MMAP_OFFSET)/4] |= 0x01;
		
	// clear the FIFO buffer
	int output;
	while(map[(FIFO0COUNT-MMAP_OFFSET)/4] & FIFO_COUNT_MASK){
		output =  map[(ADC_FIFO0DATA-MMAP_OFFSET)/4] & ADC_FIFO_MASK;
	}
	
	// just suppress the warning about output not being used
	if(output){}
	
	return 0;
}

/*
 * Read in from an analog pin
 *
 * @param p pin to read value from
 * @returns the analog value of pin p
 */
int analogRead(uint8_t p) {
		  
	// clear the FIFO buffer just in case it's not empty
	int output;
	while(map[(FIFO0COUNT-MMAP_OFFSET)/4] & FIFO_COUNT_MASK){
		output =  map[(ADC_FIFO0DATA-MMAP_OFFSET)/4] & ADC_FIFO_MASK;
	}
		
	// enable step for the right pin
	map[(ADC_STEPENABLE-MMAP_OFFSET)/4] |= (0x01<<(p+1));
	
	// wait for sample to appear in the FIFO buffer
	while(!(map[(FIFO0COUNT-MMAP_OFFSET)/4] & FIFO_COUNT_MASK)){}
	
	// return the the FIFO0 data register
	output =  map[(ADC_FIFO0DATA-MMAP_OFFSET)/4] & ADC_FIFO_MASK;
	
	return output;
}

