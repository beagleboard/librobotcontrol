

#ifndef _GPIO_H_
#define _GPIO_H_

#include "am335x_gpio_defs.h"

#define HIGH (1)
#define LOW  (0) 

int init_mmap(); /*!< mmap /dev/mem into memory */
int digitalWrite(PIN p, uint8_t mode);
int digitalRead(PIN p);
int analogRead(uint8_t p);
int adc_init_mmap();

#endif


