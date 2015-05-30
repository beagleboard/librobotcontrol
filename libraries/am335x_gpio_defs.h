/**
 * @file am335x.h
 * @author Ethan Hayon
 *
 * This file contains important config for TI am335x ARM Cortex-A8 SoC
 * 
 * PLEASE DO NOT MODIFY
 *
 * Licensed under the MIT License (MIT)
 * See MIT-LICENSE file for more information
 */

#ifndef _AM335X_H_
#define _AM335X_H_

#define MMAP_OFFSET (0x44C00000)
#define MMAP_SIZE   (0x481AEFFF-MMAP_OFFSET)

/* Clock Module Memory Registers */
#define CM_WKUP (0x44E00400)
#define CM_WKUP_ADC_TSC_CLKCTRL (CM_WKUP+0xBC)
#define CM_WKUP_MODULEMODE_ENABLE (0x02)
#define CM_WKUP_IDLEST_DISABLED (0x03<<16)

#define CM_PER (0x44E00000)
#define CM_PER_EPWMSS1_CLKCTRL (CM_PER+0xCC)
#define CM_PER_EPWMSS0_CLKCTRL (CM_PER+0xD4)
#define CM_PER_EPWMSS2_CLKCTRL (CM_PER+0xD8)


/* GPIO Memory Registers */
#define GPIO_REGISTER_SIZE (4)

#define GPIO0 	(0x44E07000)
#define GPIO1		(0x4804C000)
#define GPIO2		(0x481AC000)
#define GPIO3		(0x481AE000)

#define GPIO_CLEARDATAOUT (0x190)
#define GPIO_SETDATAOUT   (0x194)
#define GPIO_OE			      (0x134)
#define GPIO_DATAOUT      (0x13C)
#define GPIO_DATAIN       (0x138)

/* Analog Digital Converter Memory Registers */
#define ADC_TSC (0x44E0D000)

#define ADC_CTRL (ADC_TSC+0x40)
#define ADC_STEPCONFIG_WRITE_PROTECT_OFF (0x01<<2)
#define ADC_STEPENABLE (ADC_TSC+0x54)

#define ADCSTEPCONFIG1 (ADC_TSC+0x64)
#define ADCSTEPDELAY1  (ADC_TSC+0x68)
#define ADCSTEPCONFIG2 (ADC_TSC+0x6C)
#define ADCSTEPDELAY2  (ADC_TSC+0x70)
#define ADCSTEPCONFIG3 (ADC_TSC+0x74)
#define ADCSTEPDELAY3  (ADC_TSC+0x78)
#define ADCSTEPCONFIG4 (ADC_TSC+0x7C)
#define ADCSTEPDELAY4  (ADC_TSC+0x80)
#define ADCSTEPCONFIG5 (ADC_TSC+0x84)
#define ADCSTEPDELAY5  (ADC_TSC+0x88)
#define ADCSTEPCONFIG6 (ADC_TSC+0x8C)
#define ADCSTEPDELAY6  (ADC_TSC+0x90)
#define ADCSTEPCONFIG7 (ADC_TSC+0x94)
#define ADCSTEPDELAY7  (ADC_TSC+0x98)
#define ADCSTEPCONFIG8 (ADC_TSC+0x9C)
#define ADCSTEPDELAY8  (ADC_TSC+0xA0)

#define ADC_AVG1 (0b000 << 2)
#define ADC_AVG2 (0b001 << 2)
#define ADC_AVG4 (0b010 << 2)
#define ADC_AVG8 (0b011 << 2)
#define ADC_AVG16 (0b100 << 2)

#define ADC_SW_ONESHOT 0b00
#define FIFO0COUNT (ADC_TSC+0xE4)
#define FIFO_COUNT_MASK 0b01111111

#define ADC_FIFO0DATA (ADC_TSC+0x100)
#define ADC_FIFO_MASK (0xFFF)


typedef struct s_PIN {
  char *name;   /*!< readable name of pin, i.e.: "GPIO1_21", see beaglebone user guide */
  unsigned int gpio_bank; /*!< which of the four gpio banks is this pin in, i.e.: GPIO1, r 0x4804C000 */
  uint8_t gpio; /*!< pin number on the am335x processor */
  uint8_t bank_id; /*!< pin number within each bank, should be 0-31 */
  char *mux;    /*!< file name for setting mux */
  uint8_t eeprom; /*!< position in eeprom */
} PIN;

#define TRUE 1
#define FALSE 0

#define USR0  ((PIN){	  "GPIO1_21",	   GPIO1,     0,   21,   "",				0 })
#define USR1  ((PIN){	  "GPIO1_22",	   GPIO1,     0,   22,   "",				0 })
#define USR2  ((PIN){	  "GPIO1_23",	   GPIO1,     0,   23,   "",				0 })
#define USR3  ((PIN){	  "GPIO1_24",	   GPIO1,     0,   24,   "",				0 })
#define P8_3  ((PIN){   "GPIO1_6",	   GPIO1,     38, 	6,	 "gpmc_ad6",        26 })
#define P8_4  ((PIN){   "GPIO1_7",     GPIO1,     39, 	7,	 "gpmc_ad7",        27 })
#define P8_5  ((PIN){   "GPIO1_2",     GPIO1,     34, 	2,	 "gpmc_ad2",        22 })
#define P8_6  ((PIN){   "GPIO1_3",     GPIO1,     35, 	3,	 "gpmc_ad3",        23 })
#define P8_7  ((PIN){   "TIMER4",      GPIO2,     66, 	2, 	 "gpmc_advn_ale",   41 })
#define P8_8  ((PIN){   "TIMER7",      GPIO2,     67, 	3,	 "gpmc_oen_ren",    44  })
#define P8_9  ((PIN){   "TIMER5",      GPIO2,     69, 	5,	 "gpmc_ben0_cle",   42 })
#define P8_10 ((PIN){   "TIMER6",      GPIO2,     68, 	4,	 "gpmc_wen",        43 })
#define P8_11 ((PIN){   "GPIO1_13",    GPIO1,     45, 	13,	 "gpmc_ad13",       29 })
#define P8_12 ((PIN){   "GPIO1_12",    GPIO1,     44, 	12,	 "gpmc_ad12",       28 })
#define P8_13 ((PIN){   "EHRPWM2B",    GPIO0,     23, 	23,	 "gpmc_ad9",        15 })
#define P8_14 ((PIN){   "GPIO0_26",    GPIO0,     26, 	26,	 "gpmc_ad10",       16 })
#define P8_15 ((PIN){   "GPIO1_15",    GPIO1,     47, 	15,	 "gpmc_ad15",       31 })
#define P8_16 ((PIN){   "GPIO1_14",    GPIO1,     46, 	14,	 "gpmc_ad14",       30 })
#define P8_17 ((PIN){   "GPIO0_27",    GPIO0,     27, 	27,	 "gpmc_ad11",       17 })
#define P8_18 ((PIN){   "GPIO2_1",     GPIO2,     65, 	1,	 "gpmc_clk",        40 })
#define P8_19 ((PIN){   "EHRPWM2A",    GPIO0,     22, 	22,	 "gpmc_ad8",        14 })
#define P8_20 ((PIN){   "GPIO1_31",    GPIO1,     63, 	31,	 "gpmc_csn2",       39 })
#define P8_21 ((PIN){   "GPIO1_30",    GPIO1,     62, 	30,	 "gpmc_csn1",       38 })
#define P8_22 ((PIN){   "GPIO1_5",     GPIO1,     37, 	5,	 "gpmc_ad5",        25 })
#define P8_23 ((PIN){   "GPIO1_4",     GPIO1,     36, 	4,	 "gpmc_ad4",        24 })
#define P8_24 ((PIN){   "GPIO1_1",     GPIO1,     33, 	1,	 "gpmc_ad1",        21 })
#define P8_25 ((PIN){   "GPIO1_0",     GPIO1,     32, 	0,	 "gpmc_ad0",        20 })
#define P8_26 ((PIN){   "GPIO1_29",    GPIO1,     61, 	29,	 "gpmc_csn0",       37 })
#define P8_27 ((PIN){   "GPIO2_22",    GPIO2,     86, 	22,	 "lcd_vsync",       57 })
#define P8_28 ((PIN){   "GPIO2_24",    GPIO2,     88, 	24,	 "lcd_pclk",        59 })
#define P8_29 ((PIN){   "GPIO2_23",    GPIO2,     87, 	23,	 "lcd_hsync",       58 })
#define P8_30 ((PIN){   "GPIO2_25",    GPIO2,     89, 	25,	 "lcd_ac_bias_en",  60 })
#define P8_31 ((PIN){   "UART5_CTSN",  GPIO0,     10, 	10,	 "lcd_data14",       7 })
#define P8_32 ((PIN){   "UART5_RTSN",  GPIO0,     11, 	11,	 "lcd_data15",       8 })
#define P8_33 ((PIN){   "UART4_RTSN",  GPIO0,     9,  	9,	 "lcd_data13",       6 })
#define P8_34 ((PIN){   "UART3_RTSN",  GPIO2,     81, 	17,	 "lcd_data11",      56 })
#define P8_35 ((PIN){   "UART4_CTSN",  GPIO0,     8,  	8,	 "lcd_data12",       5 })
#define P8_36 ((PIN){   "UART3_CTSN",  GPIO2,     80, 	16,	 "lcd_data10",      55 })
#define P8_37 ((PIN){   "UART5_TXD",   GPIO2,     78, 	14,	 "lcd_data8",       53 })
#define P8_38 ((PIN){   "UART5_RXD",   GPIO2,     79, 	15,	 "lcd_data9",       54 })
#define P8_39 ((PIN){   "GPIO2_12",    GPIO2,     76, 	12,	 "lcd_data6",       51 })
#define P8_40 ((PIN){   "GPIO2_13",    GPIO2,     77, 	13,	 "lcd_data7",       52 })
#define P8_41 ((PIN){   "GPIO2_10",    GPIO2,     74, 	10,	 "lcd_data4",       49 })
#define P8_42 ((PIN){   "GPIO2_11",    GPIO2,     75, 	11,	 "lcd_data5",       50 })
#define P8_43 ((PIN){   "GPIO2_8",     GPIO2,     72, 	8,	 "lcd_data2",       47 })
#define P8_44 ((PIN){   "GPIO2_9",     GPIO2,     73, 	9,	 "lcd_data3",       48 })
#define P8_45 ((PIN){   "GPIO2_6",     GPIO2,     70, 	6,	 "lcd_data0",       45 })
#define P8_46 ((PIN){   "GPIO2_7",     GPIO2,     71, 	7,	 "lcd_data1",       46 })

#define P9_11 ((PIN){   "UART4_RXD",   GPIO0,     30,  30,   "gpmc_wait0",       18 })
#define P9_12 ((PIN){   "GPIO1_28",    GPIO1,     60,  28,   "gpmc_ben1",        36 })
#define P9_13 ((PIN){   "UART4_TXD",   GPIO0,     31,  31,   "gpmc_wpn",         19 })
#define P9_14 ((PIN){   "EHRPWM1A",    GPIO1,     50,  18,   "gpmc_a2",          34 })
#define P9_15 ((PIN){   "GPIO1_16",    GPIO1,     48,  16,   "mii1_rxd3",        32 })
#define P9_16 ((PIN){   "EHRPWM1B",    GPIO1,     51,  19,   "gpmc_a3",          35 })
#define P9_17 ((PIN){   "I2C1_SCL",    GPIO0,      5,   5,   "spi0_cs0",          3 })
#define P9_18 ((PIN){   "I2C1_SDA",    GPIO0,      4,   4,   "spi0_d1",           2 })
#define P9_19 ((PIN){   "I2C2_SCL",    GPIO0,     13,  13,   "uart1_rtsn",        9 })
#define P9_20 ((PIN){   "I2C2_SDA",    GPIO0,     12,  12,   "uart1_ctsn",       10 })
#define P9_21 ((PIN){   "UART2_TXD",   GPIO0,      3,   3,   "spi0_d0",           1 })
#define P9_22 ((PIN){   "UART2_RXD",   GPIO0,      2,   2,   "spi0_sclk",         0 })
#define P9_23 ((PIN){   "GPIO1_17",    GPIO1,     49,  17,   "gpmc_a1",          33 })
#define P9_24 ((PIN){   "UART1_TXD",   GPIO0,     15,  15,   "uart1_txd",        12 })
#define P9_25 ((PIN){   "GPIO3_21",    GPIO3,    117,  21,   "mcasp0_ahclkx",    66 })
#define P9_26 ((PIN){   "UART1_RXD",   GPIO0,     14,  14,   "uart1_rxd",        11 })
#define P9_27 ((PIN){   "GPIO3_19",    GPIO3,    115,  19,   "mcasp0_fsr",       64 })
#define P9_28 ((PIN){   "SPI1_CS0",    GPIO3,    113,  17,   "mcasp0_ahclkr",    63 })
#define P9_29 ((PIN){   "SPI1_D0",     GPIO3,    111,  15,   "mcasp0_fsx",       61 })
#define P9_30 ((PIN){   "SPI1_D1",     GPIO3,    112,  16,   "mcasp0_axr0",      62 })
#define P9_31 ((PIN){   "SPI1_SCLK",   GPIO3,    110,  14,   "mcasp0_aclkx",     65 })
#define P9_33 ((PIN){   "AIN4",        0,          4,   4,   "",                 71 })
#define P9_35 ((PIN){   "AIN6",        0,          6,   6,   "",                 73 })
#define P9_36 ((PIN){   "AIN5",        0,          5,   5,   "",                 72 })
#define P9_37 ((PIN){   "AIN2",        0,          2,   2,   "",                 69 })
#define P9_38 ((PIN){   "AIN3",        0,          3,   3,   "",                 70 })
#define P9_39 ((PIN){   "AIN0",        0,          0,   0,   "",                 67 })
#define P9_40 ((PIN){   "AIN1",        0,          1,   1,   "",                 68 })
#define P9_41 ((PIN){   "CLKOUT2",     GPIO0,     20,  20,   "xdma_event_intr1", 13 })
#define P9_42 ((PIN){   "GPIO0_7",     GPIO0,      7,   7,   "ecap0_in_pwm0_out", 4 })


#define INPUT    ((unsigned char)(1))
#define OUTPUT   ((unsigned char)(0))
#define PULLUP   ((unsigned char)(1))
#define PULLDOWN ((unsigned char)(0))
#define PULL_DISABLED ((unsigned char)(2))




#endif
