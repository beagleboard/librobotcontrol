/*******************************************************************************
* Pin Multiplexing (pinmux)
*
* On the Robotics Cape, we allow changing the pinmux on the SPI, GPS, and UART1
* headers in case you wish to expose GPIO, CAN, or PWM functionality.
* We use the GPIO number to identify the pins even though they may be used
* for things other than GPIO as this provides consistency with the GPIO
* functions which will likely be used. A list of defines are also given here
* to make your code easier to read and to indicate which pins are available
* for pinmuxing.
*
* enum pinmux_mode_t gives options for pinmuxing. Not every mode if available on
* each pin. refer to the pin table for which to use. 
*******************************************************************************/

#include "../usefulincludes.h"
#include "../roboticscape.h"
#include "../roboticscape-defs.h"



// P9_11 used for DSM2 radio and not exposed to user
#define P9_11_PATH "/sys/devices/platform/ocp/ocp:P9_11_pinmux/state"
// options from robotics_cape.h
#define P9_22_PATH "/sys/devices/platform/ocp/ocp:P9_22_pinmux/state"
#define P9_21_PATH "/sys/devices/platform/ocp/ocp:P9_21_pinmux/state"
#define P9_26_PATH "/sys/devices/platform/ocp/ocp:P9_26_pinmux/state"
#define P9_24_PATH "/sys/devices/platform/ocp/ocp:P9_24_pinmux/state"
#define P9_30_PATH "/sys/devices/platform/ocp/ocp:P9_30_pinmux/state"
#define P9_29_PATH "/sys/devices/platform/ocp/ocp:P9_29_pinmux/state"
#define P9_31_PATH "/sys/devices/platform/ocp/ocp:P9_31_pinmux/state"
#define P9_28_PATH "/sys/devices/platform/ocp/ocp:P9_28_pinmux/state"





int set_pinmux_mode(int pin, pinmux_mode_t mode){
	int fd, ret;
	char * path;

	// big switch case checks validity of pins and mode
	switch(pin){

	// DSM2 data/pairing pin, for internal use only
	case DSM2_PIN:
		if( mode!=PINMUX_GPIO    	&& \
			mode!=PINMUX_GPIO_PU 	&& \
			mode!=PINMUX_GPIO_PD 	&& \
			mode!=PINMUX_UART){
			printf("ERROR: DSM2 pairing pin can only be put in GPIO or UART mode\n");
			return -1;
		}
		path = P9_11_PATH;
		break;


	// GPS
	case GPS_HEADER_PIN_3:
		if( mode!=PINMUX_GPIO    	&& \
			mode!=PINMUX_GPIO_PU 	&& \
			mode!=PINMUX_GPIO_PD 	&& \
			mode!=PINMUX_PWM	 	&& \
			mode!=PINMUX_I2C 	 	&& \
			mode!=PINMUX_UART){
			printf("ERROR: GPS_HEADER_PIN_3 can only be put in GPIO, UART, PWM, or I2C modes\n");
			return -1;
		}
		path = P9_22_PATH;
		break;

	case GPS_HEADER_PIN_4:
		if( mode!=PINMUX_GPIO    	&& \
			mode!=PINMUX_GPIO_PU 	&& \
			mode!=PINMUX_GPIO_PD 	&& \
			mode!=PINMUX_PWM	 	&& \
			mode!=PINMUX_I2C 	 	&& \
			mode!=PINMUX_UART){
			printf("ERROR: GPS_HEADER_PIN_4 can only be put in GPIO, UART, PWM, or I2C modes\n");
			return -1;
		}
		path = P9_21_PATH;
		break;


	// UART1
	case UART1_HEADER_PIN_3:
		if( mode!=PINMUX_GPIO    	&& \
			mode!=PINMUX_GPIO_PU 	&& \
			mode!=PINMUX_GPIO_PD 	&& \
			mode!=PINMUX_CAN 	 	&& \
			mode!=PINMUX_UART){
			printf("ERROR: UART1_HEADER_PIN_3 can only be put in GPIO, UART, CAN modes\n");
			return -1;
		}
		path = P9_26_PATH;
		break;

	case UART1_HEADER_PIN_4:
		if( mode!=PINMUX_GPIO    	&& \
			mode!=PINMUX_GPIO_PU 	&& \
			mode!=PINMUX_GPIO_PD 	&& \
			mode!=PINMUX_CAN 	 	&& \
			mode!=PINMUX_UART){
			printf("ERROR: UART1_HEADER_PIN_3 can only be put in GPIO, UART, CAN modes\n");
			return -1;
		}
		path = P9_24_PATH;
		break;


	// SPI
	case SPI_HEADER_PIN_3:
		if( mode!=PINMUX_GPIO    	&& \
			mode!=PINMUX_GPIO_PU 	&& \
			mode!=PINMUX_GPIO_PD 	&& \
			mode!=PINMUX_SPI){
			printf("ERROR: SPI_HEADER_PIN_3 can only be put in GPIO, or SPI modes\n");
			return -1;
		}
		path = P9_30_PATH;
		break;
	
	case SPI_HEADER_PIN_4:
		if( mode!=PINMUX_GPIO    	&& \
			mode!=PINMUX_GPIO_PU 	&& \
			mode!=PINMUX_GPIO_PD 	&& \
			mode!=PINMUX_SPI){
			printf("ERROR: SPI_HEADER_PIN_4 can only be put in GPIO, or SPI modes\n");
			return -1;
		}
		path = P9_29_PATH;
		break;
	
	case SPI_HEADER_PIN_5:
		if( mode!=PINMUX_GPIO    	&& \
			mode!=PINMUX_GPIO_PU 	&& \
			mode!=PINMUX_GPIO_PD 	&& \
			mode!=PINMUX_SPI){
			printf("ERROR: SPI_HEADER_PIN_5 can only be put in GPIO, or SPI modes\n");
			return -1;
		}
		path = P9_31_PATH;
		break;

	case SPI_HEADER_PIN_6_SS1:
		if( mode!=PINMUX_GPIO    	&& \
			mode!=PINMUX_GPIO_PU 	&& \
			mode!=PINMUX_GPIO_PD 	&& \
			mode!=PINMUX_SPI){
			printf("ERROR: SPI_HEADER_PIN_6_SS1 can only be put in GPIO, or SPI modes\n");
			return -1;
		}
		path = P9_28_PATH;
		break;

	default:
		printf("ERROR: Pinmuxing on pin %d is not supported\n", pin);
		return -1;
	}


	// check pinmux driver is loaded
	if(access(path, F_OK)!=0){
		printf("ERROR: userspace PINMUX driver not loaded\n");
		printf("check that you are using a kernel newer than 4.4.4-ti-r62\n");
		return -1;
	}

	// open pin state fd
	fd = open(path, O_WRONLY);
	if(fd == -1){
		printf("ERROR: can't open userspace pinmux driver\n");
		return -1;
	}


	switch(mode){
	case PINMUX_GPIO:
		ret = write(fd, "gpio", 4);
		break;
	case PINMUX_GPIO_PU:
		ret = write(fd, "gpio_pu", 7);
		break;
	case PINMUX_GPIO_PD:
		ret = write(fd, "gpio_pd", 7);
		break;
	case PINMUX_PWM:
		ret = write(fd, "pwm", 3);
		break;
	case PINMUX_SPI:
		ret = write(fd, "spi", 3);
		break;
	case PINMUX_UART:
		ret = write(fd, "uart", 4);
		break;
	case PINMUX_CAN:
		ret = write(fd, "can", 3);
		break;
	case PINMUX_I2C:
		ret = write(fd, "i2c", 3);
		break;
	default:
		printf("ERROR: unknown PINMUX mode\n");
		close(fd);
		return -1;
	}

	if(ret<0){
		printf("ERROR: failed to write to pinmux driver\n");
		close(fd);
		return -1;
	}

	close(fd);
	return 0;
}



int set_default_pinmux(){
	int ret = 0;

	// bb blue device tree not done yet, so just one pinmux for now
	if(get_bb_model()==BB_BLUE){
		ret |= set_pinmux_mode(DSM2_PIN, PINMUX_UART);

	}

	// bb black and everything else should use this
	else{
		ret |= set_pinmux_mode(DSM2_PIN, PINMUX_UART);
		ret |= set_pinmux_mode(GPS_HEADER_PIN_3, PINMUX_UART);
		ret |= set_pinmux_mode(GPS_HEADER_PIN_4, PINMUX_UART);
		ret |= set_pinmux_mode(UART1_HEADER_PIN_3, PINMUX_UART);
		ret |= set_pinmux_mode(UART1_HEADER_PIN_4, PINMUX_UART);
		ret |= set_pinmux_mode(SPI_HEADER_PIN_3, PINMUX_SPI);
		ret |= set_pinmux_mode(SPI_HEADER_PIN_4, PINMUX_SPI);
		ret |= set_pinmux_mode(SPI_HEADER_PIN_5, PINMUX_SPI);
		ret |= set_pinmux_mode(SPI_HEADER_PIN_6_SS1, PINMUX_SPI);
	}

	if(ret){
		printf("WARNING: missing PINMUX driver\n");
		printf("You probbaly just need a newer kernel\n");
		return -1;
	}

	return 0;
}