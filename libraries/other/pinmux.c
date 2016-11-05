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

#include "../roboticscape-usefulincludes.h"
#include "../roboticscape.h"
#include "../roboticscape-defs.h"



// P9_11 used for DSM2 radio and not exposed to user
#define P9_11_PATH "/sys/devices/platform/ocp/ocp:P9_11_pinmux/state"

// cape and blue
#define P9_22_PATH "/sys/devices/platform/ocp/ocp:P9_22_pinmux/state"
#define P9_21_PATH "/sys/devices/platform/ocp/ocp:P9_21_pinmux/state"
#define P9_26_PATH "/sys/devices/platform/ocp/ocp:P9_26_pinmux/state"
#define P9_24_PATH "/sys/devices/platform/ocp/ocp:P9_24_pinmux/state"
#define P9_30_PATH "/sys/devices/platform/ocp/ocp:P9_30_pinmux/state"
#define P9_29_PATH "/sys/devices/platform/ocp/ocp:P9_29_pinmux/state"
#define P9_31_PATH "/sys/devices/platform/ocp/ocp:P9_31_pinmux/state"
#define P9_28_PATH "/sys/devices/platform/ocp/ocp:P9_28_pinmux/state"
#define P9_23_PATH "/sys/devices/platform/ocp/ocp:P9_23_pinmux/state"

// Blue Only
#define E17_PATH "/sys/devices/platform/ocp/ocp:E17_pinmux/state"
#define A15_PATH "/sys/devices/platform/ocp/ocp:A15_pinmux/state"
#define U16_PATH "/sys/devices/platform/ocp/ocp:U16_pinmux/state"
#define D13_PATH "/sys/devices/platform/ocp/ocp:D13_pinmux/state"
#define J15_PATH "/sys/devices/platform/ocp/ocp:J15_pinmux/state"
#define H17_PATH "/sys/devices/platform/ocp/ocp:H17_pinmux/state"



/*******************************************************************************
* int set_pinmux_mode(int pin, pinmux_mode_t mode)
*
* checks the desired pin and mode for validity depending on if you are running
* a blue or cape. Returns -1 on failure, 0 on success.
*******************************************************************************/
int set_pinmux_mode(int pin, pinmux_mode_t mode){
	int fd, ret;
	char* path;

	// flag set when parsing pin switch case
	int blue_only = 0;

	// big switch case checks validity of pins and mode
	switch(pin){

	/***************************************************************************
	* Cape and Blue pins
	***************************************************************************/
	// DSM2 data/pairing pin, for internal use only
	case DSM_PIN:
		if( mode!=PINMUX_GPIO    	&& \
			mode!=PINMUX_GPIO_PU 	&& \
			mode!=PINMUX_GPIO_PD 	&& \
			mode!=PINMUX_UART){
			printf("ERROR: DSM pairing pin can only be put in GPIO or UART mode\n");
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
			mode!=PINMUX_UART){
			printf("ERROR: GPS_HEADER_PIN_3 can only be put in GPIO, UART, or PWM modes\n");
			return -1;
		}
		path = P9_22_PATH;
		break;

	case GPS_HEADER_PIN_4:
		if( mode!=PINMUX_GPIO    	&& \
			mode!=PINMUX_GPIO_PU 	&& \
			mode!=PINMUX_GPIO_PD 	&& \
			mode!=PINMUX_PWM	 	&& \
			mode!=PINMUX_UART){
			printf("ERROR: GPS_HEADER_PIN_4 can only be put in GPIO, UART, or PWM modes\n");
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

	/***************************************************************************
	* SPI_HEADER_PIN_6_SS1 is the same as BLUE_GP0_PIN_6
	***************************************************************************/
	case SPI_HEADER_PIN_6_SS1:
		if( mode!=PINMUX_GPIO    	&& \
			mode!=PINMUX_GPIO_PU 	&& \
			mode!=PINMUX_GPIO_PD 	&& \
			mode!=PINMUX_SPI){
			if(get_bb_model()==BB_BLUE){
				printf("ERROR: BLUE_GP0_PIN_6 can only be put in GPIO modes\n");
			}
			else{
				printf("ERROR: SPI_HEADER_PIN_6_SS1 can only be put in GPIO or SPI modes\n");
			}
			return -1;
		}
		path = P9_28_PATH;
		break;

	/***************************************************************************
	* SPI_HEADER_PIN_6_SS2  is the same as BLUE_GP0_PIN_4:
	***************************************************************************/
	case SPI_HEADER_PIN_6_SS2:
		if( mode!=PINMUX_GPIO    	&& \
			mode!=PINMUX_GPIO_PU 	&& \
			mode!=PINMUX_GPIO_PD ){
			if(get_bb_model()==BB_BLUE){
				printf("ERROR: BLUE_GP0_PIN_4 can only be put in GPIO modes\n");
			}
			else{
				printf("ERROR: SPI_HEADER_PIN_6_SS2 can only be put in GPIO modes\n");
			}
			return -1;
		}
		path = P9_23_PATH;
		break;

	/***************************************************************************
	* Blue Only
	***************************************************************************/
	case BLUE_SPI_PIN_6_SS1:
		if( mode!=PINMUX_GPIO    	&& \
			mode!=PINMUX_GPIO_PU 	&& \
			mode!=PINMUX_GPIO_PD 	&& \
			mode!=PINMUX_SPI ){
			printf("ERROR: BLUE_SPI_PIN_6_SS1 can only be put in GPIO, or SPI modes\n");
			return -1;
		}
		path = E17_PATH;
		blue_only = 1;
		break;

	case BLUE_SPI_PIN_6_SS2:
		if( mode!=PINMUX_GPIO    	&& \
			mode!=PINMUX_GPIO_PU 	&& \
			mode!=PINMUX_GPIO_PD 	&& \
			mode!=PINMUX_SPI ){
			printf("ERROR: BLUE_SPI_PIN_6_SS2 can only be put in GPIO, or SPI modes\n");
			return -1;
		}
		path = A15_PATH;
		blue_only = 1;
		break;

	case BLUE_GP0_PIN_3:
		if( mode!=PINMUX_GPIO    	&& \
			mode!=PINMUX_GPIO_PU 	&& \
			mode!=PINMUX_GPIO_PD ){
			printf("ERROR: BLUE_GP0_PIN_3 can only be put in GPIO modes\n");
			return -1;
		}
		path = U16_PATH;
		blue_only = 1;
		break;

	// BLUE_GP0_PIN_4 defined above with cape pins since it is shared

	case BLUE_GP0_PIN_5:
		if( mode!=PINMUX_GPIO    	&& \
			mode!=PINMUX_GPIO_PU 	&& \
			mode!=PINMUX_GPIO_PD ){
			printf("ERROR: BLUE_GP0_PIN_5 can only be put in GPIO modes\n");
			return -1;
		}
		path = D13_PATH;
		blue_only = 1;
		break;

	// BLUE_GP0_PIN_6 defined above with cape pins since it is shared

	case BLUE_GP1_PIN_3:
		if( mode!=PINMUX_GPIO    	&& \
			mode!=PINMUX_GPIO_PU 	&& \
			mode!=PINMUX_GPIO_PD ){
			printf("ERROR: BLUE_GP1_PIN_3 can only be put in GPIO modes\n");
			return -1;
		}
		path = J15_PATH;
		blue_only = 1;
		break;

	case BLUE_GP1_PIN_4:
		if( mode!=PINMUX_GPIO    	&& \
			mode!=PINMUX_GPIO_PU 	&& \
			mode!=PINMUX_GPIO_PD ){
			printf("ERROR: BLUE_GP1_PIN_4 can only be put in GPIO modes\n");
			return -1;
		}
		path = H17_PATH;
		blue_only = 1;
		break;


	/***************************************************************************
	* Phew, end of long switch case. Print error if pin not supported.
	***************************************************************************/
	default:
		printf("ERROR: Pinmuxing on pin %d is not supported\n", pin);
		return -1;
	}


	// check for board incompatibility
	if(blue_only && get_bb_model()!=BB_BLUE){
		printf("ERROR: Trying to set pinmux on pin that should only used on BB Blue\n");
		return -1;
	}

	// open pin state fd
	fd = open(path, O_WRONLY);
	if(fd == -1){
		printf("can't open: ");
		printf(path);
		printf("\n");
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



/*******************************************************************************
* int set_default_pinmux()
*
* puts everything back to standard and is used by initialize_cape
*******************************************************************************/
int set_default_pinmux(){
	int ret = 0;

	// bb blue available pinmux
	if(get_bb_model()==BB_BLUE){
		ret |= set_pinmux_mode(DSM2_PIN, PINMUX_UART);
		ret |= set_pinmux_mode(GPS_HEADER_PIN_3, PINMUX_UART);
		ret |= set_pinmux_mode(GPS_HEADER_PIN_4, PINMUX_UART);
		ret |= set_pinmux_mode(UART1_HEADER_PIN_3, PINMUX_UART);
		ret |= set_pinmux_mode(UART1_HEADER_PIN_4, PINMUX_UART);
		ret |= set_pinmux_mode(SPI_HEADER_PIN_3, PINMUX_SPI);
		ret |= set_pinmux_mode(SPI_HEADER_PIN_4, PINMUX_SPI);
		ret |= set_pinmux_mode(SPI_HEADER_PIN_5, PINMUX_SPI);
		ret |= set_pinmux_mode(BLUE_SPI_PIN_6_SS1, PINMUX_SPI);
		ret |= set_pinmux_mode(BLUE_SPI_PIN_6_SS2, PINMUX_SPI);
		ret |= set_pinmux_mode(BLUE_GP0_PIN_3, PINMUX_GPIO_PU);
		ret |= set_pinmux_mode(BLUE_GP0_PIN_4, PINMUX_GPIO_PU);
		ret |= set_pinmux_mode(BLUE_GP0_PIN_5, PINMUX_GPIO_PU);
		ret |= set_pinmux_mode(BLUE_GP0_PIN_6, PINMUX_GPIO_PU);
		ret |= set_pinmux_mode(BLUE_GP1_PIN_3, PINMUX_GPIO_PU);
		ret |= set_pinmux_mode(BLUE_GP1_PIN_4, PINMUX_GPIO_PU);

	}

	// bb black and everything else
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
		ret |= set_pinmux_mode(SPI_HEADER_PIN_6_SS2, PINMUX_GPIO);
	}

	if(ret){
		printf("WARNING: missing PINMUX driver\n");
		printf("You probbaly just need a newer kernel\n");
		return -1;
	}

	return 0;
}