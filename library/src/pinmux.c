/**
 * @file pinmux.c
 *
 * @author	  James Strawson
 * @date		 3/21/2018
 */

#include <errno.h>
#include <stdio.h>
#include <fcntl.h> // for open
#include <unistd.h> // for close
#include <rc/pinmux.h>
#include <rc/model.h>

// preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)
#define likely(x)	__builtin_expect (!!(x), 1)

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
#define H18_PATH "/sys/devices/platform/ocp/ocp:H18_pinmux/state"
#define C18_PATH "/sys/devices/platform/ocp/ocp:C18_pinmux/state"
#define U16_PATH "/sys/devices/platform/ocp/ocp:U16_pinmux/state"
#define D13_PATH "/sys/devices/platform/ocp/ocp:D13_pinmux/state"
#define J15_PATH "/sys/devices/platform/ocp/ocp:J15_pinmux/state"
#define H17_PATH "/sys/devices/platform/ocp/ocp:H17_pinmux/state"


int rc_pinmux_set(int pin, rc_pinmux_mode_t mode)
{
	int fd, ret;
	char* path;

	// flag set when parsing pin switch case
	int blue_only = 0;

	// big switch case checks validity of pins and mode
	switch(pin){

	/***********************************************************************
	* Cape and Blue pins
	***********************************************************************/
	// DSM2 data/pairing pin, for internal use only
	case DSM_HEADER_PIN:
		if(	mode!=PINMUX_GPIO	&& \
			mode!=PINMUX_GPIO_PU	&& \
			mode!=PINMUX_GPIO_PD	&& \
			mode!=PINMUX_UART){
			fprintf(stderr,"ERROR in rc_pinmux_set, DSM pairing pin can only be put in GPIO or UART mode\n");
			return -1;
		}
		path = P9_11_PATH;
		break;


	// GPS
	case GPS_HEADER_PIN_3:
		if(	mode!=PINMUX_GPIO	&& \
			mode!=PINMUX_GPIO_PU	&& \
			mode!=PINMUX_GPIO_PD	&& \
			mode!=PINMUX_PWM	&& \
			mode!=PINMUX_UART){
			fprintf(stderr,"ERROR in rc_pinmux_set, GPS_HEADER_PIN_3 can only be put in GPIO, UART, or PWM modes\n");
			return -1;
		}
		path = P9_22_PATH;
		break;

	case GPS_HEADER_PIN_4:
		if(	mode!=PINMUX_GPIO	&& \
			mode!=PINMUX_GPIO_PU	&& \
			mode!=PINMUX_GPIO_PD	&& \
			mode!=PINMUX_PWM	&& \
			mode!=PINMUX_UART){
			fprintf(stderr,"ERROR in rc_pinmux_set, GPS_HEADER_PIN_4 can only be put in GPIO, UART, or PWM modes\n");
			return -1;
		}
		path = P9_21_PATH;
		break;

	// UART1
	case UART1_HEADER_PIN_3:
		if(	mode!=PINMUX_GPIO	&& \
			mode!=PINMUX_GPIO_PU	&& \
			mode!=PINMUX_GPIO_PD	&& \
			mode!=PINMUX_CAN	&& \
			mode!=PINMUX_UART){
			fprintf(stderr,"ERROR in rc_pinmux_set, UART1_HEADER_PIN_3 can only be put in GPIO, UART, CAN modes\n");
			return -1;
		}
		path = P9_26_PATH;
		break;

	case UART1_HEADER_PIN_4:
		if(	mode!=PINMUX_GPIO	&& \
			mode!=PINMUX_GPIO_PU	&& \
			mode!=PINMUX_GPIO_PD	&& \
			mode!=PINMUX_CAN	&& \
			mode!=PINMUX_UART){
			fprintf(stderr,"ERROR in rc_pinmux_set, UART1_HEADER_PIN_3 can only be put in GPIO, UART, CAN modes\n");
			return -1;
		}
		path = P9_24_PATH;
		break;


	// SPI
	case SPI_HEADER_PIN_3:
		if(	mode!=PINMUX_GPIO	&& \
			mode!=PINMUX_GPIO_PU	&& \
			mode!=PINMUX_GPIO_PD	&& \
			mode!=PINMUX_SPI){
			fprintf(stderr,"ERROR in rc_pinmux_set, SPI_HEADER_PIN_3 can only be put in GPIO, or SPI modes\n");
			return -1;
		}
		path = P9_30_PATH;
		break;

	case SPI_HEADER_PIN_4:
		if(	mode!=PINMUX_GPIO	&& \
			mode!=PINMUX_GPIO_PU	&& \
			mode!=PINMUX_GPIO_PD	&& \
			mode!=PINMUX_SPI){
			fprintf(stderr,"ERROR in rc_pinmux_set, SPI_HEADER_PIN_4 can only be put in GPIO, or SPI modes\n");
			return -1;
		}
		path = P9_29_PATH;
		break;

	case SPI_HEADER_PIN_5:
		if(	mode!=PINMUX_GPIO	&& \
			mode!=PINMUX_GPIO_PU	&& \
			mode!=PINMUX_GPIO_PD	&& \
			mode!=PINMUX_SPI){
			fprintf(stderr,"ERROR in rc_pinmux_set, SPI_HEADER_PIN_5 can only be put in GPIO, or SPI modes\n");
			return -1;
		}
		path = P9_31_PATH;
		break;

	/***************************************************************************
	* CAPE_SPI_PIN_6_SS1 is the same as BLUE_GP0_PIN_6
	***************************************************************************/
	case CAPE_SPI_PIN_6_SS1:
		if(	mode!=PINMUX_GPIO	&& \
			mode!=PINMUX_GPIO_PU	&& \
			mode!=PINMUX_GPIO_PD	&& \
			mode!=PINMUX_SPI){
			if(rc_model()==MODEL_BB_BLUE){
				fprintf(stderr,"ERROR in rc_pinmux_set, BLUE_GP0_PIN_6 can only be put in GPIO modes\n");
			}
			else{
				fprintf(stderr,"ERROR in rc_pinmux_set, SPI_HEADER_PIN_6_SS1 can only be put in GPIO or SPI modes\n");
			}
			return -1;
		}
		path = P9_28_PATH;
		break;

	/***************************************************************************
	* CAPE_SPI_PIN_6_SS2  is the same as BLUE_GP0_PIN_4:
	***************************************************************************/
	case CAPE_SPI_PIN_6_SS2:
		if(	mode!=PINMUX_GPIO	&& \
			mode!=PINMUX_GPIO_PU	&& \
			mode!=PINMUX_GPIO_PD){
			if(rc_model()==MODEL_BB_BLUE){
				fprintf(stderr,"ERROR in rc_pinmux_set, BLUE_GP0_PIN_4 can only be put in GPIO modes\n");
			}
			else{
				fprintf(stderr,"ERROR in rc_pinmux_set, SPI_HEADER_PIN_6_SS2 can only be put in GPIO modes\n");
			}
			return -1;
		}
		path = P9_23_PATH;
		break;

	/***************************************************************************
	* Blue Only
	***************************************************************************/
	case BLUE_SPI_PIN_6_SS1:
		if(	mode!=PINMUX_GPIO	&& \
			mode!=PINMUX_GPIO_PU	&& \
			mode!=PINMUX_GPIO_PD	&& \
			mode!=PINMUX_SPI){
			fprintf(stderr,"ERROR in rc_pinmux_set, BLUE_SPI_PIN_6_SS1 can only be put in GPIO, or SPI modes\n");
			return -1;
		}
		path = H18_PATH;
		blue_only = 1;
		break;

	case BLUE_SPI_PIN_6_SS2:
		if(	mode!=PINMUX_GPIO	&& \
			mode!=PINMUX_GPIO_PU	&& \
			mode!=PINMUX_GPIO_PD	&& \
			mode!=PINMUX_SPI){
			fprintf(stderr,"ERROR in rc_pinmux_set, BLUE_SPI_PIN_6_SS2 can only be put in GPIO, or SPI modes\n");
			return -1;
		}
		path = C18_PATH;
		blue_only = 1;
		break;

	case BLUE_GP0_PIN_3:
		if(	mode!=PINMUX_GPIO	&& \
			mode!=PINMUX_GPIO_PU	&& \
			mode!=PINMUX_GPIO_PD){
			fprintf(stderr,"ERROR in rc_pinmux_set, BLUE_GP0_PIN_3 can only be put in GPIO modes\n");
			return -1;
		}
		path = U16_PATH;
		blue_only = 1;
		break;

	// BLUE_GP0_PIN_4 defined above with cape pins since it is shared

	case BLUE_GP0_PIN_5:
		if(	mode!=PINMUX_GPIO	&& \
			mode!=PINMUX_GPIO_PU	&& \
			mode!=PINMUX_GPIO_PD){
			fprintf(stderr,"ERROR in rc_pinmux_set, BLUE_GP0_PIN_5 can only be put in GPIO modes\n");
			return -1;
		}
		path = D13_PATH;
		blue_only = 1;
		break;

	// BLUE_GP0_PIN_6 defined above with cape pins since it is shared

	case BLUE_GP1_PIN_3:
		if(	mode!=PINMUX_GPIO	&& \
			mode!=PINMUX_GPIO_PU	&& \
			mode!=PINMUX_GPIO_PD){
			fprintf(stderr,"ERROR in rc_pinmux_set, BLUE_GP1_PIN_3 can only be put in GPIO modes\n");
			return -1;
		}
		path = J15_PATH;
		blue_only = 1;
		break;

	case BLUE_GP1_PIN_4:
		if(	mode!=PINMUX_GPIO	&& \
			mode!=PINMUX_GPIO_PU	&& \
			mode!=PINMUX_GPIO_PD){
			fprintf(stderr,"ERROR in rc_pinmux_set, BLUE_GP1_PIN_4 can only be put in GPIO modes\n");
			return -1;
		}
		path = H17_PATH;
		blue_only = 1;
		break;


	/***************************************************************************
	* Phew, end of long switch case. Print error if pin not supported.
	***************************************************************************/
	default:
		fprintf(stderr,"ERROR in rc_pinmux_set, Pinmuxing on pin %d is not supported\n", pin);
		return -1;
	}

	// check for board incompatibility
	if(blue_only && rc_model()!=MODEL_BB_BLUE){
		fprintf(stderr,"ERROR in rc_pinmux_set, Trying to set pinmux on pin that should only used on BB Blue\n");
		return -1;
	}

	// open pin state fd
	errno=0;
	fd = open(path, O_WRONLY);
	if(unlikely(fd==-1)){
		perror("ERORR opening pinmux driver");
		fprintf(stderr,"can't open: %s\n",path);
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
		fprintf(stderr,"ERROR in rc_pinmux_set, unknown PINMUX mode\n");
		close(fd);
		return -1;
	}

	if(ret<0){
		perror("ERROR in rc_pinmux_set, failed to write to pinmux driver");
		close(fd);
		return -1;
	}

	close(fd);
	return 0;
}


int rc_pinmux_set_default(void)
{
	int ret = 0;
	// bb blue available pinmux
	if(rc_model()==MODEL_BB_BLUE){
		ret |= rc_pinmux_set(BLUE_SPI_PIN_6_SS1, PINMUX_SPI);
		ret |= rc_pinmux_set(BLUE_SPI_PIN_6_SS2, PINMUX_SPI);
		ret |= rc_pinmux_set(BLUE_GP0_PIN_3, PINMUX_GPIO_PU);
		ret |= rc_pinmux_set(BLUE_GP0_PIN_4, PINMUX_GPIO_PU);
		ret |= rc_pinmux_set(BLUE_GP0_PIN_5, PINMUX_GPIO_PU);
		ret |= rc_pinmux_set(BLUE_GP0_PIN_6, PINMUX_GPIO_PU);
		ret |= rc_pinmux_set(BLUE_GP1_PIN_3, PINMUX_GPIO_PU);
		ret |= rc_pinmux_set(BLUE_GP1_PIN_4, PINMUX_GPIO_PU);
	}
	// bb black and everything else
	else{
		ret |= rc_pinmux_set(CAPE_SPI_PIN_6_SS1, PINMUX_SPI);
		ret |= rc_pinmux_set(CAPE_SPI_PIN_6_SS2, PINMUX_GPIO);
	}
	// shared pins
	ret |= rc_pinmux_set(DSM_HEADER_PIN, PINMUX_UART);
	ret |= rc_pinmux_set(GPS_HEADER_PIN_3, PINMUX_UART);
	ret |= rc_pinmux_set(GPS_HEADER_PIN_4, PINMUX_UART);
	ret |= rc_pinmux_set(UART1_HEADER_PIN_3, PINMUX_UART);
	ret |= rc_pinmux_set(UART1_HEADER_PIN_4, PINMUX_UART);
	ret |= rc_pinmux_set(SPI_HEADER_PIN_3, PINMUX_SPI);
	ret |= rc_pinmux_set(SPI_HEADER_PIN_4, PINMUX_SPI);
	ret |= rc_pinmux_set(SPI_HEADER_PIN_5, PINMUX_SPI);

	if(ret){
		fprintf(stderr,"ERROR in rc_pinmux_set_default\n");
		printf("You probbaly just need a newer kernel\n");
		return -1;
	}
	return 0;
}
