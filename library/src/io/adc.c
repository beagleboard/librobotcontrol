/**
 * @file adc_bbb_iio.c
 *
 * Basic interface to the iio adc driver
 *
 * @author     James Strawson
 * @date       1/29/2018
 */

#include <stdio.h>
#include <stdlib.h> // for atoi
#include <errno.h>
#include <fcntl.h> // for open
#include <unistd.h> // for close
#include <rc/adc.h>

// preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)

#define DC_JACK_OFFSET	-0.15
#define LIPO_OFFSET	-0.10
#define LIPO_ADC_CH	6
#define DC_JACK_ADC_CH	5
#define V_DIV_RATIO	11.0
#define BATT_DEADZONE	1.0


#define CHANNELS 8
#define IIO_DIR "/sys/bus/iio/devices/iio:device0"
#define RAW_MAX 4095
#define RAW_MIN 0
#define MAX_BUF 64

static int init_flag = 0; // boolean to check if mem mapped
static int fd[CHANNELS]; // file descriptors for 8 channels


int rc_adc_init(void)
{
	char buf[MAX_BUF];
	int i, temp_fd;
	if(init_flag) return 0;

	for(i=0;i<CHANNELS;i++){
		snprintf(buf, sizeof(buf), IIO_DIR "/in_voltage%d_raw", i);
		temp_fd = open(buf, O_RDONLY);
		if(temp_fd<0){
			perror("ERROR in rc_adc_init, failed to open iio adc interface\n");
			fprintf(stderr,"Perhaps kernel or device tree is too old\n");
			return -1;
		}
		fd[i]=temp_fd;
	}
	init_flag = 1;
	return 0;
}

int rc_adc_cleanup(void)
{
	int i;
	for(i=0;i<CHANNELS;i++){
		close(fd[i]);
	}
	init_flag = 0;
	return 0;
}


int rc_adc_read_raw(int ch)
{
	char buf[8];
	int i;
	//sanity checks
	if(unlikely(!init_flag)){
		fprintf(stderr,"ERROR in rc_adc_read_raw, please initialize with rc_adc_init() first\n");
		return -1;
	}
	if(unlikely(ch<0 || ch>=CHANNELS)){
		fprintf(stderr,"ERROR: in rc_adc_read_raw, adc channel must be between 0 & %d\n", CHANNELS-1);
		return -1;
	}
	if(unlikely(lseek(fd[ch],0,SEEK_SET)<0)){
		perror("ERROR: in rc_adc_read_raw, failed to seek to beginning of FD");
		return -1;
	}
	if(unlikely(read(fd[ch], buf, sizeof(buf))==-1)){
		perror("ERROR in rc_adc_read_raw, can't read iio adc fd");
		return -1;
	}
	i=atoi(buf);
	if(i>RAW_MAX || i< RAW_MIN){
		fprintf(stderr, "ERROR: in rc_adc_read_raw, value out of bounds: %d\n", i);
		return -1;
	}
	return i;
}


double rc_adc_read_volt(int ch)
{
	int raw = rc_adc_read_raw(ch);
	if(raw<0) return -1;
	return raw * 1.8 / 4095.0;
}


double rc_adc_batt(void)
{
	double v = (rc_adc_read_volt(LIPO_ADC_CH)*V_DIV_RATIO)+LIPO_OFFSET;
	// add in a little dead zone to make disconnected battery easier to detect
	if(v<BATT_DEADZONE) v = 0.0;
	return v;
}


double rc_adc_dc_jack(void)
{
	double v = (rc_adc_read_volt(DC_JACK_ADC_CH)*V_DIV_RATIO)+DC_JACK_OFFSET;
	if(v<BATT_DEADZONE) v = 0.0;
	return v;
}




