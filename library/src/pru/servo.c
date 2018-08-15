/**
 * @file servo.c
 *
 * @author     James Strawson
 * @date       3/7/2018
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <math.h> //for lround
#include <rc/pru.h>
#include <rc/gpio.h>
#include <rc/servo.h>
#include <rc/time.h>

#define TOL		0.01	// acceptable tolerance on doubleing point bounds
#define GPIO_POWER_PIN	2,16	//gpio2.16 P8.36
#define SERVO_PRU_CH	1	// PRU1
#define SERVO_PRU_FW	"am335x-pru1-rc-servo-fw"
#define PRU_SERVO_LOOP_INSTRUCTIONS 48 // instructions per PRU servo timer loop

// pru shared memory pointer
static volatile unsigned int* shared_mem_32bit_ptr = NULL;
static int init_flag=0;

static int esc_max_us =  RC_ESC_DEFAULT_MAX_US;
static int esc_min_us =  RC_ESC_DEFAULT_MIN_US;

int rc_servo_init(void)
{
	int i;
	// start gpio power rail pin
	if(rc_gpio_init(GPIO_POWER_PIN, GPIOHANDLE_REQUEST_OUTPUT)==-1){
		fprintf(stderr, "ERROR in rc_servo_init, failed to set up power rail GPIO pin\n");
		init_flag=0;
		return -1;
	}
	// map memory
	shared_mem_32bit_ptr = rc_pru_shared_mem_ptr();
	if(shared_mem_32bit_ptr == NULL){
		fprintf(stderr, "ERROR in rc_servo_init, failed to map shared memory pointer\n");
		init_flag=0;
		return -1;
	}
	// set channels to be nonzero, PRU binary will zero this out later
	for(i=RC_SERVO_CH_MIN;i<=RC_SERVO_CH_MAX;i++){
		// write to PRU shared memory
		shared_mem_32bit_ptr[i-1] = 42;
	}

	// start pru
	if(rc_pru_start(SERVO_PRU_CH, SERVO_PRU_FW)){
		fprintf(stderr,"ERROR in rc_servo_init, failed to start PRU%d\n", SERVO_PRU_CH);
		return -1;
	}

	// make sure memory actually got zero'd out
	for(i=0;i<40;i++){
		if(shared_mem_32bit_ptr[0]==0){
			init_flag=1;
			return 0;
		}
		rc_usleep(100000);
	}

	fprintf(stderr, "ERROR in rc_servo_init, %s failed to load\n", SERVO_PRU_FW);
	fprintf(stderr, "attempting to stop PRU1\n");
	rc_pru_stop(SERVO_PRU_CH);
	init_flag=0;
	return -1;
}


void rc_servo_cleanup(void)
{
	int i;
	// zero out shared memory
	if(shared_mem_32bit_ptr != NULL){
		for(i=0;i<RC_SERVO_CH_MAX;i++) shared_mem_32bit_ptr[i]=0;
	}
	if(init_flag!=0){
		rc_gpio_set_value(GPIO_POWER_PIN,0);
		rc_gpio_cleanup(GPIO_POWER_PIN);
	}
	rc_pru_stop(SERVO_PRU_CH);
	shared_mem_32bit_ptr = NULL;
	init_flag=0;
	return;
}


int rc_servo_power_rail_en(int en)
{
	if(init_flag==0){
		fprintf(stderr, "ERROR in rc_servo_power_rail_en, call rc_servo_init first\n");
		return -1;
	}
	if(rc_gpio_set_value(GPIO_POWER_PIN,en)==-1){
		fprintf(stderr, "ERROR in rc_servo_power_rail_en, failed to write to GPIO pin\n");
		return -1;
	}
	return 0;
}

int rc_servo_set_esc_range(int min, int max)
{
	if(min<1 || max<2){
		fprintf(stderr, "ERROR in rc_servo_set_esc_range, in and max values must be positive\n");
		return -1;
	}
	if(min>=max){
		fprintf(stderr, "ERROR in rc_servo_set_esc_range. max must be greater than min\n");
		return -1;
	}
	esc_min_us = min;
	esc_max_us = max;
	return 0;
}


int rc_servo_send_pulse_us(int ch, int us)
{
	int i, ret;
	uint32_t num_loops;
	// Sanity Checks
	if(ch<0 || ch>RC_SERVO_CH_MAX){
		fprintf(stderr,"ERROR: in rc_servo_send_pulse_us, channel argument must be between 0&%d\n", RC_SERVO_CH_MAX);
		return -1;
	}
	if(init_flag==0){
		fprintf(stderr,"ERROR: in rc_servo_send_pulse_us, call rc_servo_init first\n");
		return -1;
	}


	// calculate what to write to pru shared memory to set pulse width
	num_loops = ((us*200.0)/PRU_SERVO_LOOP_INSTRUCTIONS);

	// for single channel requests, write once
	if(ch!=0){
		// first check to make sure no pulse is currently being sent
		if(shared_mem_32bit_ptr[ch-1] != 0){
			fprintf(stderr,"ERROR: in rc_servo_send_pulse_us, tried to start a new pulse amidst another\n");
			fprintf(stderr,"PRU may need more time to start up before sending pulses\n");;
			return -1;
		}
		// write to PRU shared memory
		shared_mem_32bit_ptr[ch-1] = num_loops;
		return 0;
	}

	// if all channels are requested, loop through them
	ret=0;
	for(i=RC_SERVO_CH_MIN;i<=RC_SERVO_CH_MAX;i++){
		// first check to make sure no pulse is currently being sent
		if(shared_mem_32bit_ptr[i-1] != 0){
			fprintf(stderr,"ERROR: in rc_servo_send_pulse_us, tried to start a new pulse amidst another on channel %d\n", i);
			fprintf(stderr,"current val:%d\n", shared_mem_32bit_ptr[i-1]);
			fprintf(stderr,"this either means you are sending pulses too fast, or the PRU binary didn't load properly\n");
			ret = -1;
			continue;
		}
		// write to PRU shared memory
		shared_mem_32bit_ptr[i-1] = num_loops;
	}
	return ret;
}


int rc_servo_send_pulse_normalized(int ch, double input)
{
	int us;
	if(input<(-1.5-TOL) || input>(1.5+TOL)){
		fprintf(stderr,"ERROR in rc_servo_send_pulse_normalized, normalized input must be between -1.5 & 1.5\n");
		return -1;
	}
	// normal range is from 900 to 2100 for 120 degree servos
	us = 1500 + lround((input*600.0));
	return rc_servo_send_pulse_us(ch, us);
}



int rc_servo_send_esc_pulse_normalized(int ch, double input)
{
	int us;
	if(input<(-0.1-TOL) || input>(1.0+TOL)){
		fprintf(stderr,"ERROR in rc_servo_send_esc_pulse_normalized, normalized input must be between -0.1 & 1.0\n");
		return -1;
	}
	us = esc_min_us + lround((input*(esc_max_us-esc_min_us)));
	return rc_servo_send_pulse_us(ch, us);
}



int rc_servo_send_oneshot_pulse_normalized(int ch, double input)
{
	int us;
	if(input<(-0.1-TOL) || input>(1.0+TOL)){
		fprintf(stderr,"ERROR in rc_servo_send_oneshot_pulse_normalized, normalized input must be between -0.1 & 1.0\n");
		return -1;
	}
	us = 125 + lround(input*125.0);
	return rc_servo_send_pulse_us(ch, us);
}
