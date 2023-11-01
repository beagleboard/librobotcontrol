/**
 * @file motor.c
 * @author James Strawson
 * @date 2018
 */

#include <stdio.h>
#include <rc/motor.h>
#include <rc/model.h>
#include <rc/gpio.h>
#include <rc/pwm.h>

// preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)


/*
$ cat /proc/device-tree/chosen/overlays/ROBOTICS-CAPE-GATEWARE
BVF-0.3.0-7-gfd28a2c
$ gpioinfo | grep -E '"P|chip'
gpiochip0 - 14 lines:
gpiochip1 - 24 lines:
gpiochip2 - 32 lines:
	line   0: "P8_PIN3_USER_LED_0" unused output active-high
	line   1: "P8_PIN4_USER_LED_1" unused input active-high
	line   2: "P8_PIN5_USER_LED_2" unused output active-high
	line   3: "P8_PIN6_USER_LED_3" unused output active-high
	line   4: "P8_PIN7_USER_LED_4" unused output active-high
	line   5: "P8_PIN8_USER_LED_5" unused output active-high
	line   6: "P8_PIN9_USER_LED_6" unused input active-high
	line   7: "P8_PIN10_USER_LED_7" unused input active-high
	line   8: "P8_PIN11_USER_LED_8" unused output active-high
	line   9: "P8_PIN12_USER_LED_9" unused output active-high
	line  10: "P8_PIN13_USER_LED_10" unused output active-high
	line  11: "P8_PIN14_USER_LED_11" unused output active-high
	line  12:   "P8_PIN15"       unused  output  active-high
	line  13:   "P8_PIN16"       unused  output  active-high
	line  14:   "P8_PIN17"       unused  output  active-high
	line  15:   "P8_PIN18"       unused  output  active-high
	line  16:   "P8_PIN19"       unused  output  active-high
	line  17:   "P8_PIN20"       unused  output  active-high
	line  18:   "P8_PIN21"       unused  output  active-high
	line  19:   "P8_PIN22"       unused  output  active-high
	line  20:   "P8_PIN23"       unused  output  active-high
	line  21:   "P8_PIN24"       unused  output  active-high
	line  22:   "P8_PIN25"       unused  output  active-high
	line  23:   "P8_PIN26"       unused  output  active-high
	line  24:   "P8_PIN27"       unused  output  active-high
	line  25:   "P8_PIN28"       unused  output  active-high
	line  26:   "P8_PIN29"       unused  output  active-high
	line  27:   "P8_PIN30"       unused  output  active-high
gpiochip3 - 21 lines:
	line   0:      "P9_12"       unused  output  active-high
	line   1:      "P9_15"       unused  output  active-high
	line   2:      "P9_23"       unused  output  active-high
	line   3:      "P9_25"       unused  output  active-high
	line   4:      "P9_30"       unused  output  active-high
	line   5:      "P9_41"       unused  output  active-high
	line   6:      "P9_13"       unused  output  active-high
gpiochip4 - 16 lines:
	line   0:      "P8_31"       unused  output  active-high
	line   1:      "P8_32"       unused  output  active-high
	line   3:      "P8_34"       unused  output  active-high
	line   5:      "P8_36"       unused  output  active-high
	line   6:      "P8_37"       unused  output  active-high
	line   7:      "P8_38"       unused  output  active-high
	line   8:      "P8_39"       unused  output  active-high
	line   9:      "P8_40"       unused  output  active-high
	line  10:      "P8_41"       unused  output  active-high
	line  11:      "P8_42"       unused  output  active-high
	line  12:      "P8_43"       unused  output  active-high
	line  13:      "P8_44"       unused  output  active-high
	line  14:      "P8_45"       unused  output  active-high
	line  15:      "P8_46"       unused  output  active-high
*/
#define MDIR1A_CHIP_FIRE	3	//P9.12
#define MDIR1A_PIN_FIRE		0
#define MDIR1B_CHIP_FIRE	3	//P9.13
#define MDIR1B_PIN_FIRE		6
#define MDIR2A_CHIP_FIRE	3	//P9.15
#define MDIR2A_PIN_FIRE		1
#define MDIR2B_CHIP_FIRE	4	//P8.34
#define MDIR2B_PIN_FIRE		3
#define MDIR3B_CHIP_FIRE	4	//P8.43
#define MDIR3B_PIN_FIRE		12
#define MDIR3A_CHIP_FIRE	4	//P8.44
#define MDIR3A_PIN_FIRE		13
#define MDIR4A_CHIP_FIRE	4	//P8.45
#define MDIR4A_PIN_FIRE		14
#define MDIR4B_CHIP_FIRE	4	//P8.46
#define MDIR4B_PIN_FIRE		15

#define MOT_STBY_FIRE		4,10	//P9.41/P1.20
// motor pin definitions
#define MDIR1A_CHIP		1	//gpio1.28	P9.12/P2.08
#define MDIR1A_PIN		28	//gpio1.28	P9.12/P2.08
#define MDIR1A_CHIP_BLUE	2	//gpio2.0	pin T13
#define MDIR1A_PIN_BLUE		0	//gpio2.0	pin T13
#define MDIR1B_CHIP		0	//gpio0.31	P9.13/P2.07
#define MDIR1B_PIN		31	//gpio0.31	P9.13/P2.07

#define MDIR2A_CHIP		1	//gpio1.16	P9.15
#define MDIR2A_PIN		16	//gpio1.16	P9.15
#define MDIR2A_CHIP_POCKET	0	//gpio0.26	P1.34
#define MDIR2A_PIN_POCKET	26	//gpio0.26	P1.34
#define MDIR2B_CHIP		2	//gpio2.17	P8.34
#define MDIR2B_PIN		17	//gpio2.17	P8.34
#define MDIR2B_CHIP_BLUE	0	//gpio0.10	P8_31
#define MDIR2B_PIN_BLUE		10	//gpio0.10	P8_31
#define MDIR2B_CHIP_POCKET	1	//gpio1.27	P2.02
#define MDIR2B_PIN_POCKET	27	//gpio1.27	P2.02

#define MDIR3B_CHIP		2	//gpio2.8	P8.43
#define MDIR3B_PIN		8	//gpio2.8	P8.43
#define MDIR3A_CHIP		2	//gpio2.9	P8.44
#define MDIR3A_PIN		9	//gpio2.9	P8.44

#define MDIR4A_CHIP		2	//gpio2.6	P8.45
#define MDIR4A_PIN		6	//gpio2.6	P8.45
#define MDIR4B_CHIP		2	//gpio2.7	P8.46
#define MDIR4B_PIN		7	//gpio2.7	P8.46

#define MOT_STBY		0,20	//gpio0.20	P9.41/P1.20

#define CHANNELS		4
#define CHANNELS_POCKET		2


// polarity of the motor connections
static const double polarity[]={1.0,-1.0,-1.0,1.0};

static int init_flag = 0;
static int stby_state = 0;
static int dirA_chip[CHANNELS];
static int dirA_pin[CHANNELS];
static int dirB_chip[CHANNELS];
static int dirB_pin[CHANNELS];
static int pwmss[CHANNELS];
static int pwmch[CHANNELS];
static int channels = 0;



int rc_motor_init(void)
{
	return rc_motor_init_freq(RC_MOTOR_DEFAULT_PWM_FREQ);
}


int rc_motor_init_freq(int pwm_frequency_hz)
{
	int i;

	if(rc_model()==MODEL_BB_POCKET){
		channels = CHANNELS_POCKET;
	}
	else{
		channels = CHANNELS;
	}

	// set pins for motor 1
	// assign gpio pins for blue/black/pocket
	dirA_chip[0] = MDIR1A_CHIP;
	dirA_pin[0] = MDIR1A_PIN;
	dirB_chip[0]=MDIR1B_CHIP;
	dirB_pin[0]=MDIR1B_PIN;
	pwmss[0]=1;
	pwmch[0]='A';
	if(rc_model()==MODEL_BB_BLUE){
		dirA_chip[0]=MDIR1A_CHIP_BLUE;
		dirA_pin[0]=MDIR1A_PIN_BLUE;
	}
	else if(rc_model()==MODEL_BB_FIRE) {
		dirA_chip[0]=MDIR1A_CHIP_FIRE;
		dirA_pin[0]=MDIR1A_PIN_FIRE;
		dirB_chip[0]=MDIR1B_CHIP_FIRE;
		dirB_pin[0]=MDIR1B_PIN_FIRE;
		pwmss[0]=0;
		pwmch[0]='A';
	}
	// motor 2
	dirA_chip[1]=MDIR2A_CHIP;
	dirA_pin[1]=MDIR2A_PIN;
	dirB_chip[1] = MDIR2B_CHIP;
	dirB_pin[1] = MDIR2B_PIN;
	pwmss[1]=1;
	pwmch[1]='B';
	if(rc_model()==MODEL_BB_POCKET) {
		dirA_chip[1]=MDIR2A_CHIP_POCKET;
		dirA_pin[1]=MDIR2A_PIN_POCKET;
		dirB_chip[1]=MDIR2B_CHIP_POCKET;
		dirB_pin[1]=MDIR2B_PIN_POCKET;
		pwmss[1]=0;
		pwmch[1]='A';
	}
	else if(rc_model()==MODEL_BB_BLUE){
		dirB_chip[1]=MDIR2B_CHIP_BLUE;
		dirB_pin[1]=MDIR2B_PIN_BLUE;
	}
	else if(rc_model()==MODEL_BB_FIRE){
		dirA_chip[1]=MDIR2A_CHIP_FIRE;
		dirA_pin[1]=MDIR2A_PIN_FIRE;
		dirB_chip[1]=MDIR2B_CHIP_FIRE;
		dirB_pin[1]=MDIR2B_PIN_FIRE;
		pwmss[1]=0;
		pwmch[1]='B';
	}

	// motor 3
	if(rc_model()==MODEL_BB_FIRE) {
		dirA_chip[2]=MDIR3A_CHIP_FIRE;
		dirA_pin[2]=MDIR3A_PIN_FIRE;
		dirB_chip[2]=MDIR3B_CHIP_FIRE;
		dirB_pin[2]=MDIR3B_PIN_FIRE;
		pwmss[2]=1;
		pwmch[2]='A';
	}
	else{
		dirA_chip[2]=MDIR3A_CHIP;
		dirA_pin[2]=MDIR3A_PIN;
		dirB_chip[2]=MDIR3B_CHIP;
		dirB_pin[2]=MDIR3B_PIN;
		pwmss[2]=2;
		pwmch[2]='A';
	}
	// motor 4
	if(rc_model()==MODEL_BB_FIRE) {
		dirA_chip[3]=MDIR4A_CHIP_FIRE;
		dirA_pin[3]=MDIR4A_PIN_FIRE;
		dirB_chip[3]=MDIR4B_CHIP_FIRE;
		dirB_pin[3]=MDIR4B_PIN_FIRE;
		pwmss[3]=1;
		pwmch[3]='B';
	}
	else{
		dirA_chip[3]=MDIR4A_CHIP;
		dirA_pin[3]=MDIR4A_PIN;
		dirB_chip[3]=MDIR4B_CHIP;
		dirB_pin[3]=MDIR4B_PIN;
		pwmss[3]=2;
		pwmch[3]='B';
	}

	// set up pwm channels
	if(unlikely(rc_pwm_init(0,pwm_frequency_hz))){
		fprintf(stderr,"ERROR in rc_motor_init, failed to initialize pwm subsystem 1\n");
		return -1;
	}
	if(unlikely(rc_pwm_init(1,pwm_frequency_hz))){
		fprintf(stderr,"ERROR in rc_motor_init, failed to initialize pwm subsystem 1\n");
		return -1;
	}
	if(rc_model()!=MODEL_BB_FIRE) {
		if(unlikely(rc_pwm_init(2,pwm_frequency_hz))){
			fprintf(stderr,"ERROR in rc_motor_init, failed to initialize pwm subsystem 2\n");
			return -1;
		}
	}
	// set up gpio pins
	if(rc_model()==MODEL_BB_FIRE) {
		if(unlikely(rc_gpio_init(MOT_STBY_FIRE, GPIOHANDLE_REQUEST_OUTPUT))){
			fprintf(stderr,"ERROR in rc_motor_init, failed to set up gpio %d,%d\n", MOT_STBY_FIRE);
			return -1;
		}	
	}
	else{
		if(unlikely(rc_gpio_init(MOT_STBY, GPIOHANDLE_REQUEST_OUTPUT))){
			fprintf(stderr,"ERROR in rc_motor_init, failed to set up gpio %d,%d\n", MOT_STBY);
			return -1;
		}
	}
	for(i=0;i<channels;i++){
		if(unlikely(rc_gpio_init(dirA_chip[i],dirA_pin[i], GPIOHANDLE_REQUEST_OUTPUT))){
			fprintf(stderr,"ERROR in rc_motor_init, failed to set up gpio %d,%d\n", dirA_chip[i],dirA_pin[i]);
			return -1;
		}
		if(unlikely(rc_gpio_init(dirB_chip[i],dirB_pin[i], GPIOHANDLE_REQUEST_OUTPUT))){
			fprintf(stderr,"ERROR in rc_motor_init, failed to set up gpio %d,%d\n", dirB_chip[i],dirB_pin[i]);
			return -1;
		}
	}

	// now set all the gpio pins and pwm to something predictable
	stby_state = 0;
	init_flag = 1;
	if(unlikely(rc_motor_free_spin(0))){
		fprintf(stderr,"ERROR in rc_motor_init\n");
		init_flag = 0;
		return -1;
	}

	// make sure standby is off since most users won't use it
	if(rc_model()==MODEL_BB_FIRE) {
		if(unlikely(rc_gpio_set_value(MOT_STBY_FIRE,1))){
			fprintf(stderr,"ERROR in rc_motor_init, can't write to gpio %d,%d\n",MOT_STBY_FIRE);
			return -1;
		}
	}
	else{
		if(unlikely(rc_gpio_set_value(MOT_STBY,0))){
			fprintf(stderr,"ERROR in rc_motor_init, can't write to gpio %d,%d\n",MOT_STBY);
			return -1;
		}
	}
	stby_state = 0;
	init_flag = 1;
	return 0;
}



int rc_motor_cleanup(void)
{
	int i;
	if(!init_flag) return 0;
	rc_motor_free_spin(0);
	rc_pwm_cleanup(0);
	rc_pwm_cleanup(1);
	rc_pwm_cleanup(2);
	if(rc_model()==MODEL_BB_FIRE) {
		rc_gpio_cleanup(MOT_STBY_FIRE);
	}
	else{
		rc_gpio_cleanup(MOT_STBY);
	}
	for(i=0;i<channels;i++){
		rc_gpio_cleanup(dirA_chip[i],dirA_pin[i]);
		rc_gpio_cleanup(dirB_chip[i],dirB_pin[i]);
	}
	return 0;
}


int rc_motor_standby(int standby_en)
{
	int val;
	if(unlikely(!init_flag)){
		fprintf(stderr,"ERROR in rc_motor_standby, must call rc_motor_init first\n");
		return -1;
	}
	// if use is requesting standby
	if(standby_en){
		// return if already in standby
		if(stby_state) return 0;
		val=0;
		rc_motor_free_spin(0);
	}
	else{
		if(!stby_state) return 0;
		val=1;
	}
	if(rc_model()==MODEL_BB_FIRE) {
		if(unlikely(rc_gpio_set_value(MOT_STBY_FIRE,val))){
			fprintf(stderr,"ERROR in rc_motor_standby, unable to write to gpio %d,%d\n", MOT_STBY_FIRE);
			return -1;
		}
	}
	else{
		if(unlikely(rc_gpio_set_value(MOT_STBY,val))){
			fprintf(stderr,"ERROR in rc_motor_standby, unable to write to gpio %d,%d\n", MOT_STBY);
			return -1;
		}
	}
	stby_state = standby_en;
	return 0;
}


int rc_motor_set(int motor, double duty)
{
	int a,b,i;

	// sanity checks
	if(unlikely(motor<0 || motor>channels)){
		fprintf(stderr,"ERROR in rc_motor_set, motor argument must be between 0 & %d\n", channels);
		return -1;
	}
	if(unlikely(init_flag==0)){
		fprintf(stderr, "ERROR in rc_motor_set, call rc_motor_init first\n");
		return -1;
	}

	// check that the duty cycle is within +-1
	if	(duty > 1.0)	duty = 1.0;
	else if	(duty <-1.0)	duty =-1.0;

	if(motor==0){
		for(i=1;i<=channels;i++){
			if(rc_motor_set(i,duty)==-1) return -1;
		}
		return 0;
	}

	// determine the direction pins to H-bridge
	duty=duty*polarity[motor-1];
	if(duty>=0.0){	a=1; b=0;}
	else{		a=0; b=1; duty=-duty;}

	// set gpio and pwm for that motor
	if(unlikely(rc_gpio_set_value(dirA_chip[motor-1],dirA_pin[motor-1], a))){
		fprintf(stderr,"ERROR in rc_motor_set, failed to write to gpio pin %d,%d\n",dirA_chip[motor-1],dirA_pin[motor-1]);
		return -1;
	}
	if(unlikely(rc_gpio_set_value(dirB_chip[motor-1],dirB_pin[motor-1], b))){
		fprintf(stderr,"ERROR in rc_motor_set, failed to write to gpio pin %d,%d\n",dirB_chip[motor-1],dirB_pin[motor-1]);
		return -1;
	}
	if(unlikely(rc_pwm_set_duty(pwmss[motor-1], pwmch[motor-1], duty))){
		fprintf(stderr,"ERROR in rc_motor_set, failed to write to pwm %d%c\n",pwmss[motor-1], pwmch[motor-1]);
		return -1;
	}
	return 0;
}


int rc_motor_free_spin(int motor)
{
	int i;

	// sanity checks
	if(unlikely(motor<0 || motor>channels)){
		fprintf(stderr,"ERROR in rc_motor_free_spin, motor argument must be between 0 & %d\n", channels);
		return -1;
	}
	if(unlikely(init_flag==0)){
		fprintf(stderr, "ERROR in rc_motor_free_spin, call rc_motor_init first\n");
		return -1;
	}

	// case for all channels
	if(motor==0){
		for(i=1;i<=channels;i++){
			if(rc_motor_free_spin(i)==-1) return -1;
		}
		return 0;
	}

	// set gpio and pwm for that motor
	if(unlikely(rc_gpio_set_value(dirA_chip[motor-1],dirA_pin[motor-1], 0))){
		fprintf(stderr,"ERROR in rc_motor_free_spin, failed to write to gpio pin %d,%d\n",dirA_chip[motor-1],dirA_pin[motor-1]);
		return -1;
	}
	if(unlikely(rc_gpio_set_value(dirB_chip[motor-1],dirB_pin[motor-1], 0))){
		fprintf(stderr,"ERROR in rc_motor_free_spin, failed to write to gpio pin %d,%d\n",dirB_chip[motor-1],dirB_pin[motor-1]);
		return -1;
	}
	if(unlikely(rc_pwm_set_duty(pwmss[motor-1], pwmch[motor-1], 0.0))){
		fprintf(stderr,"ERROR in rc_motor_free_spin, failed to write to pwm %d%c\n",pwmss[motor-1], pwmch[motor-1]);
		return -1;
	}
	return 0;
}


int rc_motor_brake(int motor)
{
	int i;

	// sanity checks
	if(unlikely(motor<0 || motor>channels)){
		fprintf(stderr,"ERROR in rc_motor_brake, motor argument must be between 0 & %d\n", channels);
		return -1;
	}
	if(unlikely(init_flag==0)){
		fprintf(stderr, "ERROR in rc_motor_brake, call rc_motor_init first\n");
		return -1;
	}

	// case for all channels
	if(motor==0){
		for(i=1;i<=channels;i++){
			if(rc_motor_brake(i)==-1) return -1;
		}
		return 0;
	}

	// set gpio and pwm for that motor
	if(unlikely(rc_gpio_set_value(dirA_chip[motor-1],dirA_pin[motor-1], 1))){
		fprintf(stderr,"ERROR in rc_motor_brake, failed to write to gpio pin %d,%d\n",dirA_chip[motor-1],dirA_pin[motor-1]);
		return -1;
	}
	if(unlikely(rc_gpio_set_value(dirB_chip[motor-1],dirB_pin[motor-1], 1))){
		fprintf(stderr,"ERROR in rc_motor_brake, failed to write to gpio pin %d,%d\n",dirB_chip[motor-1],dirB_pin[motor-1]);
		return -1;
	}
	if(unlikely(rc_pwm_set_duty(pwmss[motor-1], pwmch[motor-1], 0.0))){
		fprintf(stderr,"ERROR in rc_motor_brake, failed to write to pwm %d%c\n",pwmss[motor-1], pwmch[motor-1]);
		return -1;
	}
	return 0;
}
