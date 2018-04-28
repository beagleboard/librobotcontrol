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

// motor pin definitions
#define MDIR1A			60	//gpio1.28	P9.12
#define MDIR1A_BLUE		64	//gpio2.0	pin T13
#define MDIR1B			31	//gpio0.31	P9.13
#define MDIR2A			48	//gpio1.16	P9.15
#define MDIR2B			81	//gpio2.17	P8.34
#define MDIR2B_BLUE		10	//gpio0.10	P8_31
#define MDIR4A			70	//gpio2.6	P8.45
#define MDIR4B			71	//gpio2.7	P8.46
#define MDIR3B			72	//gpio2.8	P8.43
#define MDIR3A			73	//gpio2.9	P8.44
#define MOT_STBY		20	//gpio0.20	P9.41

#define CHANNELS		4
#define PWM_FREQ		25000	// 25kHz

// polarity of the motor connections
const static float polarity[]={1.0,-1.0,-1.0,1.0};

static int init_flag = 0;
static int stby_state = 0;
static int dirA[CHANNELS];
static int dirB[CHANNELS];
static int pwmss[CHANNELS];
static int pwmch[CHANNELS];




int rc_motor_init()
{
	int i;

	// set pins for motor 1
	// assign gpio pins for blue/black
	if(rc_model()==BB_BLUE) dirA[0]=MDIR1A_BLUE;
	else dirA[0] = MDIR1A;
	dirB[0]=MDIR1B;
	pwmss[0]=1;
	pwmch[0]='A';

	// motor 2
	dirA[1]=MDIR2A;
	if(rc_model()==BB_BLUE) dirB[1]=MDIR2B_BLUE;
	else dirB[1] = MDIR2B;
	pwmss[1]=1;
	pwmch[1]='B';

	// motor 3
	dirA[2]=MDIR3A;
	dirB[2]=MDIR3B;
	pwmss[2]=2;
	pwmch[2]='A';

	// motor 4
	dirA[3]=MDIR4A;
	dirB[3]=MDIR4B;
	pwmss[3]=2;
	pwmch[3]='B';

	// set up pwm channels
	if(unlikely(rc_pwm_init(1,PWM_FREQ))){
		fprintf(stderr,"ERROR in rc_motor_init, failed to initialize pwm subsystem 1\n");
		return -1;
	}
	if(unlikely(rc_pwm_init(2,PWM_FREQ))){
		fprintf(stderr,"ERROR in rc_motor_init, failed to initialize pwm subsystem 2\n");
		return -1;
	}

	// set up gpio pins
	if(unlikely(rc_gpio_init(MOT_STBY, GPIOHANDLE_REQUEST_OUTPUT))){
		fprintf(stderr,"ERROR in rc_motor_init, failed to set up gpio %d\n", MOT_STBY);
		return -1;
	}
	for(i=0;i<CHANNELS;i++){
		if(unlikely(rc_gpio_init(dirA[i], GPIOHANDLE_REQUEST_OUTPUT))){
			fprintf(stderr,"ERROR in rc_motor_init, failed to set up gpio %d\n", dirA[i]);
			return -1;
		}
		if(unlikely(rc_gpio_init(dirB[i], GPIOHANDLE_REQUEST_OUTPUT))){
			fprintf(stderr,"ERROR in rc_motor_init, failed to set up gpio %d\n", dirB[i]);
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
	if(unlikely(rc_gpio_set_value(MOT_STBY,1))){
		fprintf(stderr,"ERROR in rc_motor_init, can't write to gpio %d\n",MOT_STBY);
		return -1;
	}
	stby_state = 0;
	init_flag = 1;
	return 0;
}



int rc_motor_cleanup()
{
	int i;
	if(!init_flag) return 0;
	rc_motor_free_spin(0);
	rc_pwm_cleanup(1);
	rc_pwm_cleanup(2);
	rc_gpio_cleanup(MOT_STBY);
	for(i=0;i<CHANNELS;i++){
		rc_gpio_cleanup(dirA[i]);
		rc_gpio_cleanup(dirB[i]);
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
	if(unlikely(rc_gpio_set_value(MOT_STBY,val))){
		fprintf(stderr,"ERROR in rc_motor_standby, unable to write to gpio %d\n", MOT_STBY);
		return -1;
	}
	stby_state = standby_en;
	return 0;
}


int rc_motor_set(int motor, float duty)
{
	int a,b,i;

	// sanity checks
	if(unlikely(motor<0 || motor>CHANNELS)){
		fprintf(stderr,"ERROR in rc_motor_set, motor argument must be between 0 & %d\n", CHANNELS);
		return -1;
	}
	if(unlikely(init_flag==0)){
		fprintf(stderr, "ERROR in rc_motor_set, call rc_motor_init first\n");
		return -1;
	}

	// check that the duty cycle is within +-1
	if	(duty > 1.0f)	duty = 1.0f;
	else if	(duty <-1.0f)	duty =-1.0f;

	if(motor==0){
		for(i=1;i<CHANNELS;i++){
			if(rc_motor_set(i,duty)==-1) return -1;
		}
		return 0;
	}

	// determine the direction pins to H-bridge
	duty=duty*polarity[motor-1];
	if(duty>=0){	a=1; b=0;}
	else{		a=0; b=1; duty=-duty;}

	// set gpio and pwm for that motor
	if(unlikely(rc_gpio_set_value(dirA[motor-1], a))){
		fprintf(stderr,"ERROR in rc_motor_set, failed to write to gpio pin %d\n",dirA[motor-1]);
		return -1;
	}
	if(unlikely(rc_gpio_set_value(dirB[motor-1], b))){
		fprintf(stderr,"ERROR in rc_motor_set, failed to write to gpio pin %d\n",dirB[motor-1]);
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
	if(unlikely(motor<0 || motor>CHANNELS)){
		fprintf(stderr,"ERROR in rc_motor_free_spin, motor argument must be between 0 & %d\n", CHANNELS);
		return -1;
	}
	if(unlikely(init_flag==0)){
		fprintf(stderr, "ERROR in rc_motor_free_spin, call rc_motor_init first\n");
		return -1;
	}

	// case for all channels
	if(motor==0){
		for(i=1;i<CHANNELS;i++){
			if(rc_motor_free_spin(i)==-1) return -1;
		}
		return 0;
	}

	// set gpio and pwm for that motor
	if(unlikely(rc_gpio_set_value(dirA[motor-1], 0))){
		fprintf(stderr,"ERROR in rc_motor_free_spin, failed to write to gpio pin %d\n",dirA[motor-1]);
		return -1;
	}
	if(unlikely(rc_gpio_set_value(dirB[motor-1], 0))){
		fprintf(stderr,"ERROR in rc_motor_free_spin, failed to write to gpio pin %d\n",dirB[motor-1]);
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
	if(unlikely(motor<0 || motor>CHANNELS)){
		fprintf(stderr,"ERROR in rc_motor_brake, motor argument must be between 0 & %d\n", CHANNELS);
		return -1;
	}
	if(unlikely(init_flag==0)){
		fprintf(stderr, "ERROR in rc_motor_brake, call rc_motor_init first\n");
		return -1;
	}

	// case for all channels
	if(motor==0){
		for(i=1;i<CHANNELS;i++){
			if(rc_motor_brake(i)==-1) return -1;
		}
		return 0;
	}

	// set gpio and pwm for that motor
	if(unlikely(rc_gpio_set_value(dirA[motor-1], 1))){
		fprintf(stderr,"ERROR in rc_motor_brake, failed to write to gpio pin %d\n",dirA[motor-1]);
		return -1;
	}
	if(unlikely(rc_gpio_set_value(dirB[motor-1], 1))){
		fprintf(stderr,"ERROR in rc_motor_brake, failed to write to gpio pin %d\n",dirB[motor-1]);
		return -1;
	}
	if(unlikely(rc_pwm_set_duty(pwmss[motor-1], pwmch[motor-1], 0.0))){
		fprintf(stderr,"ERROR in rc_motor_brake, failed to write to pwm %d%c\n",pwmss[motor-1], pwmch[motor-1]);
		return -1;
	}
	return 0;
}