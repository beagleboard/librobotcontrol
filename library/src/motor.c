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

#define MOTOR_CHANNELS		4
#define PWM_SUBSYSTEMS		3

struct motorcfg {
	const struct rc_gpiodesc inA;
	const struct rc_gpiodesc inB;
	const struct rc_pwmdesc pwm;
};

enum configs {
	BLACK = 0,
	BLUE = 1,
	AI64 = 2,
	POCKET = 3,
	NUM_CONFIGS
};

struct sys_motorcfg {
	const char * desc;
	const struct motorcfg m[MOTOR_CHANNELS];
	const double polarity[MOTOR_CHANNELS];
	const struct rc_gpiodesc standby;
	const int pwms[PWM_SUBSYSTEMS];
	const int channels;
} cfgs[NUM_CONFIGS] =
{
  [BLACK] = {
	.desc = "Black+RC-A",
	.m = {
		{
			.inA = { .chip = 1, .pin = 28 }, /* P9_12 */
			.inB = { .chip = 0, .pin = 31 }, /* P9_13 */
			.pwm = { .ss = 1, .chan = 'A' },
		},
		{
			.inA = { .chip = 1, .pin = 16 }, /* P9_15 */
			.inB = { .chip = 2, .pin = 17 }, /* P8_34 */
			.pwm = { .ss = 1, .chan = 'B' },
		},
		{
			.inA = { .chip = 2, .pin = 9 }, /* P8_44 */
			.inB = { .chip = 2, .pin = 8 }, /* P8_43 */
			.pwm = { .ss = 2, .chan = 'A' },
		},
		{
			.inA = { .chip = 2, .pin = 6 }, /* P8_45 */
			.inB = { .chip = 2, .pin = 7 }, /* P8_46 */
			.pwm = { .ss = 2, .chan = 'B' },
		},
	},
	.polarity = { 1.0, -1.0, -1.0, 1.0 },
	.standby = { .chip = 0, .pin = 20 }, /* P9_41 */
	.pwms = { 1, 2, -1 },
	.channels = 4,
  },
  [BLUE] = {
	.desc = "Blue-A",
	.m = {
		{
			.inA = { .chip = 2, .pin = 0 }, /* T13 */
			.inB = { .chip = 0, .pin = 31 },
			.pwm = { .ss = 1, .chan = 'A' },
		},
		{
			.inA = { .chip = 1, .pin = 16 },
			.inB = { .chip = 0, .pin = 10 }, /* P8_31 */
			.pwm = { .ss = 1, .chan = 'B' },
		},
		{
			.inA = { .chip = 2, .pin = 9 }, /* P8_44 */
			.inB = { .chip = 2, .pin = 8 }, /* P8_43 */
			.pwm = { .ss = 2, .chan = 'A' },
		},
		{
			.inA = { .chip = 2, .pin = 6 }, /* P8_45 */
			.inB = { .chip = 2, .pin = 7 }, /* P8_46 */
			.pwm = { .ss = 2, .chan = 'B' },
		},
	},
	.polarity = { 1.0, -1.0, -1.0, 1.0 },
	.standby = { .chip = 0, .pin = 20 }, /* P9_41 */
	.pwms = { 1, 2, -1 },
	.channels = 4,
  },
  [AI64] = {
	.desc = "AI64+RC-B",
	.m = {
		/* PWM subsystem 100 is software emulation on a PRU */
		{
			.inA = { .chip = 2, .pin = 0 }, /* T13 */
			.inB = { .chip = 0, .pin = 31 },
			.pwm = { .ss = 100, .chan = 'A' },
		},
		{
			.inA = { .chip = 1, .pin = 16 },
			.inB = { .chip = 0, .pin = 10 }, /* P8_31 */
			.pwm = { .ss = 100, .chan = 'B' },
		},
		{
			.inA = { .chip = 2, .pin = 9 }, /* P8_44 */
			.inB = { .chip = 2, .pin = 8 }, /* P8_43 */
			.pwm = { .ss = 100, .chan = 'C' },
		},
		{
			.inA = { .chip = 2, .pin = 6 }, /* P8_45 */
			.inB = { .chip = 2, .pin = 7 }, /* P8_46 */
			.pwm = { .ss = 100, .chan = 'D' },
		},
	},
	.polarity = { 1.0, -1.0, -1.0, 1.0 },
	.standby = { .chip = 0, .pin = 20 }, /* P9_41 */
	.pwms = { 100, -1, -1 },
	.channels = 4,
  },
  [POCKET] = {
	.desc = "Pocket",
	.m = {
		{
			.inA = { .chip = 1, .pin = 28 }, /* P2_08 */
			.inB = { .chip = 0, .pin = 31 }, /* P2_07 */
			.pwm = { .ss = 1, .chan = 'A' },
		},
		{
			.inA = { .chip = 0, .pin = 26 }, /* P1_34 */
			.inB = { .chip = 1, .pin = 27 }, /* P2_02 */
			.pwm = { .ss = 0, .chan = 'A' },
		},
	},
	.polarity = { 1.0, -1.0, -1.0, 1.0 },
	.standby = { .chip = 0, .pin = 20 }, /* P1_20 */
	.pwms = { 0, 1, -1 },
	.channels = 2,
  },
};

static int init_flag = 0;
static int stby_state = 0;

static struct sys_motorcfg * cfg;

#define MOT_STBY cfg->standby.chip,cfg->standby.pin
#define INA(i) cfg->m[i].inA.chip,cfg->m[i].inA.pin
#define INB(i) cfg->m[i].inB.chip,cfg->m[i].inB.pin
#define PWM(i) cfg->m[i].pwm.ss,cfg->m[i].pwm.chan

int rc_motor_init(void)
{
	return rc_motor_init_freq(RC_MOTOR_DEFAULT_PWM_FREQ);
}


int rc_motor_init_freq(int pwm_frequency_hz)
{
	int i;

	if(rc_model()==MODEL_BB_BLUE){
		cfg = &cfgs[BLUE];
	}
	else if(rc_model()==MODEL_BB_AI64){
		cfg = &cfgs[AI64];
	}
	else if(rc_model()==MODEL_BB_POCKET) {
		cfg = &cfgs[POCKET];
	}
	else{
		cfg = &cfgs[BLACK];
	}

	// setup pwm channels
	for(i=0;i<PWM_SUBSYSTEMS;i++){
		if(cfg->pwms[i] >= 0){
			if(unlikely(rc_pwm_init(cfg->pwms[i],pwm_frequency_hz))){
				fprintf(stderr,
		"ERROR in rc_motor_init, failed to initialize pwm subsystem %d\n",
				cfg->pwms[i]);
				return -1;
			}
		}
		return -1;
	}

	// setup gpio pins
	if(unlikely(rc_gpio_init(MOT_STBY, GPIOHANDLE_REQUEST_OUTPUT))){
		fprintf(stderr,"ERROR in rc_motor_init, failed to set up gpio %d,%d\n", MOT_STBY);
		return -1;
	}
	for(i=0;i<cfg->channels;i++){
		if(unlikely(rc_gpio_init(INA(i), GPIOHANDLE_REQUEST_OUTPUT))){
			fprintf(stderr,"ERROR in rc_motor_init, failed to set up gpio %d,%d\n", INA(i));
			return -1;
		}
		if(unlikely(rc_gpio_init(INB(i), GPIOHANDLE_REQUEST_OUTPUT))){
			fprintf(stderr,"ERROR in rc_motor_init, failed to set up gpio %d,%d\n", INB(i));
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
		fprintf(stderr,"ERROR in rc_motor_init, can't write to gpio %d,%d\n",MOT_STBY);
		return -1;
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
	rc_gpio_cleanup(MOT_STBY);
	if(!cfg) return(-1);
	for(i=0;i<cfg->channels;i++){
		rc_gpio_cleanup(INA(i));
		rc_gpio_cleanup(INB(i));
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
		fprintf(stderr,"ERROR in rc_motor_standby, unable to write to gpio %d,%d\n", MOT_STBY);
		return -1;
	}
	stby_state = standby_en;
	return 0;
}


int rc_motor_set(int motor, double duty)
{
	int a,b,i;

	// sanity checks
	if(unlikely(init_flag==0)){
		fprintf(stderr, "ERROR in rc_motor_set, call rc_motor_init first\n");
		return -1;
	}
	if(unlikely(!cfg)){
		fprintf(stderr,"ERROR in rc_motor_set, 'cfg' not set\n");
		return -1;
	}
	if(unlikely(motor<0 || motor>cfg->channels)){
		fprintf(stderr,"ERROR in rc_motor_set, motor argument must be between 0 & %d\n", cfg->channels);
		return -1;
	}

	// check that the duty cycle is within +-1
	if	(duty > 1.0)	duty = 1.0;
	else if	(duty <-1.0)	duty =-1.0;

	if(motor==0){
		for(i=1;i<=cfg->channels;i++){
			if(rc_motor_set(i,duty)==-1) return -1;
		}
		return 0;
	}

	// determine the direction pins to H-bridge
	duty=duty*cfg->polarity[motor-1];
	if(duty>=0.0){	a=1; b=0;}
	else{		a=0; b=1; duty=-duty;}

	// set gpio and pwm for that motor
	if(unlikely(rc_gpio_set_value(INA(motor-1), a))){
		fprintf(stderr,"ERROR in rc_motor_set, failed to write to gpio pin %d,%d\n",INA(motor-1));
		return -1;
	}
	if(unlikely(rc_gpio_set_value(INB(motor-1), b))){
		fprintf(stderr,"ERROR in rc_motor_set, failed to write to gpio pin %d,%d\n",INB(motor-1));
		return -1;
	}
	if(unlikely(rc_pwm_set_duty(PWM(motor-1), duty))){
		fprintf(stderr,"ERROR in rc_motor_set, failed to write to pwm %d%c\n",PWM(motor-1));
		return -1;
	}
	return 0;
}


int rc_motor_free_spin(int motor)
{
	int i;

	// sanity checks
	if(unlikely(init_flag==0)){
		fprintf(stderr, "ERROR in rc_motor_free_spin, call rc_motor_init first\n");
		return -1;
	}
	if(unlikely(!cfg)){
		fprintf(stderr,"ERROR in rc_motor_free_spin, 'cfg' not set\n");
		return -1;
	}
	if(unlikely(motor<0 || motor>cfg->channels)){
		fprintf(stderr,"ERROR in rc_motor_free_spin, motor argument must be between 0 & %d\n", cfg->channels);
		return -1;
	}

	// case for all channels
	if(motor==0){
		for(i=1;i<=cfg->channels;i++){
			if(rc_motor_free_spin(i)==-1) return -1;
		}
		return 0;
	}

	// set gpio and pwm for that motor
	if(unlikely(rc_gpio_set_value(INA(motor-1), 0))){
		fprintf(stderr,"ERROR in rc_motor_free_spin, failed to write to gpio pin %d,%d\n",INA(motor-1));
		return -1;
	}
	if(unlikely(rc_gpio_set_value(INB(motor-1), 0))){
		fprintf(stderr,"ERROR in rc_motor_free_spin, failed to write to gpio pin %d,%d\n",INB(motor-1));
		return -1;
	}
	if(unlikely(rc_pwm_set_duty(PWM(motor-1), 0.0))){
		fprintf(stderr,"ERROR in rc_motor_free_spin, failed to write to pwm %d%c\n",PWM(motor-1));
		return -1;
	}
	return 0;
}


int rc_motor_brake(int motor)
{
	int i;

	// sanity checks
	if(unlikely(init_flag==0)){
		fprintf(stderr, "ERROR in rc_motor_brake, call rc_motor_init first\n");
		return -1;
	}
	if(unlikely(!cfg)){
		fprintf(stderr,"ERROR in rc_motor_break, 'cfg' not set\n");
		return -1;
	}
	if(unlikely(motor<0 || motor>cfg->channels)){
		fprintf(stderr,"ERROR in rc_motor_brake, motor argument must be between 0 & %d\n", cfg->channels);
		return -1;
	}

	// case for all channels
	if(motor==0){
		for(i=1;i<=cfg->channels;i++){
			if(rc_motor_brake(i)==-1) return -1;
		}
		return 0;
	}

	// set gpio and pwm for that motor
	if(unlikely(rc_gpio_set_value(INA(motor-1), 1))){
		fprintf(stderr,"ERROR in rc_motor_brake, failed to write to gpio pin %d,%d\n",INA(motor-1));
		return -1;
	}
	if(unlikely(rc_gpio_set_value(INB(motor-1), 1))){
		fprintf(stderr,"ERROR in rc_motor_brake, failed to write to gpio pin %d,%d\n",INB(motor-1));
		return -1;
	}
	if(unlikely(rc_pwm_set_duty(PWM(motor-1), 0.0))){
		fprintf(stderr,"ERROR in rc_motor_brake, failed to write to pwm %d%c\n",PWM(motor-1));
		return -1;
	}
	return 0;
}
