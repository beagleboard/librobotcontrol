/**
 * @file button.c
 */

#include <errno.h>
#include <stdio.h>
#include <poll.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>

#ifdef RC_AUTOPILOT_EXT
// Not sure why #include <linux/gpio.h> did not work here after explicitly include
// the default path /usr/include, so use full path for now.
#include "/usr/include/linux/gpio.h"
#else
#include <linux/gpio.h>
#endif

#include <rc/gpio.h>
#include <rc/time.h>
#include <rc/pthread.h>
#include <rc/button.h>

// preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)
#define likely(x)	__builtin_expect (!!(x), 1)

#define CHIPS_MAX 6
#define POLL_TIMEOUT_MS	500	// 0.1 seconds
#define THREAD_TIMEOUT	3.0	// 3 seconds

// state of a given pin
typedef struct btn_cfg_t{
	void (*press_cb)(void);
	void (*release_cb)(void);
	pthread_t poll_thread;
	char started;
	char pol;
	pthread_mutex_t press_mutex;
	pthread_cond_t  press_condition;
	pthread_mutex_t release_mutex;
	pthread_cond_t  release_condition;
} btn_cfg_t;


// struct passed to each button thread to configure it
typedef struct thread_cfg_t{
	int chip;
	int pin;
	int pol;
	int debounce;
} thread_cfg_t;

// pointer to dynamically allocated btn_cfg_t structs
static btn_cfg_t* cfg[CHIPS_MAX][GPIOHANDLES_MAX];
static int shutdown_flag = 0;

/**
 * poll a gpio edge with debounce check. When the button changes state spawn off
 * the user-defined callback in its own thread.
 */
static void* poll_thread_func(void* arg)
{
	int val, event, chip, pin, pol, debounce;
	int press_expected_event, release_expected_event;
	pthread_t press_thread, release_thread;

	thread_cfg_t thread_cfg = *(thread_cfg_t*)arg;
	chip = thread_cfg.chip;
	pin = thread_cfg.pin;
	pol = thread_cfg.pol;
	debounce = thread_cfg.debounce;

	// once config data has been saved locally, flag that the thread has started
	cfg[chip][pin]->started=1;

	// based on polarity, set up expected events for each direction
	if(pol==RC_BTN_POLARITY_NORM_HIGH){
		press_expected_event = RC_GPIOEVENT_FALLING_EDGE;
		release_expected_event = RC_GPIOEVENT_RISING_EDGE;
	}
	else{
		press_expected_event = RC_GPIOEVENT_RISING_EDGE;
		release_expected_event = RC_GPIOEVENT_FALLING_EDGE;
	}


	// keep running until the program closes
	while(!shutdown_flag){
		event=rc_gpio_poll(chip, pin, POLL_TIMEOUT_MS, NULL);
		if(event==RC_GPIOEVENT_ERROR){
			fprintf(stderr,"ERROR in rc_button handler thread\n");
			return NULL;
		}
		if(event==RC_GPIOEVENT_TIMEOUT) continue;

		// if debounce is enabled, do extra wait and check
		if(debounce){
			rc_usleep(debounce);
			val=rc_gpio_get_value(chip,pin);
			if(val==-1){
				fprintf(stderr,"ERROR in rc_button handler thread\n");
				return NULL;
			}
			if(event==RC_GPIOEVENT_FALLING_EDGE && val!=0) continue;
			else if(event==RC_GPIOEVENT_RISING_EDGE && val!=1) continue;

		}

		// spawn callback functions
		if(event==press_expected_event){
			if(cfg[chip][pin]->press_cb!=NULL){
				rc_pthread_create(&press_thread, (void* (*)(void*))cfg[chip][pin]->press_cb, NULL, SCHED_OTHER, 0);
			}
			// aquires mutex and broadcast condition
			pthread_mutex_lock(&cfg[chip][pin]->press_mutex);
			pthread_cond_broadcast(&cfg[chip][pin]->press_condition);
			pthread_mutex_unlock(&cfg[chip][pin]->press_mutex);

		}
		else if(event==release_expected_event){
			if(cfg[chip][pin]->release_cb!=NULL){
				rc_pthread_create(&release_thread, (void* (*)(void*))cfg[chip][pin]->release_cb, NULL, SCHED_OTHER, 0);
			}
			// aquires mutex and broadcast condition
			pthread_mutex_lock(&cfg[chip][pin]->release_mutex);
			pthread_cond_broadcast(&cfg[chip][pin]->release_condition);
			pthread_mutex_unlock(&cfg[chip][pin]->release_mutex);
		}
	}

	return NULL;
}


int rc_button_init(int chip, int pin, char polarity, int debounce_us)
{
	int i;
	btn_cfg_t* ptr = NULL;
	thread_cfg_t thread_cfg;

	// sanity checks
	if(chip<0 || chip>=CHIPS_MAX){
		fprintf(stderr,"ERROR in rc_button_init, chip out of bounds\n");
		return -1;
	}
	if(pin<0 || pin>=GPIOHANDLES_MAX){
		fprintf(stderr,"ERROR in rc_button_init, pin out of bounds\n");
		return -1;
	}
	if(polarity!=RC_BTN_POLARITY_NORM_LOW && polarity!=RC_BTN_POLARITY_NORM_HIGH){
		fprintf(stderr,"ERROR in rc_button_init\n");
		fprintf(stderr,"polarity must be RC_BTN_POLARITY_NORM_LOW or RC_BTN_POLARITY_NORM_HIGH\n");
		return -1;
	}
	if(debounce_us<0){
		fprintf(stderr, "ERROR in rc_button_init, debounce_us must be >=0\n");
		return -1;
	}
	if(ptr!=NULL){
		fprintf(stderr, "ERROR in rc_button_init, button already initialized\n");
		return -1;
	}

	// basic gpio setup
	if(rc_gpio_init_event(chip,pin,GPIOHANDLE_REQUEST_INPUT,GPIOEVENT_REQUEST_BOTH_EDGES)==-1){
		fprintf(stderr,"ERROR: in rc_button_init, failed to setup GPIO pin\n");
		return -1;
	}

	// allocate memory for the config for that pin
	ptr = (btn_cfg_t*)malloc(sizeof(btn_cfg_t));
	if(ptr==NULL){
		perror("ERROR in rc_button_init");
		return -1;
	}

	// start filling in the pin config struct
	ptr->press_cb=NULL;
	ptr->release_cb=NULL;
	ptr->poll_thread=0;
	ptr->started=0;
	ptr->pol=polarity;
	pthread_mutex_init(&ptr->press_mutex, NULL);
	pthread_cond_init(&ptr->press_condition, NULL);
	pthread_mutex_init(&ptr->release_mutex, NULL);
	pthread_cond_init(&ptr->release_condition, NULL);

	// set up thread config structs
	thread_cfg.chip = chip;
	thread_cfg.pin = pin;
	thread_cfg.pol = polarity;
	thread_cfg.debounce = debounce_us;

	// start threads
	shutdown_flag=0;
	cfg[chip][pin]=ptr;

	if(rc_pthread_create(&ptr->poll_thread, poll_thread_func, (void*)&thread_cfg, SCHED_OTHER, 0)){
		fprintf(stderr,"ERROR in rc_button_init, failed to start press handler thread\n");
		cfg[chip][pin]=NULL;
		return -1;
	}

	// wait for thread to start
	i=0;
	while(ptr->started==0){
		i++;
		if(i>=100){
			fprintf(stderr,"ERROR in rc_button_init, timeout waiting for thread to start\n");
			cfg[chip][pin]=NULL;
			return -1;
		}
		rc_usleep(1000);
	}

	// done!

	return 0;
}


void rc_button_cleanup(void)
{
	int i,j, ret;
	// signal threads to close
	shutdown_flag=1;
	// loop through all pins
	for(i=0;i<CHIPS_MAX;i++){
		for(j=0;j<GPIOHANDLES_MAX;j++){
			// skip uninitialized pins
			if(cfg[i][j]==NULL) continue;
			// join thread
			ret=rc_pthread_timed_join(cfg[i][j]->poll_thread,NULL,THREAD_TIMEOUT);
			if(ret==-1){
				fprintf(stderr,"WARNING in rc_button_cleanup, problem joining button handler thread for pin %d\n",i);
			}
			else if(ret==1){
				fprintf(stderr,"WARNING in rc_button_cleanup, thread exit timeout for pin %d\n",i);
				fprintf(stderr,"most likely cause is your button press callback function is stuck and didn't return\n");
			}
			free(cfg[i][j]);
			cfg[i][j]=NULL;
		}
	}
	return;
}


int rc_button_set_callbacks(int chip, int pin, void (*press_func)(void), void (*release_func)(void))
{
	// sanity checks
	if(chip<0 || chip>=CHIPS_MAX){
		fprintf(stderr,"ERROR in rc_button_set_callbacks, chip out of bounds\n");
		return -1;
	}
	if(pin<0 || pin>=GPIOHANDLES_MAX){
		fprintf(stderr,"ERROR in rc_button_set_callbacks, pin out of bounds\n");
		return -1;
	}
	if(cfg[chip][pin]==NULL){
		fprintf(stderr,"ERROR in rc_button_set_callbacks, pin not initialized yet\n");
		return -1;
	}
	cfg[chip][pin]->press_cb = press_func;
	cfg[chip][pin]->release_cb = release_func;
	return 0;
}


int rc_button_get_state(int chip, int pin)
{
	int val,ret;
	// sanity checks
	if(chip<0 || chip>=CHIPS_MAX){
		fprintf(stderr,"ERROR in rc_button_get_state, chip out of bounds\n");
		return -1;
	}
	if(pin<0 || pin>=GPIOHANDLES_MAX){
		fprintf(stderr,"ERROR in rc_button_get_state, pin out of bounds\n");
		return -1;
	}
	if(cfg[chip][pin]==NULL){
		fprintf(stderr,"ERROR in rc_button_get_state, pin not initialized yet\n");
		return -1;
	}
	// read gpio chip
	val=rc_gpio_get_value(chip,pin);
	if(val==-1){
		fprintf(stderr,"ERROR in rc_button_get_state\n");
		return -1;
	}
	// determine return based on polarity
	if(cfg[chip][pin]->pol==RC_BTN_POLARITY_NORM_HIGH){
		if(val) ret=RC_BTN_STATE_RELEASED;
		else ret=RC_BTN_STATE_PRESSED;
	}
	else{
		if(val) ret=RC_BTN_STATE_PRESSED;
		else ret=RC_BTN_STATE_RELEASED;
	}
	return ret;
}

int rc_button_wait_for_event(int chip, int pin, int press_or_release)
{
	// sanity checks
	if(chip<0 || chip>=CHIPS_MAX){
		fprintf(stderr,"ERROR in rc_button_wait_for_event, chip out of bounds\n");
		return -1;
	}
	if(pin<0 || pin>=GPIOHANDLES_MAX){
		fprintf(stderr,"ERROR in rc_button_wait_for_event, pin out of bounds\n");
		return -1;
	}
	if(cfg[chip][pin]==NULL){
		fprintf(stderr,"ERROR in rc_button_wait_for_event, pin not initialized yet\n");
		return -1;
	}

	if(press_or_release==RC_BTN_STATE_PRESSED){
		// wait for condition signal which unlocks mutex
		pthread_mutex_lock(&cfg[chip][pin]->press_mutex);
		pthread_cond_wait(&cfg[chip][pin]->press_condition, &cfg[chip][pin]->press_mutex);
		pthread_mutex_unlock(&cfg[chip][pin]->press_mutex);
	}
	else if(press_or_release==RC_BTN_STATE_RELEASED){
		// wait for condition signal which unlocks mutex
		pthread_mutex_lock(&cfg[chip][pin]->release_mutex);
		pthread_cond_wait(&cfg[chip][pin]->release_condition, &cfg[chip][pin]->release_mutex);
		pthread_mutex_unlock(&cfg[chip][pin]->release_mutex);
	}
	else{
		fprintf(stderr, "ERROR in rc_button_wait_for_event, argument should be RC_BTN_STATE_PRESSED or RC_BTN_STATE_RELEASED\n");
		return -1;
	}

	return 0;
}


