/**
 * @file button.c
 */

#include <errno.h>
#include <stdio.h>
#include <poll.h>
#include <unistd.h>
#include <fcntl.h>

#ifndef RC_AUTOPILOT_EXT
#include <linux/gpio.h>
#endif

#include <rc/gpio.h>
#include <rc/time.h>
#include <rc/pthread.h>
#include <rc/button.h>

// preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)
#define likely(x)	__builtin_expect (!!(x), 1)

#define MAX_PINS	128
#define POLL_TIMEOUT_MS	500	// 0.1 seconds
#define THREAD_TIMEOUT	3.0	// 3 seconds

static void (*press_cb[MAX_PINS])(void);
static void (*release_cb[MAX_PINS])(void);
static pthread_t poll_thread[MAX_PINS];
static char init_flag[MAX_PINS];
static char started[MAX_PINS];
static int shutdown_flag = 0;
static char pol[MAX_PINS];

// struct passed to each button thread to configure it
typedef struct thread_cfg_t{
	int pin;
	int debounce;
} thread_cfg_t;


/**
 * poll a gpio edge with debounce check. When the button changes state spawn off
 * teh user-defined callback in its own thread.
 */
void* poll_thread_func(void* arg)
{
	int val, event;
	int press_expected_event, release_expected_event;
	pthread_t press_thread, release_thread;

	thread_cfg_t cfg = *(thread_cfg_t*)arg;
	// once config data has been saved locally, flag that the thread has started
	started[cfg.pin]=1;

	// based on polaity, set up expected events for each direction
	if(pol[cfg.pin]==RC_BTN_POLARITY_NORM_HIGH){
		press_expected_event = RC_GPIOEVENT_FALLING_EDGE;
		release_expected_event = RC_GPIOEVENT_RISING_EDGE;
	}
	else{
		press_expected_event = RC_GPIOEVENT_RISING_EDGE;
		release_expected_event = RC_GPIOEVENT_FALLING_EDGE;
	}


	// keep running until the program closes
	while(!shutdown_flag){
		event=rc_gpio_poll(cfg.pin,POLL_TIMEOUT_MS,NULL);
		if(event==RC_GPIOEVENT_ERROR){
			fprintf(stderr,"ERROR in rc_button handler thread\n");
			return NULL;
		}
		if(event==RC_GPIOEVENT_TIMEOUT) continue;

		// if debounce is enabled, do extra wait and check
		if(cfg.debounce){
			rc_usleep(cfg.debounce);
			val=rc_gpio_get_value(cfg.pin);
			if(val==-1){
				fprintf(stderr,"ERROR in rc_button handler thread\n");
				return NULL;
			}
			if(event==RC_GPIOEVENT_FALLING_EDGE && val!=0) continue;
			else if(event==RC_GPIOEVENT_RISING_EDGE && val!=1) continue;

		}

		// spawn callback functions
		if(event==press_expected_event){
			if(press_cb[cfg.pin]!=NULL){
				rc_pthread_create(&press_thread,(void * (*)(void *))press_cb[cfg.pin],NULL,SCHED_OTHER, 0);
			}
		}
		else if(event==release_expected_event){
			if(release_cb[cfg.pin]!=NULL){
				rc_pthread_create(&release_thread,(void * (*)(void *))release_cb[cfg.pin],NULL,SCHED_OTHER, 0);
			}
		}
	}

	return NULL;
}


int rc_button_init(int pin, char polarity, int debounce_us)
{
	int i;
	thread_cfg_t cfg;

	// sanity checks
	if(pin<0 || pin>MAX_PINS){
		fprintf(stderr,"ERROR in rc_button_init, pin must be between 0 and %d\n",MAX_PINS);
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

	// basic gpio setup
	if(rc_gpio_init_event(pin,GPIOHANDLE_REQUEST_INPUT,GPIOEVENT_REQUEST_BOTH_EDGES)==-1){
		fprintf(stderr,"ERROR: in rc_button_init, failed to setup GPIO pin\n");
		return -1;
	}

	// set up thread config structs
	cfg.pin = pin;
	cfg.debounce = debounce_us;
	pol[pin] = polarity;

	// start threads
	shutdown_flag=0;
	started[pin]=0;
	if(rc_pthread_create(&poll_thread[pin], poll_thread_func, (void*)&cfg, SCHED_OTHER, 0)){
		fprintf(stderr,"ERROR in rc_button_init, failed to start press handler thread\n");
		return -1;
	}

	// wait for thread to start
	i=0;
	while(started[pin]==0){
		i++;
		if(i>=100){
			fprintf(stderr,"ERROR in rc_button_init, timeout waiting for thread to start\n");
			return -1;
		}
		rc_usleep(1000);
	}

	// set flags
	init_flag[pin]=1;
	return 0;
}


void rc_button_cleanup()
{
	int i, ret;
	// signal threads to close
	shutdown_flag=1;
	// loop through all pins
	for(i=0;i<MAX_PINS;i++){
		// skip uninitialized pins
		if(!init_flag[i]) continue;
		// join thread
		ret=rc_pthread_timed_join(poll_thread[i],NULL,THREAD_TIMEOUT);
		if(ret==-1){
			fprintf(stderr,"WARNING in rc_button_cleanup, problem joining button handler thread for pin %d\n",i);
		}
		else if(ret==1){
			fprintf(stderr,"WARNING in rc_button_cleanup, thread exit timeout for pin %d\n",i);
			fprintf(stderr,"most likely cause is your button press callback function is stuck and didn't return\n");
		}

	}
	return;
}


int rc_button_set_callbacks(int pin, void (*press_func)(void), void (*release_func)(void))
{
	// sanity checks
	if(pin<0 || pin>MAX_PINS){
		fprintf(stderr,"ERROR in rc_button_set_callabcks, pin must be between 0 and %d\n",MAX_PINS);
		return -1;
	}
	press_cb[pin] = press_func;
	release_cb[pin] = release_func;
	return 0;
}


int rc_button_get_state(int pin)
{
	int val,ret;
	if(!init_flag[pin]){
		fprintf(stderr,"ERROR: call to rc_button_get_state without calling rc_button_init first\n");
		return -1;
	}
	val=rc_gpio_get_value(pin);
	if(val==-1){
		fprintf(stderr,"ERROR in rc_button_get_state\n");
		return -1;
	}
	// determine return based on polarity
	if(pol[pin]==RC_BTN_POLARITY_NORM_HIGH){
		if(val) ret=RC_BTN_STATE_RELEASED;
		else ret=RC_BTN_STATE_PRESSED;
	}
	else{
		if(val) ret=RC_BTN_STATE_PRESSED;
		else ret=RC_BTN_STATE_RELEASED;
	}
	return ret;
}




