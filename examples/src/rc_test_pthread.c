/**
 * @file rc_test_pthread.c
 * @example rc_test_pthread
 * @brief test of pthread helpers from rc/pthread_helpers.h
 *
 * @verbatim
 Usage:
	-p <policy><pri> Set scheduling policy and priority
	                 <policy> can be
	                     f  SCHED_FIFO
	                     r  SCHED_RR
	                     o  SCHED_OTHER
	-d               Use default thread attributes object
	-h               Print this help message

	For example, to run with SCHED_FIFO at priority 50 run:
	 rc_test_pthread -p f 50
 * @endverbatim
 *
 */

#include <stdio.h>
#include <unistd.h> // for sleep
#include <signal.h>
#include <stdlib.h> // for strtol
#include <rc/pthread.h>


static int running = 1;
static pthread_t thread;
static int thread_ret_val;

static void* __thread_func(void* arg)
{
	int received_argument = *(int*)arg;
	printf("thread received argument value: %d\n",received_argument);
	printf("thread has properties: ");
	rc_pthread_print_properties(thread);
	printf("current process niceness: %d\n", rc_pthread_get_process_niceness());
	while(running){
		sleep(1);
		printf("thread running! press ctrl-c to exit\n");
	}
	printf("exiting thread\n");
	thread_ret_val=received_argument;
	return (void*)&thread_ret_val;
}

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}

static void __print_usage(void)
{
	fprintf(stderr, "Usage:\n");

	#define fpe(msg) fprintf(stderr, "\t%s", msg);          /* Shorter */
	fpe("-p <policy><pri> Set scheduling policy and priority\n");
	fpe("                 <policy> can be\n");
	fpe("                     f  SCHED_FIFO\n");
	fpe("                     r  SCHED_RR\n");
	fpe("                     o  SCHED_OTHER\n");
	fpe("-d               Use default thread attributes object\n");
	fpe("-h               Print this help message\n");
	fpe("\n");
	fpe("For example, to run with SCHED_FIFO at priority 50 run:\n");
	fpe(" rc_test_pthread -p f50\n")
	#undef fpe
	return;
}

static int __get_policy(char p, int *policy)
{
	switch (p) {
	case 'f': *policy = SCHED_FIFO;     return 0;
	case 'r': *policy = SCHED_RR;       return 0;
	case 'o': *policy = SCHED_OTHER;    return 0;
	default:
		fprintf(stderr, "invalid policy\n");
		return -1;
	}
}

int main(int argc, char *argv[])
{
	int ret, opt;
	int policy = SCHED_OTHER;
	int priority = 0;
	char *attr_sched_str;
	int use_default=0;
	int use_custom=0;
	int arg=42; // argument sent to the thread
	void* retval; // return value of the thread

	while((opt = getopt(argc, argv, "p:dh")) != -1){
		switch (opt) {
		case 'p':
			attr_sched_str = optarg;
			use_custom = 1;
			break;
		case 'd':
			use_default = 1;
			break;
		case 'h':
			__print_usage();
			break;
		default:
			fprintf(stderr,"invalid option\n");
			__print_usage();
			return -1;
		}
	}

	if(use_custom && use_default){
		fprintf(stderr, "ERROR: can't use custom and default properties\n");
		return -1;
	}
	if(!use_custom && !use_default){
		fprintf(stderr, "one argument must be given\n");
		__print_usage();
		return -1;
	}
	if(use_custom){
		if(__get_policy(attr_sched_str[0], &policy)){
			__print_usage();
			return -1;
		}
		priority = strtol(&attr_sched_str[1], NULL, 0);
	}

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);

	// start thread
	printf("starting thread with argument: %d\n",arg);
	if(rc_pthread_create(&thread, __thread_func, (void*)&arg, policy, priority)){
		fprintf(stderr, "failed to start thread\n");
		return -1;
	}

	// wait for shutdown signal
	while(running)	sleep(1);

	// join thread with 1.5s timeout
	ret=rc_pthread_timed_join(thread, &retval, 1.5);
	if(ret==1) fprintf(stderr,"joining thread timed out\n");
	printf("pthread returned:%d\n",*(int*)retval);

	return 0;

}
