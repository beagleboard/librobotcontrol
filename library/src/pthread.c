
#define _GNU_SOURCE // to allow pthread_timedjoin_np

#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <sys/resource.h> // for get/set priority niceness
#include <sys/types.h>	// for getpid
#include <unistd.h>	// for getpid

#include <rc/pthread.h>


int rc_pthread_create(pthread_t *thread, void*(*func)(void*), void* arg, int policy, int priority)
{
	pthread_attr_t pthread_attr;
	struct sched_param pthread_param;

	// sanity checks
	if(policy!=SCHED_FIFO && policy!=SCHED_RR && policy!=SCHED_OTHER){
		fprintf(stderr,"ERROR in rc_pthread_create: policy must be SCHED_FIFO, SCHED_RR, or SCHED_OTHER\n");
		return -1;
	}
	if(thread==NULL || func==NULL){
		fprintf(stderr,"ERROR in rc_pthread_create: received NULL pointer\n");
		return -1;
	}

	// necessary attribute initialization
	pthread_attr_init(&pthread_attr);

	// if user is requesting anything other than inherited policy 0 and
	// priority 0, make sure we have permission to do explicit scheduling
	if(priority!=0 || policy!=SCHED_OTHER){
		// print warning if no permissions
		errno = pthread_attr_setinheritsched(&pthread_attr, PTHREAD_EXPLICIT_SCHED);
		if(errno){
			perror("ERROR: pthread_attr_setinheritsched: ");
			return -1;
		}
	}

	// set scheduling policy
	const int max_pri = sched_get_priority_max(policy);
	const int min_pri = sched_get_priority_min(policy);
	if(priority>max_pri || priority<min_pri){
		fprintf(stderr,"ERROR in rc_pthread_create, priority must be between %d & %d\n",min_pri,max_pri);
		return -1;
	}

	// set policy to attributes
	errno = pthread_attr_setschedpolicy(&pthread_attr, policy);
	if(errno){
		perror("ERROR: pthread_attr_setschedpolicy");
			return -1;
	}

	// set priority in attributes
	pthread_param.sched_priority = priority;
	errno = pthread_attr_setschedparam(&pthread_attr, &pthread_param);
	if(errno){
		perror("ERROR: pthread_attr_setschedparam");
		return -1;
	}

	// create the thread
	errno=pthread_create(thread, &pthread_attr, func, arg);
	if(errno==EPERM){
		fprintf(stderr,"WARNING: in rc_pthread_create, insufficient privileges to set scheduling policy\n");
		fprintf(stderr,"starting thread with inherited scheduling policy instead\n");
		fprintf(stderr,"to silence this warning, call with policy=SCHED_OTHER & priority=0\n");
		policy=SCHED_OTHER;
		priority=0;
		errno=pthread_create(thread, NULL, func, arg);
		if(errno!=0){
			perror("ERROR: in rc_pthread_create ");
			pthread_attr_destroy(&pthread_attr);
			return -1;
		}
	}
	else if(errno){
		perror("ERROR: in rc_pthread_create: ");
		pthread_attr_destroy(&pthread_attr);
		return -1;
	}

	// check if it worked
	int policy_new;
	struct sched_param params_new;
	// get parameters from pthread_t
	errno = pthread_getschedparam(*thread, &policy_new, &params_new);
	if(errno){
		perror("ERROR: pthread_getschedparam");
		return -1;
	}

	// print warnings if there is a mismatch
	if(policy_new!=policy){
		fprintf(stderr,"WARNING in rc_pthread_create, policy actually got set to %d, expected %d\n", \
				policy_new, policy);
	}
	if(params_new.sched_priority!=priority){
		fprintf(stderr,"WARNING in rc_pthread_create, priority actually got set to %d, expected %d\n", \
				params_new.sched_priority, priority);
	}

	// destroy the attributes object
	if(pthread_attr_destroy(&pthread_attr)){
		fprintf(stderr,"WARNING, failed to destroy pthread_attr\n");
	}

	return 0;
}

int rc_pthread_timed_join(pthread_t thread, void** retval, float timeout_sec){
	struct timespec thread_timeout;
	clock_gettime(CLOCK_REALTIME, &thread_timeout);
	uint64_t timeout_ns = timeout_sec*1000000000;
	thread_timeout.tv_sec += timeout_ns/1000000000;
	thread_timeout.tv_nsec += timeout_ns%1000000000;
	if(thread_timeout.tv_nsec>1000000000){
		thread_timeout.tv_sec += 1;
		thread_timeout.tv_nsec -= 1000000000;
	}

	errno = pthread_timedjoin_np(thread,retval,&thread_timeout);
	// if no error, return 0
	if(errno==0) return 0;
	// in case of timeout, return 1
	if(errno==ETIMEDOUT) return 1;
	// otherwise print error message
	perror("ERROR: in rc_pthread_timed_join: ");
	return -1;
}

int rc_pthread_print_properties(pthread_t thread)
{
	int policy;
	struct sched_param params;
	// get parameters from pthread_t
	if(pthread_getschedparam(thread, &policy, &params)){
		perror("ERROR: pthread_getschedparam");
		return -1;
	}
	printf("policy=%s, priority=%d\n",
		(policy == SCHED_FIFO)  ? "SCHED_FIFO" :
		(policy == SCHED_RR)    ? "SCHED_RR" :
		(policy == SCHED_OTHER) ? "SCHED_OTHER" :
		"???",
		params.sched_priority);
	return 0;
}

int rc_pthread_set_properties_self(int policy, int priority)
{
	struct sched_param params;
	params.sched_priority = priority;
	errno = pthread_setschedparam(pthread_self(), policy, &params);
	if(errno){
		perror("ERROR in rc_pthread_set_properties_self: ");
		return -1;
	}
	return 0;
}



int rc_pthread_get_process_niceness(void)
{
	int which = PRIO_PROCESS;
	id_t pid;
	int ret;
	pid = getpid();
	errno=0;
	ret = getpriority(which, pid);
	if(errno) perror("ERROR in rc_pthread_get_process_niceness: ");
	return ret;
}

int rc_pthread_set_process_niceness(int niceness)
{
	int which = PRIO_PROCESS;
	id_t pid;
	int ret;
	pid = getpid();
	errno=0;
	ret = setpriority(which, pid, niceness);
	if(errno) perror("ERROR in rc_pthread_set_process_niceness: ");
	return ret;
}