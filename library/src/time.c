/**
 * @file time.h
 * @brief      Handy sleep and timing functions.
 *
 *             All functions are POSIX compliant and should work on any linux
 *             system.
 *
 * @author     James Strawson
 * @date       1/31/2018
 */

#include <time.h>
#include <sys/time.h> // for timeval
#include <errno.h>
#include <unistd.h> // for sysconf
#include <stdint.h> // for uint64_t

#include <rc/time.h>


void rc_nanosleep(uint64_t ns){
	struct timespec req,rem;
	req.tv_sec = ns/1000000000;
	req.tv_nsec = ns%1000000000;
	// loop untill nanosleep sets an error or finishes successfully
	errno=0; // reset errno to avoid false detection
	while(nanosleep(&req, &rem) && errno==EINTR){
		req.tv_sec = rem.tv_sec;
		req.tv_nsec = rem.tv_nsec;
	}
	return;
}


void rc_usleep(unsigned int us){
	rc_nanosleep(us*1000);
	return;
}

uint64_t rc_nanos_since_epoch(void){
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return ((uint64_t)ts.tv_sec*1000000000)+ts.tv_nsec;
}

uint64_t rc_nanos_since_boot(void){
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ((uint64_t)ts.tv_sec*1000000000)+ts.tv_nsec;
}

uint64_t rc_nanos_thread_time(void){
	struct timespec ts;
	clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
	return ((uint64_t)ts.tv_sec*1000000000)+ts.tv_nsec;
}

uint64_t rc_timespec_to_micros(struct timespec ts){
	return (((uint64_t)ts.tv_sec*1000000000)+ts.tv_nsec)/1000;
}


uint64_t rc_timespec_to_millis(timespec ts){
	return (((uint64_t)ts.tv_sec*1000000000)+ts.tv_nsec)/1000000;
}


uint64_t rc_timeval_to_micros(timeval tv){
	return ((uint64_t)tv.tv_sec*1000000)+tv.tv_usec;
}


uint64_t rc_timeval_to_millis(timeval tv){
	return (((uint64_t)tv.tv_sec*1000000)+tv.tv_usec)/1000;
}


timespec rc_timespec_diff(timespec A, timespec B){
	timespec temp;
	// check which is greater and flip if necessary
	// For the calculation we want A>B
	if(B.tv_sec>A.tv_sec){
		temp = A;
		A = B;
		B = temp;
	} else if (B.tv_sec==A.tv_sec && B.tv_nsec>A.tv_nsec){
		temp = A;
		A = B;
		B = temp;
	}
	// now calculate the difference
	if((A.tv_nsec-B.tv_nsec)<0) {
		temp.tv_sec = A.tv_sec-B.tv_sec-1;
		temp.tv_nsec = 1000000000L+A.tv_nsec-B.tv_nsec;
	}else{
		temp.tv_sec = A.tv_sec-B.tv_sec;
		temp.tv_nsec = A.tv_nsec-B.tv_nsec;
	}
	return temp;
}

void rc_timespec_add(timespec* start, double seconds){
	int s = (int)seconds; // round down by truncation
	start->tv_sec += s;
	start->tv_nsec += (seconds-s)*1000000000;

	if (start->tv_nsec>=1000000000){
		start->tv_nsec -= 1000000000;
		start->tv_sec += 1;
	}
	else if (start->tv_nsec<0){
		start->tv_nsec += 1000000000;
		start->tv_sec -= 1;
	}

	return;
}


