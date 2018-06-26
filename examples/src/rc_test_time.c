/**
 * @file rc_test_time.c
 * @example    rc_test_time
 *
 * Prints the current version of the robot control library.
 *
 *
 *
 * @author     James Strawson
 * @date       1/29/2018
 */

#include <stdio.h>
#include <inttypes.h> // for PRIu64
#include <rc/time.h>

#define LOOPS 10000

int main()
{
	int i;
	uint64_t a,b,nanos;


	printf("\ntesting rc time functions\n");

	// time rc_nanos_since_epoch
	a=rc_nanos_since_epoch();
	for(i=0;i<LOOPS;i++) b=rc_nanos_since_epoch();
	nanos=(b-a)/LOOPS;
	printf("average time to call rc_nanos_since_epoch: %" PRIu64 "ns\n",nanos);

	// time rc_nanos_since_boot
	a=rc_nanos_since_boot();
	for(i=0;i<LOOPS;i++) b=rc_nanos_since_boot();
	nanos=(b-a)/LOOPS;
	printf("average time to call rc_nanos_since_boot: %" PRIu64 "ns\n",nanos);

	// time rc_nanos_thread_time
	a=rc_nanos_thread_time();
	for(i=0;i<LOOPS;i++) b=rc_nanos_thread_time();
	nanos=(b-a)/LOOPS;
	printf("average time to call rc_nanos_thread_time: %" PRIu64 "ns\n",nanos);

	return 0;
}
