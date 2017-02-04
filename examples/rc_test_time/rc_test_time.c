/*******************************************************************************
* rc_test_time.c
*
* James Strawson 2016
* This is meant to be a skeleton program for robotics cape projects. 
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

#define LOOPS 10000
int main(){
	int i;
	uint64_t a,b,nanos;
	
	// set clock speed to 1000mhz to make sure scaling doesn't effect results
	rc_set_cpu_freq(FREQ_1000MHZ);
	
	printf("\ntesting rc time functions\n");
	
	// time rc_nanos_since_epoch
	a=rc_nanos_since_epoch();
	for(i=0;i<LOOPS;i++) b=rc_nanos_since_epoch();
	nanos=(b-a)/LOOPS;
	printf("time to call rc_nanos_since_epoch: %lldns\n",nanos);
	
	// time rc_nanos_since_boot
	a=rc_nanos_since_boot();
	for(i=0;i<LOOPS;i++) b=rc_nanos_since_boot();
	nanos=(b-a)/LOOPS;
	printf("time to call rc_nanos_since_boot: %lldns\n",nanos);
	
	// time rc_nanos_thread_time
	a=rc_nanos_thread_time();
	for(i=0;i<LOOPS;i++) b=rc_nanos_thread_time();
	nanos=(b-a)/LOOPS;
	printf("time to call rc_nanos_thread_time: %lldns\n",nanos);
	
	rc_set_cpu_freq(FREQ_ONDEMAND);
	return 0;
}
