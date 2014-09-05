// mmap_eqep.c
// prints data about PWMSS and EQEP using mmap memory access
// also continuously prints out eQep encoder positions
// James Strawson 2014

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

#include "tipwmss.h"

#define PWM0_BASE   0x48300000
#define PWM1_BASE   0x48302000
#define PWM2_BASE   0x48304000
#define EQEP_OFFSET  0x180

volatile char *pwm_map_base[3];

long int get_encoder_pos(int ch){
	if(ch>2 || ch<0){
		printf("Encoder Channel must be in [0,2]\n");
		exit(0);
	}
	return  *(unsigned long*)(pwm_map_base[ch] + EQEP_OFFSET +QPOSCNT);
}

int set_encoder_pos(int ch, long value){
	if(ch>2 || ch<0){
		printf("Encoder Channel must be in [0,2]\n");
		return -1;
	}
	*(unsigned long*)(pwm_map_base[ch] + EQEP_OFFSET +QPOSCNT) = value;
	return 0;
}

int main(){
	int fd;
	int i;

	if ((fd = open("/dev/mem", O_RDWR | O_SYNC))==-1){
	  printf("Could not open /dev/mem\n");
	  return 0;
	}
	printf("opened /dev/mem\n");

	pwm_map_base[0]= mmap(0,getpagesize(),PROT_READ|PROT_WRITE,MAP_SHARED,fd,PWM0_BASE);
	pwm_map_base[1]= mmap(0,getpagesize(),PROT_READ|PROT_WRITE,MAP_SHARED,fd,PWM1_BASE);
	pwm_map_base[2]= mmap(0,getpagesize(),PROT_READ|PROT_WRITE,MAP_SHARED,fd,PWM2_BASE);
	if(pwm_map_base[0] == (void *) -1) {
		printf("Unable to mmap pwm\n");
		exit(-1);
	}
	close(fd);
	
	//turn off clock to eqep
	*(unsigned long*)(pwm_map_base[0]+PWMSS_CLKCONFIG) &= ~0x010;
	// Write the decoder control settings
	*(unsigned short*)(pwm_map_base[0]+EQEP_OFFSET+QDECCTL) = 0;
	// set maximum position to two's compliment of -1, aka UINT_MAX
	*(unsigned long*)(pwm_map_base[0]+EQEP_OFFSET+QPOSMAX)=-1;
	// Enable interrupt
	*(unsigned short*)(pwm_map_base[0]+EQEP_OFFSET+QEINT) = UTOF;
	// set unit period register
	*(unsigned long*)(pwm_map_base[0]+EQEP_OFFSET+QUPRD)=0x5F5E100;
	// enable counter in control register
	*(unsigned short*)(pwm_map_base[0]+EQEP_OFFSET+QEPCTL) = PHEN|IEL0|SWI|UTE|QCLM;
	//enable clock from PWMSS
	*(unsigned long*)(pwm_map_base[0]+PWMSS_CLKCONFIG) |= 0x010;
	
	// PWMSS registers
	printf("SYSCONFIG 0x%lX\n", *(unsigned long*)(pwm_map_base[0]+PWMSS_SYSCONFIG));
	printf("CLKCONFIG 0x%lX\n", *(unsigned long*)(pwm_map_base[0]+PWMSS_CLKCONFIG));
	
	// eQep registers
	printf("QPOSMAX0  0x%lX\n", *(unsigned long*)(pwm_map_base[0]+EQEP_OFFSET+QPOSMAX));
	printf("QEPCTL0   0x%X\n",  *(unsigned short*)(pwm_map_base[0]+EQEP_OFFSET+QEPCTL));
	printf("QDECCTL0  0x%X\n",  *(unsigned short*)(pwm_map_base[0]+EQEP_OFFSET+QDECCTL)); 
	printf("QEINT0    0x%X\n",	*(unsigned short*)(pwm_map_base[0]+EQEP_OFFSET+QEINT)); 
	printf("QUPRD0    0x%lX\n", *(unsigned long*)(pwm_map_base[0]+EQEP_OFFSET+QUPRD));
	printf("QUTMR0    0x%lX\n", *(unsigned long*)(pwm_map_base[0]+EQEP_OFFSET+QUTMR));
	printf("QEPSTS0   0x%X\n",  *(unsigned short*)(pwm_map_base[0]+EQEP_OFFSET+QEPSTS)); 
	printf("\n");
	
	set_encoder_pos(0,0);
	set_encoder_pos(1,0);
	set_encoder_pos(2,0);
	
	// print out current eQep position for a while
	for(i=0;i<100000;i++){	
		 printf("\reqep0: %ld eqep1: %ld eqep2: %ld   ",get_encoder_pos(0),get_encoder_pos(1),get_encoder_pos(2)); 
		 fflush(stdout);
		 usleep(10000);
	}
	return 0;
}