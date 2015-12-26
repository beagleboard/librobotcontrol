/*
Copyright (c) 2014, James Strawson
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*/

//#define DEBUG

#include "mmap_pwmss.h"
#include "tipwmss.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>

#define DEFAULT_FREQ 40000 // 40khz pwm freq
#define DEFAULT_DIVIDER 4  // clock divider

volatile char *cm_per_base;
int cm_per_mapped=0;
volatile char *pwm_base[3]; // pwm subsystem pointers for eQEP
int pwmss_mapped[3] = {0,0,0}; // to record which subsystems have been mapped
int eqep_initialized[3] = {0,0,0};
int pwm_initialized[3] = {0,0,0};
uint16_t period[3] = {0,0,0}; // real world period, not tb_prd
int divider[3] = {0,0,0}; 	// these get set in initialize_pwm

/********************************************
*  PWMSS Mapping
*********************************************/
// maps the base of each PWM subsystem into an array
// this is used by eQEP and PWM
// returns immediately if this has already been done 
int map_pwmss(int ss){
	if(ss>2 || ss<0){
		printf("error: PWM subsystem must be 0, 1, or 2\n");
		return -1;
	}
	//return 0 if it's already been mapped.
	if(pwmss_mapped[ss]){
		return 0;
	}
	
	//open /dev/mem file pointer for mmap
	#ifdef DEBUG
		printf("opening /dev/mem\n");
	#endif
	int dev_mem;
	if ((dev_mem = open("/dev/mem", O_RDWR | O_SYNC))==-1){
	  printf("Could not open /dev/mem \n");
	  return -1;
	}
	
	// first open the clock register to see if the PWMSS has clock enabled
	if(!cm_per_mapped){
		#ifdef DEBUG
		printf("mapping CM_PER\n");
		#endif
		cm_per_base=mmap(0,CM_PER_PAGE_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,dev_mem,CM_PER);
		if(cm_per_base == (void *) -1) {
			printf("Unable to mmap cm_per\n");
			return -1;
		}
		cm_per_mapped = 1;
	}
	
	// if this subsystem hasn't already been mapped, 
	// then we probably need to enable clock signal to it in cm_per
	uint32_t cm_per_clkctrl;
	switch(ss){
	case 0:
		cm_per_clkctrl = CM_PER_EPWMSS0_CLKCTRL;
		break;
	case 1:
		cm_per_clkctrl = CM_PER_EPWMSS1_CLKCTRL;
		break;
	case 2:
		cm_per_clkctrl = CM_PER_EPWMSS2_CLKCTRL;
		break;
	default:
		return -1;
	}
	
	*(uint16_t*)(cm_per_base + cm_per_clkctrl) |= MODULEMODE_ENABLE;
	#ifdef DEBUG
	printf("new clkctrl%d: %d\n", ss, *(uint16_t*)(cm_per_base + cm_per_clkctrl));
	#endif

	
	// now map the appropriate subsystem base address
	#ifdef DEBUG
		printf("calling mmap() for base %d\n", ss);
	#endif
	switch(ss){
	case 0:
		pwm_base[0] = mmap(0,PWMSS_MEM_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,dev_mem,PWMSS0_BASE);
		break;
	case 1:
		pwm_base[1] = mmap(0,PWMSS_MEM_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,dev_mem,PWMSS1_BASE);
		break;
	case 2:
		pwm_base[2] = mmap(0,PWMSS_MEM_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,dev_mem,PWMSS2_BASE);
		break;
	default:
		printf("invalid ss\n");
		return -1;
	}
	#ifdef DEBUG
		printf("finished mapping for base %d\n", ss);
	#endif
	
	if(pwm_base[ss] == (void *) -1) {
		printf("Unable to mmap pwm \n");
		return -1;
	}
	pwmss_mapped[ss]=1;
	
	// //enable clock from PWMSS
	*(uint32_t*)(pwm_base[ss]+PWMSS_CLKCONFIG) |= 0x010;
	
	close(dev_mem);
	#ifdef DEBUG
		printf("closed /dev/mem\n");
	#endif
	return 0;
}

/********************************************
*  eQEP
*********************************************/

// init_eqep takes care of sanity checks and returns quickly
// if nothing is to be initialized.
int init_eqep(int ss){
	// range sanity check
	if(ss>2 || ss<0){
		printf("error: PWM subsystem must be 0, 1, or 2\n");
		return -1;
	}
	// see if eQEP already got initialized
	if(eqep_initialized[ss]){
		return 0;
	}
	// make sure the subsystem is mapped
	if(map_pwmss(ss)){
		printf("failed to map PWMSS %d\n", ss);
		return -1;
	}
	#ifdef DEBUG
		printf("setting eqep ctrl registers\n");
	#endif
	//turn off clock to eqep
	*(uint32_t*)(pwm_base[ss]+PWMSS_CLKCONFIG) &= ~PWMSS_EQEPCLK_EN;
	// Write the decoder control settings
	*(uint16_t*)(pwm_base[ss]+EQEP_OFFSET+QDECCTL) = 0;
	// set maximum position to two's compliment of -1, aka UINT_MAX
	*(uint32_t*)(pwm_base[ss]+EQEP_OFFSET+QPOSMAX)=-1;
	// Enable interrupt
	*(uint16_t*)(pwm_base[ss]+EQEP_OFFSET+QEINT) = UTOF;
	// set unit period register
	*(uint32_t*)(pwm_base[ss]+EQEP_OFFSET+QUPRD)=0x5F5E100;
	// enable counter in control register
	*(uint16_t*)(pwm_base[ss]+EQEP_OFFSET+QEPCTL) = PHEN|IEL0|SWI|UTE|QCLM;
	//enable clock from PWMSS
	*(uint32_t*)(pwm_base[ss]+PWMSS_CLKCONFIG) |= PWMSS_EQEPCLK_EN;
	
	// Test eqep by resetting position
	#ifdef DEBUG
		printf("testing eQEP write\n");
	#endif
	*(uint32_t*)(pwm_base[ss] + EQEP_OFFSET +QPOSCNT) = 0;
	#ifdef DEBUG
		printf("successfully tested eQEP write\n");
	#endif
	eqep_initialized[ss] = 1;
	return 0;
}

// read a value from eQEP counter
int read_eqep(int ch){
	if(init_eqep(ch)) return -1;
	return  *(int*)(pwm_base[ch-1] + EQEP_OFFSET +QPOSCNT);
}

// write a value to the eQEP counter
int write_eqep(int ch, int val){
	if(init_eqep(ch)) return -1;
	*(int*)(pwm_base[ch] + EQEP_OFFSET +QPOSCNT) = val;
	return 0;
}

/****************************************************************
* PWM
* we use up-count mode to generate an asymmetric PWM
* same frequency for each pair on signals in each subsystem
* refer to page 1523 of AM335x TRM
*****************************************************************/
int init_pwm(int ss){
	if(ss>2 || ss<0){
		printf("pwm subsystem must be 0,1, or 2\n");
		return -1;
	}
	if(pwm_initialized[ss]){
		return 0;
	}
	if(map_pwmss(ss)){
		printf("failed to map PWMSS\n");
		return -1;
	}
	
	// set up the divider bits to match desired divider ratio
	divider[ss] = DEFAULT_DIVIDER;
	uint16_t div_bits;
	switch(divider[ss]){
		case 1:
			div_bits=TB_DIV1;
			break;
		case 2:
			div_bits=TB_DIV2;
			break;
		case 4:
			div_bits=TB_DIV4;
			break;
		case 8:
			div_bits=TB_DIV8;
			break;
		case 16:
			div_bits=TB_DIV16;
			break;
		case 32:
			div_bits=TB_DIV32;
			break;
		case 64:
			div_bits=TB_DIV64;
			break;
		case 128:
			div_bits=TB_DIV128;
			break;
		default:
			printf("invalid divider\n");
			return -1;
	}
	
	//disable clock to EPWM
	*(uint32_t*)(pwm_base[ss]+PWMSS_CLKCONFIG) &= ~PWMSS_EPWMCLK_EN;
	// set period to default for divider=4
	int new_period = (1000000000/(DEFAULT_FREQ*divider[ss]));
    *(uint16_t*)(pwm_base[ss]+PWM_OFFSET+TBPRD) = new_period-1;
	period[ss] = new_period;
	//set phase to 0
	*(uint16_t*)(pwm_base[ss]+PWM_OFFSET+TBPHS) = 0;
	// clear TB counter
	*(uint16_t*)(pwm_base[ss]+PWM_OFFSET+TBCNT) = 0;
	// disable phase loading and set clock divider to 4
	*(uint16_t*)(pwm_base[ss]+PWM_OFFSET+TBCTL) \
		= TB_UP||TB_DISABLE||TB_SHADOW||TB_SYNC_DISABLE||div_bits||TB_HDIV1;
	// set compare registers to 0
	*(uint16_t*)(pwm_base[ss]+PWM_OFFSET+CMPA) = 0;
	*(uint16_t*)(pwm_base[ss]+PWM_OFFSET+CMPB) = 0;
	// set comparator to toggle on at TBCNT = 0;
	*(uint16_t*)(pwm_base[ss]+PWM_OFFSET+CMPCTL) \
		=CC_SHADOW_A||CC_SHADOW_B||CC_CTR_ZERO_A||CC_CTR_ZERO_B;
	// don't use Action Qualifier Dead Band
	*(uint16_t*)(pwm_base[ss]+PWM_OFFSET+AQ_CTLA) \
		= AQ_ZRO_SET||AQ_CAU_CLEAR;
	*(uint16_t*)(pwm_base[ss]+PWM_OFFSET+AQ_CTLB) \
		= AQ_ZRO_SET||AQ_CBU_CLEAR;
	// Enable Clock
	*(uint32_t*)(pwm_base[ss]+PWMSS_CLKCONFIG) |= PWMSS_EPWMCLK_EN;
	
	pwm_initialized[ss]=1;
	return 0;
}


// set PWM frequency for a subsystem, this effects both channels A & B
int set_pwm_freq(int ss, int hz){
	// sanity check initialization
	if(init_pwm(ss)) return -1;
	
	// period = (TBPRD+1) * TBCLK
	// we set divider to 1 to TBCLK = SYSCLK (1ghz)
	int new_period = (1000000000/(hz*divider[ss]));
	// check that the period isn't higher than the 16-bit register can hold
	if(new_period>65535){
		int min_hz = (1000000000/(65535*divider[ss]));
		printf("frequency must be faster than %dhz\n", min_hz);
		printf("you may change the pwm clock divider in init_pwm() if needed\n");
		return -1;
	}
	period[ss] = new_period;
	// set duty cycle to 0 before changing frequency
	*(uint16_t*)(pwm_base[ss]+PWM_OFFSET+CMPA) = 0;
	*(uint16_t*)(pwm_base[ss]+PWM_OFFSET+CMPA) = 0;
	// finally set period
	*(uint16_t*)(pwm_base[ss]+PWM_OFFSET+TBPRD) = period[ss]-1;
	return 0;
}

// set duty cycle for either channel A or B in a given subsystem
// input channel is a character 'A' or 'B'
int set_pwm_duty(int ss, char ch, float duty){
	if(init_pwm(ss)) return -1;
	//sanity check duty
	if(duty>1.0 || duty<0.0){
		printf("duty must be between 0.0 & 1.0\n");
		return -1;
	}
	uint16_t new = (uint16_t)lroundf(duty * period[ss]);
	
	// change appropriate compare register
	// from 0 to TBPRD+1 to achieve 0-100% PWM duty
	switch(ch){
	case 'A':
		*(uint16_t*)(pwm_base[ss]+PWM_OFFSET+CMPA) = new;
		break;
	case 'B':
		*(uint16_t*)(pwm_base[ss]+PWM_OFFSET+CMPB) = new;
		break;
	default:
		printf("pwm channel must be 'A' or 'B'\n");
		return -1;
	}
	
	return 0;
}


