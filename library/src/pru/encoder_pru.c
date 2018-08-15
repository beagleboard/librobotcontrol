/**
 * @file encoder_pru.c
 *
 * @author     James Strawson
 * @date       3/7/2018
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <rc/pru.h>
#include <rc/time.h>
#include <rc/encoder_pru.h>

#define ENCODER_PRU_CH		0 // PRU0
#define ENCODER_PRU_FW		"am335x-pru0-rc-encoder-fw"
#define ENCODER_MEM_OFFSET	16

// pru shared memory pointer
static volatile unsigned int* shared_mem_32bit_ptr = NULL;
static int init_flag=0;

int rc_encoder_pru_init(void)
{
	int i;
	// map memory
	shared_mem_32bit_ptr = rc_pru_shared_mem_ptr();
	if(shared_mem_32bit_ptr==NULL){
		fprintf(stderr, "ERROR in rc_encoder_pru_init, failed to map shared memory pointer\n");
		init_flag=0;
		return -1;
	}
	// set first channel to be nonzero, PRU binary will zero this out later
	shared_mem_32bit_ptr[ENCODER_MEM_OFFSET]=42;

	// start pru
	if(rc_pru_start(ENCODER_PRU_CH, ENCODER_PRU_FW)){
		fprintf(stderr,"ERROR in rc_encoder_pru_init, failed to start PRU%d\n", ENCODER_PRU_CH);
		return -1;
	}

	// make sure memory actually got zero'd out
	for(i=0;i<40;i++){
		if(shared_mem_32bit_ptr[ENCODER_MEM_OFFSET]==0){
			init_flag=1;
			return 0;
		}
		rc_usleep(100000);
	}

	fprintf(stderr, "ERROR in rc_encoder_pru_init, %s failed to load\n", ENCODER_PRU_FW);
	fprintf(stderr, "attempting to stop PRU%d\n", ENCODER_PRU_CH);
	rc_pru_stop(ENCODER_PRU_CH);
	init_flag=0;
	return -1;
}


void rc_encoder_pru_cleanup(void)
{
	// zero out shared memory
	if(shared_mem_32bit_ptr != NULL){
		shared_mem_32bit_ptr[ENCODER_MEM_OFFSET]=0;
	}
	rc_pru_stop(ENCODER_PRU_CH);
	shared_mem_32bit_ptr = NULL;
	init_flag=0;
	return;
}


int rc_encoder_pru_read(void)
{
	if(shared_mem_32bit_ptr==NULL || init_flag==0){
		fprintf(stderr, "ERROR in rc_encoder_pru_read, call rc_encoder_pru_init first\n");
		return -1;
	}
	return (int) shared_mem_32bit_ptr[ENCODER_MEM_OFFSET];
}


int rc_encoder_pru_write(int pos)
{
	if(shared_mem_32bit_ptr==NULL || init_flag==0){
		fprintf(stderr, "ERROR in rc_encoder_pru_write, call rc_encoder_pru_init first\n");
		return -1;
	}
	shared_mem_32bit_ptr[ENCODER_MEM_OFFSET] = pos;
	return 0;
}
