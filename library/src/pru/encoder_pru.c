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
#include <rc/model.h>

#define AM335X_ENCODER_PRU_CH		0 // PRU0
#define AM335X_ENCODER_PRU_FW		"am335x-pru0-rc-encoder-fw"
#define AM57XX_ENCODER_PRU_CH		3 // PRU1_1
#define AM57XX_ENCODER_PRU_FW		"am57xx-pru1_1-rc-encoder-fw"
#define ENCODER_MEM_OFFSET		16

// pru shared memory pointer
static volatile unsigned int* shared_mem_32bit_ptr = NULL;
static int init_flag=0;

int rc_encoder_pru_init(void)
{
	return 0; // until we fix for AM5
	int i;
	// map memory
	if(rc_model()==MODEL_BB_AI || rc_model()==MODEL_BB_AI_RC)
		shared_mem_32bit_ptr = rc_pru_shared_mem_ptr(AM57XX_ENCODER_PRU_CH);
	else
		shared_mem_32bit_ptr = rc_pru_shared_mem_ptr(AM335X_ENCODER_PRU_CH);
	if(shared_mem_32bit_ptr==NULL){
		fprintf(stderr, "ERROR in rc_encoder_pru_init, failed to map shared memory pointer\n");
		init_flag=0;
		return -1;
	}
	// set first channel to be nonzero, PRU binary will zero this out later
	shared_mem_32bit_ptr[ENCODER_MEM_OFFSET]=42;

	// start pru
	if(rc_model()==MODEL_BB_AI || rc_model()==MODEL_BB_AI_RC){
		if(rc_pru_start(AM57XX_ENCODER_PRU_CH, AM57XX_ENCODER_PRU_FW)){
			fprintf(stderr,"ERROR in rc_encoder_pru_init, failed to start PRU%d\n", AM57XX_ENCODER_PRU_CH);
			return -1;
		}
	} else {
		if(rc_pru_start(AM335X_ENCODER_PRU_CH, AM335X_ENCODER_PRU_FW)){
			fprintf(stderr,"ERROR in rc_encoder_pru_init, failed to start PRU%d\n", AM335X_ENCODER_PRU_CH);
			return -1;
		}
	}

	// make sure memory actually got zero'd out
	for(i=0;i<40;i++){
		if(shared_mem_32bit_ptr[ENCODER_MEM_OFFSET]==0){
			init_flag=1;
			return 0;
		}
		rc_usleep(100000);
	}

	if(rc_model()==MODEL_BB_AI || rc_model()==MODEL_BB_AI_RC){
		fprintf(stderr, "ERROR in rc_encoder_pru_init, %s failed to load\n", AM57XX_ENCODER_PRU_FW);
		fprintf(stderr, "attempting to stop PRU%d\n", AM57XX_ENCODER_PRU_CH);
		rc_pru_stop(AM57XX_ENCODER_PRU_CH);
	} else {
		fprintf(stderr, "ERROR in rc_encoder_pru_init, %s failed to load\n", AM335X_ENCODER_PRU_FW);
		fprintf(stderr, "attempting to stop PRU%d\n", AM335X_ENCODER_PRU_CH);
		rc_pru_stop(AM335X_ENCODER_PRU_CH);
	}
	init_flag=0;
	return -1;
}


void rc_encoder_pru_cleanup(void)
{
	// zero out shared memory
	if(shared_mem_32bit_ptr != NULL){
		shared_mem_32bit_ptr[ENCODER_MEM_OFFSET]=0;
	}
	if(rc_model()==MODEL_BB_AI || rc_model()==MODEL_BB_AI_RC)
		rc_pru_stop(AM57XX_ENCODER_PRU_CH);
	else
		rc_pru_stop(AM335X_ENCODER_PRU_CH);
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
