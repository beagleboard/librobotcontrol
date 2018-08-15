/**
 * @file encoder.c
 */

#include <stdio.h>
#include <rc/encoder.h>
#include <rc/encoder_pru.h>
#include <rc/encoder_eqep.h>


int rc_encoder_init(void)
{
	if(rc_encoder_eqep_init()){
		fprintf(stderr,"ERROR: failed to run rc_encoder_eqep_init\n");
		return -1;
	}
	if(rc_encoder_pru_init()){
		fprintf(stderr,"ERROR: failed to run rc_encoder_pru_init\n");
		return -1;
	}
	return 0;
}

int rc_encoder_cleanup(void)
{
	rc_encoder_eqep_cleanup();
	rc_encoder_pru_cleanup();
	return 0;
}



int rc_encoder_read(int ch)
{
	// sanity check
	if(ch<1 || ch >4){
		fprintf(stderr, "ERROR in rc_encoder_read, channel must be between 1 and 4\n");
		return -1;
	}
	if(ch==4) return rc_encoder_pru_read();
	return rc_encoder_eqep_read(ch);
}

int rc_encoder_write(int ch, int value)
{
	// sanity check
	if(ch<1 || ch >4){
		fprintf(stderr, "ERROR in rc_encoder_write, channel must be between 1 and 4\n");
		return -1;
	}
	if(ch==4) return rc_encoder_pru_write(value);
	return rc_encoder_eqep_write(ch,value);
}

