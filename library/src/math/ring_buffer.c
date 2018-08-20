/**
 * @file math/ring_buffer.c
 *
 * @brief      Ring buffer implementation for double-precision doubles
 *
 *             Ring buffers are FIFO (first in first out) buffers of fixed
 *             length which efficiently boot out the oldest value when full.
 *             They are particularly well suited for storing the last n values
 *             in a discrete time filter.
 *
 *             The user creates their own instance of a buffer and passes a
 *             pointer to the these ring_buf functions to perform normal
 *             operations.
 *
 * @author     James Strawson
 * @date       2016
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <rc/math/ring_buffer.h>

#include "algebra_common.h"



rc_ringbuf_t rc_ringbuf_empty(void)
{
	rc_ringbuf_t out = RC_RINGBUF_INITIALIZER;
	return out;
}


int rc_ringbuf_alloc(rc_ringbuf_t* buf, int size)
{
	// sanity checks
	if(unlikely(buf==NULL)){
		fprintf(stderr,"ERROR in rc_ringbuf_alloc, received NULL pointer\n");
		return -1;
	}
	if(unlikely(size<2)){
		fprintf(stderr,"ERROR in rc_ringbuf_alloc, size must be >=2\n");
		return -1;
	}
	// if it's already allocated, nothing to do
	if(buf->initialized && buf->size==size && buf->d!=NULL) return 0;
	// make sure it's zero'd out
	buf->size = 0;
	buf->index = 0;
	buf->initialized = 0;
	// free memory and allocate fresh
	free(buf->d);
	buf->d = (double*)calloc(size,sizeof(double));
	if(buf->d==NULL){
		fprintf(stderr,"ERROR in rc_ringbuf_alloc, failed to allocate memory\n");
		return -1;
	}
	// write out other details
	buf->size = size;
	buf->initialized = 1;
	return 0;
}


int rc_ringbuf_free(rc_ringbuf_t* buf)
{
	rc_ringbuf_t new = RC_RINGBUF_INITIALIZER;
	if(unlikely(buf==NULL)){
		fprintf(stderr, "ERROR in rc_ringbuf_free, received NULL pointer\n");
		return -1;
	}
	if(buf->initialized) free(buf->d);
	*buf = new;
	return 0;
}


int rc_ringbuf_reset(rc_ringbuf_t* buf)
{
	// sanity checks
	if(unlikely(buf==NULL)){
		fprintf(stderr, "ERROR in rc_ringbuf_reset, received NULL pointer\n");
		return -1;
	}
	if(unlikely(!buf->initialized)){
		fprintf(stderr,"ERROR rc_ringbuf_reset, ringbuf uninitialized\n");
		return -1;
	}
	// wipe the data and index
	memset(buf->d,0,buf->size*sizeof(double));
	buf->index=0;
	return 0;
}


int rc_ringbuf_insert(rc_ringbuf_t* buf, double val)
{
	int new_index;
	// sanity checks
	if(unlikely(buf==NULL)){
		fprintf(stderr,"ERROR in rc_ringbuf_insert, received NULL pointer\n");
		return -1;
	}
	if(unlikely(!buf->initialized)){
		fprintf(stderr,"ERROR in rc_ringbuf_insert, ringbuf uninitialized\n");
		return -1;
	}
	// increment index and check for loop-around
	new_index=buf->index+1;
	if(new_index>=buf->size) new_index=0;
	// write out new value
	buf->d[new_index]=val;
	buf->index=new_index;
	return 0;
}


double rc_ringbuf_get_value(rc_ringbuf_t* buf, int pos)
{
	int return_index;
	// sanity checks
	if(unlikely(buf==NULL)){
		fprintf(stderr,"ERROR in rc_ringbuf_get_value, received NULL pointer\n");
		return -1.0f;
	}
	if(unlikely(pos<0 || pos>buf->size-1)){
		fprintf(stderr,"ERROR in rc_ringbuf_get_value, position out of bounds\n");
		return -1.0f;
	}
	if(unlikely(!buf->initialized)){
		fprintf(stderr,"ERROR in rc_ringbuf_get_value, ringbuf uninitialized\n");
		return -1.0f;
	}
	// check for looparound
	return_index=buf->index-pos;
	if(return_index<0) return_index+=buf->size;
	return buf->d[return_index];
}


double rc_ringbuf_std_dev(rc_ringbuf_t buf)
{
	int i;
	double mean, mean_sqr, diff;
	// sanity checks
	if(unlikely(!buf.initialized)){
		fprintf(stderr,"ERROR in rc_ringbuf_std_dev, ringbuf not initialized yet\n");
		return -1.0f;
	}
	// shortcut if buffer is of length 1
	if(buf.size == 1) return 0.0f;
	// calculate mean
	mean = 0.0f;
	for(i=0;i<buf.size;i++) mean+=buf.d[i];
	mean = mean/(double)buf.size;
	// calculate mean square
	mean_sqr = 0.0f;
	for(i=0;i<buf.size;i++){
		diff = buf.d[i]-mean;
		mean_sqr += diff*diff;
	}
	return sqrt(mean_sqr/(double)(buf.size-1));
}


