/*******************************************************************************
* rc_ring_buffer.c
* James Strawson 2016
*
* Ring buffers are FIFO (first in first out) buffers of fixed length which
* efficiently boot out the oldest value when full. They are particularly well
* suited for storing the last n values in a discrete time filter.
*
* The user creates their own instance of a buffer and passes a pointer to the
* these ringbuf functions to perform normal operations. 
*******************************************************************************/

#include "../roboticscape.h"
#include "../preprocessor_macros.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/*******************************************************************************
* int rc_alloc_ringbuf(rc_ringbuf_t* buf, int size)
*
* Allocates memory for a ring buffer and initializes an rc_ringbuf_t struct.
* If ring buffer b is already the right size then it is left untouched.
* Otherwise any existing memory allocated for buf is free'd to avoid memory
* leaks and new memory is allocated. Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_alloc_ringbuf(rc_ringbuf_t* buf, int size){
	// sanity checks
	if(unlikely(buf==NULL)){
		fprintf(stderr,"ERROR in rc_alloc_ringbuf, received NULL pointer\n");
		return -1;
	}
	if(unlikely(size<2)){
		fprintf(stderr,"ERROR in rc_alloc_ringbuf, size must be >=2\n");
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
	buf->d = (float*)calloc(size,sizeof(float));
	if(buf->d==NULL){
		fprintf(stderr,"ERROR in rc_alloc_ringbuf, failed to allocate memory\n");
		return -1;
	}
	// write out other details
	buf->size = size;
	buf->initialized = 1;
	return 0;
}

/*******************************************************************************
* rc_ringbuf_t rc_empty_ringbuf()
*
* Returns an rc_ringbuf_t struct which is completely zero'd out with no memory
* allocated for it. This is useful for declaring new ring buffers since structs
* declared inside of functions are not necessarily zero'd out which can cause
* the struct to contain problematic contents leading to segfaults. New ring
* buffers should be initialized with this before calling rc_alloc_ringbuf.
*******************************************************************************/
rc_ringbuf_t rc_empty_ringbuf(){
	rc_ringbuf_t out;
	// zero-out piecemeal instead of with memset to avoid issues with padding
	out.d=NULL;
	out.size=0;
	out.index=0;
	out.initialized=0;
	return out;
}

/*******************************************************************************
* int rc_free_ringbuf(rc_ringbuf_t* buf)
*
* Frees the memory allocated for buffer buf. Also set the initialized flag to 0
* so other functions don't try to access unallocated memory.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_free_ringbuf(rc_ringbuf_t* buf){
	if(unlikely(buf==NULL)){
		fprintf(stderr, "ERROR in rc_free_ringbuf, received NULL pointer\n");
		return -1;
	}
	if(buf->initialized)free(buf->d);
	*buf=rc_empty_ringbuf();
	return 0;
}

/*******************************************************************************
* int rc_reset_ringbuf(rc_ringbuf_t* buf)
*
* Sets all values in the buffer to 0 and sets the buffer index back to 0.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_reset_ringbuf(rc_ringbuf_t* buf){
	if(unlikely(buf==NULL)){
		fprintf(stderr, "ERROR in rc_reset_ringbuf, received NULL pointer\n");
		return -1;
	}
	if(unlikely(!buf->initialized)){
		fprintf(stderr,"ERROR rc_reset_ringbuf, ringbuf uninitialized\n");
		return -1;
	}
	memset(buf->d,0,buf->size*sizeof(float));
	buf->index=0;
	return 0;
}

/*******************************************************************************
* int rc_insert_new_ringbuf_value(rc_ringbuf_t* buf, float val)
* 
* Puts a new float into the ring buffer and updates the index accordingly.
* If the buffer was full then the oldest value in the buffer is automatically
* removed. Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_insert_new_ringbuf_value(rc_ringbuf_t* buf, float val){
	int new_index;
	// sanity checks
	if(unlikely(buf==NULL)){
		fprintf(stderr,"ERROR in rc_insert_new_ringbuf_value, received NULL pointer\n");
		return -1;
	}
	if(unlikely(!buf->initialized)){
		fprintf(stderr,"ERROR in rc_insert_new_ringbuf_value, ringbuf uninitialized\n");
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

/*******************************************************************************
* float rc_get_ringbuf_value(rc_ringbuf_t* buf, int pos)
*
* Returns the float which is 'pos' steps behind the last value added to the 
* buffer. If 'position' is given as 0 then the most recent value is returned. 
* Position 'pos' obviously can't be larger than the buffer size minus 1.
* Prints an error message and return -1.0f on error.
*******************************************************************************/
float rc_get_ringbuf_value(rc_ringbuf_t* buf, int pos){
	int return_index;
	// sanity checks
	if(unlikely(buf==NULL)){
		fprintf(stderr,"ERROR in rc_get_ringbuf_value, received NULL pointer\n");
		return -1.0f;
	}
	if(unlikely(pos<0 || pos>buf->size-1)){
		fprintf(stderr,"ERROR in rc_get_ringbuf_value, position out of bounds\n");
		return -1.0f;
	}
	if(unlikely(!buf->initialized)){
		fprintf(stderr,"ERROR in rc_get_ringbuf_value, ringbuf uninitialized\n");
		return -1.0f;
	}
	// check for looparound
	return_index=buf->index-pos;
	if(return_index<0) return_index+=buf->size;
	return buf->d[return_index];
}

/*******************************************************************************
* float rc_std_dev_ringbuf(rc_ringbuf_t buf)
*
* Returns the standard deviation of the values in the ring buffer.
*******************************************************************************/
float rc_std_dev_ringbuf(rc_ringbuf_t buf){
	int i;
	float mean, mean_sqr, diff;
	if(unlikely(!buf.initialized)){
		fprintf(stderr,"ERROR in rc_std_dev_ringbuf, ringbuf not initialized yet\n");
		return -1.0f;
	}
	// calculate mean
	mean = 0.0f;
	for(i=0;i<buf.size;i++) mean+=buf.d[i];
	mean = mean/(float)buf.size;
	// calculate mean square
	mean_sqr = 0.0f;
	for(i=0;i<buf.size;i++){
		diff = buf.d[i]-mean;
		mean_sqr += diff*diff;
	}
	return sqrt(mean_sqr/(float)buf.size);
}


