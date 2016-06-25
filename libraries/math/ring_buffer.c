/*******************************************************************************
* ring_buffer.c
* James Strawson 2016
*
* Ring buffers are FIFO (first in first out) buffers of fixed length which
* efficiently boot out the oldest value when full. They are particularly well
* suited for storing the last n values in a discrete time filter.
*
* The user creates their own instance of a buffer and passes a pointer to the
* these ring_buf functions to perform normal operations. 
*******************************************************************************/

#include "../robotics_cape.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*******************************************************************************
* ring_buf_t create_ring_buf(int size)
*
* Allocated memory for a ring buffer and initializes the ring_buf_t struct
*******************************************************************************/
ring_buf_t create_ring_buf(int size){
	ring_buf_t buf;
	
	if(size<2){
		printf("ERROR: ring_buf_size must be greater than or equal to 2\n");
		return buf;
	}
	buf.data = (float*)calloc(size,sizeof(float));
	if(buf.data == NULL){
		printf("ERROR: failed to allocate memory for ring buffer\n");
		return buf;
	}
	buf.size = size;
	buf.index = 0;
	buf.initialized = 1;
	return buf;
}
 
/*******************************************************************************
* int destroy_ring_buf(ring_buf_t buf)
*
* frees the memory allocated by the buffer. Also set the Initialize flag to 0
* so other functions don't try to access unallocated memory.
*******************************************************************************/
int destroy_ring_buf(ring_buf_t* buf){
	free(buf->data);
	buf->data = NULL;
	buf->initialized=0;
	return 0;
}
		
/*******************************************************************************
* int reset_ring_buf(ring_buf_t* buf)
*
* sets all values in the buffer to 0 and sets the buffer position back to 0
*******************************************************************************/
int reset_ring_buf(ring_buf_t* buf){
	if(buf->initialized !=1){
		printf("ERROR: trying to reset an uninitialized ring buffer\n");
		return -1;
	}
	memset(buf->data, 0, buf->size*sizeof(float));
	buf->index = 0;
	return 0;
}

/*******************************************************************************
* int insert_new_ring_buf_value(ring_buf_t* buf, float val)
* 
* Puts a new float into the ring buffer. If the buffer was full then the oldest
* value in the buffer is automatically removed.
*******************************************************************************/
int insert_new_ring_buf_value(ring_buf_t* buf, float val){
	if(buf->initialized !=1){
		printf("ERROR: trying add value to uninitialized ring buffer\n");
		return -1;
	}
	int new_index = buf->index + 1;
	if(new_index >= buf->size){
		new_index = 0;
	}
	buf->data[new_index] = val;
	buf->index = new_index;
	return 0;
}

/*******************************************************************************
* float get_ring_buf_value(ring_buf_t* buf, int position)
*
* returns the float which is 'position' steps behind the last value placed in
* the buffer. If 'position' is given as 0 then the most recent value is
* returned. 'Position' obviously can't be larger than buffer_size minus 1
*******************************************************************************/
float get_ring_buf_value(ring_buf_t* buf, int position){
	// sanity range check
	if((position<0) || (position>buf->size-1)){
		printf("ERROR: pos must be between 0 & %d\n", buf->size-1);
		return -1;
	}
	if(buf->initialized !=1){
		printf("ERROR: trying to read value from uninitialized ring buffer\n");
		return -1;
	}
	int return_index = buf->index - position;
	if(return_index<0){
		return_index += buf->size;
	}
	return buf->data[return_index];
}
