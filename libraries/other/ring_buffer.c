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

/*******************************************************************************
* int reset_ring_buf(ring_buf* buf)
*
* sets all values in the buffer to 0 and sets the buffer position back to 0
*******************************************************************************/
int reset_ring_buf(ring_buf* buf){
	int i;
	for(i=0; i<RING_BUF_SIZE; i++){
		buf->data[i] = 0;
	}
	buf->index = 0;
	return 0;
}

/*******************************************************************************
* int insert_new_ring_buf_value(ring_buf* buf, float val)
* 
* Puts a new float into the ring buffer. If the buffer was full then the oldest
* value in the buffer is automatically removed.
*******************************************************************************/
int insert_new_ring_buf_value(ring_buf* buf, float val){
	int new_index = buf->index + 1;
	if(new_index>=RING_BUF_SIZE){
		new_index = 0;
	}
	buf->data[new_index] = val;
	buf->index = new_index;
	return 0;
}

/*******************************************************************************
* float get_ring_buf_value(ring_buf* buf, int position)
*
* returns the float which is 'position' steps behind the last value placed in
* the buffer. If 'position' is given as 0 then the most recent value is
* returned. 'Position' obviously can't be larger than buffer_size minus 1
*******************************************************************************/
float get_ring_buf_value(ring_buf* buf, int position){
	// sanity range check
	if((position<0) || (position>RING_BUF_SIZE-1)){
		printf("ERROR: pos must be between 0 & %d\n", RING_BUF_SIZE-1);
		return -1;
	}
	int return_index = buf->index - position;
	if(return_index<0){
		return_index += RING_BUF_SIZE;
	}
	return buf->data[return_index];
}
