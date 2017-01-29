/*******************************************************************************
* rc__filter.c
* James Strawson 2016
*
* This is a collection of functions for generating and implementing discrete 
* SISO filters for arbitrary transfer functions. 
*******************************************************************************/

#include "../roboticscape.h"
#include "../preprocessor_macros.h"
#include <stdio.h>
#include <math.h>
#include <string.h> // for memset
#include <stdlib.h>

/*******************************************************************************
* int rc_alloc_filter(rc_filter_t* f, rc_vector_t num, rc_vector_t den, float dt)
*
* Allocate memory for a discrete-time filter & populates it with the transfer
* function coefficients provided in vectors num and den. The memory in num and
* den is duplicated so those vectors can be reused or freed after allocating a 
* filter without fear of disturbing the function of the filter. Argument dt is
* the timestep in seconds at which the user expects to operate the filter.
* The length of demonimator den must be at least as large as numerator num to
* avoid describing an improper transfer function. If rc_filter_t pointer f
* points to an existing filter then the old filter's contents are freed safely
* to avoid memory leaks. We suggest initializing filter f with rc_empty_filter
* before calling this function if it is not a global variable to ensure it does
* not accidentally contain invlaid contents such as null pointers. The filter's
* order is derived from the length of the denominator polynomial.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_alloc_filter(rc_filter_t* f, rc_vector_t num, rc_vector_t den, float dt){
	// sanity checks
	if(unlikely(dt<=0.0)){
		fprintf(stderr,"ERROR in rc_alloc_filter, dt must be >0\n");
		return -1;
	}
	if(unlikely(!num.initialized||!den.initialized)){
		fprintf(stderr,"ERROR in rc_alloc_filter, vector uninitialized\n");
		return -1;
	}
	if(unlikely(num.len>den.len)){
		fprintf(stderr,"ERROR in rc_alloc_filter, improper transfer function\n");
		return -1;
	}
	if(unlikely(den.d[0]==0.0f)){
		fprintf(stderr,"ERROR in rc_alloc_filter, first coefficient in denominator is 0\n");
		return -1;
	}
	// free existing memory, this also zeros out all fields
	rc_free_filter(f);
	// move in vectors
	if(unlikely(rc_duplicate_vector(num,&f->num))){
		fprintf(stderr,"ERROR in rc_alloc_filter, failed to duplicate numerator\n");
		return -1;
	}
	if(unlikely(rc_duplicate_vector(den,&f->den))){
		fprintf(stderr,"ERROR in rc_alloc_filter, failed to duplicate denominator\n");
		rc_free_vector(&f->num);
		return -1;
	}
	// allocate buffers
	if(unlikely(rc_alloc_ringbuf(&f->in_buf,den.len))){
		fprintf(stderr,"ERROR in rc_alloc_filter, failed to allocate ring buffer\n");
		rc_free_vector(&f->num);
		rc_free_vector(&f->den);
		return -1;
	}
	if(unlikely(rc_alloc_ringbuf(&f->out_buf,den.len))){
		fprintf(stderr,"ERROR in rc_alloc_filter, failed to allocate ring buffer\n");
		rc_free_vector(&f->num);
		rc_free_vector(&f->den);
		rc_free_ringbuf(&f->in_buf);
		return -1;
	}
	// populate remaining values, everything else zero'd by rc_free_filter
	f->dt=dt;
	f->order=den.len-1;
	f->initialized=1;
	return 0;
}

/*******************************************************************************
* int rc_alloc_filter_from_arrays(rc_filter_t* f,int order,float dt,float* num,float* den)
*
* Like rc_alloc_filter(), but takes arrays for the numerator and denominator
* coefficients instead of vectors. Arrays num and denmust be the same length
* (order+1) like a semi-proper transfer function. Proper transfer functions with
* relative degree >=1 can still be used but the numerator must be filled with
* leading zeros. This function will throw a segmentation fault if your arrays
* are not both of length order+1. It is safer to use the rc_alloc_filter.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_alloc_filter_from_arrays(rc_filter_t* f,int order,float dt,float* num,float* den){
	// sanity checks
	if(unlikely(order<1)){
		fprintf(stderr,"ERROR in rc_alloc_filter_from_arrays, order must be >=1\n");
		return -1;
	}
	if(unlikely(dt<0.0f)){
		fprintf(stderr,"ERROR in rc_alloc_filter_from_arrays, dt must be >0\n");
		return -1;
	}
	// free existing memory, this also zeros out all fields
	rc_free_filter(f);
	// copy numerator and denominators over
	if(unlikely(rc_vector_from_array(&f->num,num,order+1))){
		fprintf(stderr,"ERROR in rc_alloc_filter_from_arrays, failed to alloc vector\n");
		return -1;
	}
	if(unlikely(rc_vector_from_array(&f->den,den,order+1))){
		fprintf(stderr,"ERROR in rc_alloc_filter_from_arrays, failed to alloc vector\n");
		rc_free_vector(&f->num);
		return -1;
	}
	// allocate buffers
	if(unlikely(rc_alloc_ringbuf(&f->in_buf,order+1))){
		fprintf(stderr,"ERROR in rc_alloc_filter, failed to allocate ring buffer\n");
		rc_free_vector(&f->num);
		rc_free_vector(&f->den);
		return -1;
	}
	if(unlikely(rc_alloc_ringbuf(&f->out_buf,order+1))){
		fprintf(stderr,"ERROR in rc_alloc_filter, failed to duplicate denominator\n");
		rc_free_vector(&f->num);
		rc_free_vector(&f->den);
		rc_free_ringbuf(&f->in_buf);
		return -1;
	}
	// populate remaining values, everything else zero'd by rc_free_filter
	f->dt=dt;
	f->order=order;
	f->initialized=1;
	return 0;
}

/*******************************************************************************
* int rc_free_filter(rc_filter_t* f)
*
* Frees the memory allocated by a filter's buffers and coefficient vectors. Also
* resets all filter properties back to 0. Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_free_filter(rc_filter_t* f){
	if(unlikely(f==NULL)){
		fprintf(stderr, "ERROR in rc_free_filter, received NULL pointer\n");
		return -1;
	}
	rc_free_ringbuf(&f->in_buf);
	rc_free_ringbuf(&f->out_buf);
	rc_free_vector(&f->num);
	rc_free_vector(&f->den);
	*f = rc_empty_filter();
	return 0;
}

/*******************************************************************************
* rc_filter_t rc_empty_filter()
*
* This is a very important function. If your d_filter_t struct is not a global
* variable, then its initial contents cannot be guaranteed to be anything in
* particular. Therefore it could contain problematic contents which could
* interfere with functions in this library. Therefore, you should always
* initialize your filters with rc_empty_filter before using with any other
* function in this library such as rc_alloc_filter. This serves the same
* purpose as rc_empty_matrix, rc_empty_vector, and rc_empty_ringbuf.
*******************************************************************************/
rc_filter_t rc_empty_filter(){
	rc_filter_t f;
	f.order			= 0;
	f.dt			= 0.0f;
	f.gain			= 1.0f;
	f.num			= rc_empty_vector();
	f.den			= rc_empty_vector();
	f.sat_en		= 0;
	f.sat_min		= 0.0f;
	f.sat_max		= 0.0f;
	f.sat_flag		= 0;
	f.ss_en			= 0;
	f.ss_steps		= 0;
	f.in_buf		= rc_empty_ringbuf();
	f.out_buf		= rc_empty_ringbuf();
	f.newest_input	= 0.0f;
	f.newest_output = 0.0f;
	f.step			= 0;
	f.initialized	= 0;
	return f;
}

/*******************************************************************************
* float rc_march_filter(rc_filter_t* f, float new_input)
*
* March a filter forward one step with new input provided as an argument.
* Returns the new output which could also be accessed with filter.newest_output
* If saturation or soft-start are enabled then the output will automatically be
* bound appropriately. The steps counter is incremented by one and internal
* ring buffers are updated accordingly. Once a filter is created, this is
* typically the only function required afterwards.
*******************************************************************************/
float rc_march_filter(rc_filter_t* f, float new_input){
	int i, rel_deg;
	float new_out = 0.0f;
	// sanity checks
	if(unlikely(!f->initialized)){
		printf("ERROR in rc_march_filter, filter uninitialized\n");
		return -1.0f;
	}
	// log new input
	rc_insert_new_ringbuf_value(&f->in_buf, new_input);
	f->newest_input = new_input;
	// relative degree should never be negative as rc_alloc_filter checks
	// for improper transfer functions
	rel_deg = f->den.len - f->num.len;
	// evaluate the difference equation
	for(i=0; i<(f->num.len); i++){
		new_out+=f->gain*f->num.d[i]*rc_get_ringbuf_value(&f->in_buf, i+rel_deg);
	}
	for(i=0; i<(f->order); i++){
		new_out-=f->den.d[i+1]*rc_get_ringbuf_value(&f->out_buf, i);
	}
	// scale in case denominator doesn't have a leading 1
	new_out /= f->den.d[0];
	// soft start limits
	if(f->ss_en && f->step<f->ss_steps){
		float a=f->sat_max*(f->step/f->ss_steps);
		float b=f->sat_min*(f->step/f->ss_steps);
		if(new_out>a) new_out=a;
		if(new_out<b) new_out=b;
	}
	// saturate and set flag
	if(f->sat_en){
		if(new_out>f->sat_max){
			new_out=f->sat_max;
			f->sat_flag=1;
		}
		else if(new_out<f->sat_min){
			new_out=f->sat_min;
			f->sat_flag=1;
		}
		else f->sat_flag=0;
	}
	// record the output to filter struct and ring buffer
	f->newest_output = new_out;
	rc_insert_new_ringbuf_value(&f->out_buf, new_out);
	// increment steps
	f->step++;
	return new_out;
}

/*******************************************************************************
* int rc_reset_filter(rc_filter_t* filter)
*
* Resets all previous inputs and outputs to 0 and resets the step counter
* and saturation flag. This is sufficient to start the filter again as if it
* were just created. Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_reset_filter(rc_filter_t* f){
	if(unlikely(!f->initialized)){
		fprintf(stderr,"ERROR in rc_reset_filter, filter uninitialized\n");
		return -1;
	}
	rc_reset_ringbuf(&f->in_buf);
	rc_reset_ringbuf(&f->out_buf);
	f->newest_input	= 0.0f;
	f->newest_output = 0.0f;
	f->sat_flag = 0;
	f->step = 0;
	return 0;
}

/*******************************************************************************
* int print_poly_z(rc_vector_t v){
*
* only for use in this lib. Like rc_print_poly but uses 
*******************************************************************************/
int print_poly_z(rc_vector_t v){
	int i;
	static char *super[] = {"\xe2\x81\xb0", "\xc2\xb9", "\xc2\xb2",
    "\xc2\xb3", "\xe2\x81\xb4", "\xe2\x81\xb5", "\xe2\x81\xb6",
    "\xe2\x81\xb7", "\xe2\x81\xb8", "\xe2\x81\xb9"};
    if(unlikely(v.len>10)){
		fprintf(stderr,"ERROR in rc_print_filter, filter order must be <=10\n");
		return -1;
	}
    for(i=0;i<(v.len-2);i++) printf("%7.4fz%s + ",v.d[i],super[v.len-i-1]);
	if(v.len>=2) printf("%7.4fz  + ",v.d[v.len-2]);
	printf("%7.4f\n", v.d[v.len-1]);
	return 0;
}

/*******************************************************************************
* int rc_print_filter(rc_filter_t f)
*
* Prints the transfer function and other statistic of a filter to the screen.
* only works on filters up to order 9
*******************************************************************************/
int rc_print_filter(rc_filter_t f){
	int i;
	if(unlikely(!f.initialized)){
		fprintf(stderr,"ERROR in rc_print_filter, filter not initialized yet\n");
		return -1;
	}
	if(unlikely(f.order>9)){
		fprintf(stderr,"ERROR in rc_print_filter, filter order must be <=10\n");
		return -1;
	}
	
	printf("order: %d\n", f.order);
	printf("timestep dt: %0.4f\n", f.dt);
	
	// print numerator
	print_poly_z(f.num);
	printf("--------");
	for(i=0;i<f.order;i++) printf("------------");
	printf("\n");
	print_poly_z(f.den);
	
	return 0;
}

/*******************************************************************************
* int rc_enable_saturation(rc_filter_t* f, float min, float max)
*
* If saturation is enabled for a specified filter, the filter will automatically
* bound the output between min and max. You may ignore this function if you wish
* the filter to run unbounded.
*******************************************************************************/
int rc_enable_saturation(rc_filter_t* f, float min, float max){
	if(unlikely(!f->initialized)){
		fprintf(stderr,"ERROR in rc_enable_saturation, filter uninitialized\n");
		return -1;
	}
	if(unlikely(min>=max)){
		printf("ERORR in rc_enable_saturation, max must be > min\n");
		return -1;
	}
	f->sat_en	= 1;
	f->sat_min	= min;
	f->sat_max	= max;
	return 0;
}

/*******************************************************************************
* int rc_did_filter_saturate(rc_filter_t* filter)
*
* Returns 1 if the filter saturated the last time step. Returns 0 otherwise.
* This information could also be retrieved by looking at the 'sat_flag' value
* in the filter struct.
*******************************************************************************/
int rc_did_filter_saturate(rc_filter_t* f){
	if(unlikely(!f->initialized)){
		fprintf(stderr,"ERROR in rc_did_filter_saturate, filter uninitialized\n");
		return -1;
	}
	return f->sat_flag;
}

/*******************************************************************************
* int enable_soft_start(rc_filter_t* filter, float seconds)
*
* Enables soft start functionality where the output bound is gradually opened
* linearly from 0 to the normal saturation range. This occurs over the time 
* specified from argument 'seconds' from when the filter is first created or 
* reset. Saturation must already be enabled for this to work. This assumes that
* the user does indeed call rc_march_filter at roughly the same time interval
* as the 'dt' variable in the filter struct which is set at creation time.
* The soft-start property is maintained through a call to rc_reset_filter
* so the filter will soft-start again after each reset. This feature should only
* really be used for feedback controllers to prevent jerky starts.
* The saturation flag will not be set during this period as the output is
* usually expected to be bounded and we don't want to falsely trigger alarms
* or saturation counters. Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_enable_soft_start(rc_filter_t* f, float seconds){
	if(unlikely(!f->initialized)){
		fprintf(stderr,"ERROR in rc_enable_soft_start, filter uninitialized\n");
		return -1;
	}
	if(unlikely(seconds<=0.0f)){
		fprintf(stderr,"ERROR in rc_enable_soft_start, seconds must be >=0\n");
		return -1;
	}
	if(unlikely(!f->sat_en)){
		fprintf(stderr,"ERROR in rc_enable_soft_start, saturation must be enabled first\n");
		return -1;
	}
	f->ss_en	= 1;
	f->ss_steps	= seconds/f->dt;
	return 0;
}

/*******************************************************************************
* float rc_previous_filter_input(rc_filter_t* f, int steps)
*
* Returns the input 'steps' back in time. Steps = 0 returns most recent input.
* 'steps' must be between 0 and order inclusively as those are the
* only steps retained in memory for normal filter operation. To record values
* further back in time we suggest creating your own rc_ringbuf_t ring buffer.
* Returns -1.0f and prints an error message if there is an issue.
*******************************************************************************/
float rc_previous_filter_input(rc_filter_t* f, int steps){
	if(unlikely(!f->initialized)){
		fprintf(stderr,"ERROR in rc_previous_filter_input, filter uninitialized\n");
		return -1.0f;
	}
	return rc_get_ringbuf_value(&f->in_buf, steps);
}

/*******************************************************************************
* float rc_previous_filter_output(rc_filter_t* f, int steps)
*
* Returns the input 'steps' back in time. Steps = 0 returns most recent input.
* 'steps' must be between 0 and order inclusively as those are the
* only steps retained in memory for normal filter operation. To record values
* further back in time we suggest creating your own rc_ringbuf_t ring buffer.
* Returns -1.0f and prints an error message if there is an issue.
*******************************************************************************/
float rc_previous_filter_output(rc_filter_t* f, int steps){
	if(unlikely(!f->initialized)){
		fprintf(stderr,"ERROR in rc_previous_filter_output, filter uninitialized\n");
		return -1.0f;
	}
	return rc_get_ringbuf_value(&f->out_buf, steps);
}

/*******************************************************************************
* float rc_newest_filter_output(rc_filter_t* f)
*
* Returns the most recent output from the filter. Alternatively the user could
* access the 'newest_output' component of the rc_filter_t struct. Returns -1.0f
* and prints an error message if there is an issue.
*******************************************************************************/
float rc_newest_filter_output(rc_filter_t* f){
	if(unlikely(!f->initialized)){
		fprintf(stderr,"ERROR in rc_newest_filter_output, filter uninitialized\n");
		return -1.0f;
	}
	return f->newest_output;
}

/*******************************************************************************
* float rc_newest_filter_input(rc_filter_t* f)
*
* Returns the most recent input to the filter. Alternatively the user could
* access the 'newest_input' component of the rc_filter_t struct. Returns -1.0f
* and prints an error message if there is an issue.
*******************************************************************************/
float rc_newest_filter_input(rc_filter_t* f){
	if(unlikely(!f->initialized)){
		fprintf(stderr,"ERROR in rc_newest_filter_input, filter uninitialized\n");
		return -1.0f;
	}
	return f->newest_input;
}

/*******************************************************************************
* int rc_prefill_filter_inputs(rc_filter_t* f, float in)
*
* Fills all previous inputs to the filter as if they had been equal to 'in'
* Most useful when starting high-pass filters to prevent unwanted jumps in the
* output when starting with non-zero input.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_prefill_filter_inputs(rc_filter_t* f, float in){
	int i;
	if(unlikely(!f->initialized)){
		fprintf(stderr,"ERROR in rc_prefill_filter_inputs, filter uninitialized\n");
		return -1;
	}
	for(i=0;i<f->order;i++) rc_insert_new_ringbuf_value(&f->in_buf, in);
	f->newest_input = in;
	return 0;
}

/*******************************************************************************
* int rc_prefill_filter_outputs(rc_filter_t* f, float out)
*
* Fills all previous outputs of the filter as if they had been equal to 'out'
* Most useful when starting low-pass filters to prevent unwanted settling time
* when starting with non-zero input. Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_prefill_filter_outputs(rc_filter_t* f, float out){
	int i;
	if(unlikely(!f->initialized)){
		fprintf(stderr,"ERROR in rc_prefill_filter_outputs, filter uninitialized\n");
		return -1;
	}
	for(i=0;i<f->order;i++) rc_insert_new_ringbuf_value(&(f->out_buf), out);
	f->newest_output = out;
	return 0;
}

/*******************************************************************************
* int rc_multiply_filters(rc_filter_t f1, rc_filter_t f2, rc_filter_t* f3)
*
* Creates a new filter f3 by multiplying f1*f2. The contents of f3 are freed
* safely if necessary and new memory is allocated to avoid memory leaks.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_multiply_filters(rc_filter_t f1, rc_filter_t f2, rc_filter_t* f3){
	rc_vector_t newnum = rc_empty_vector();
	rc_vector_t newden = rc_empty_vector();
	// sanity checks
	if(unlikely(!f1.initialized||!f2.initialized)){
		fprintf(stderr,"ERROR in rc_multiply_filters, filter uninitialized\n");
		return -1;
	}
	if(unlikely(f1.dt!=f2.dt)){
		fprintf(stderr,"ERROR in rc_multiply_filters, timestep dt must match\n");
		return -1;
	}
	// multiply out the transfer function coefficients
	if(unlikely(rc_poly_conv(f1.num,f2.num,&newnum))){
		fprintf(stderr,"ERROR in rc_multiply_filters, failed to polyconv\n");
		return -1;
	}
	if(unlikely(rc_poly_conv(f1.den,f2.den,&newden))){
		fprintf(stderr,"ERROR in rc_multiply_filters, failed to polyconv\n");
		rc_free_vector(&newnum);
		return -1;
	}
	// create the filter
	if(unlikely(rc_alloc_filter(f3,newnum,newden,f1.dt))){
		fprintf(stderr,"ERROR in rc_multiply_filters, failed to alloc filter\n");
		return -1;
	}
	f3->gain = f1.gain * f2.gain;
	rc_free_vector(&newnum);
	rc_free_vector(&newden);
	return 0;
}

/*******************************************************************************
* int rc_c2d_tustin(rc_filter_t* f,rc_vector_t num,rc_vector_t den,float dt,float w)
* 
* Creates a discrete time filter with similar dynamics to a provided continuous
* time transfer function using tustin's approximation with prewarping about a
* frequency of interest 'w' in radians per second.
*
* arguments:
* rc_vector_t num: 	continuous time numerator coefficients
* rc_vector_t den: 	continuous time denominator coefficients
* float dt:			desired timestep of discrete filter
* float w:			prewarping frequency in rad/s
*
* Any existing memory allocated for f is freed is necessary to prevent memory
* leaks. Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_c2d_tustin(rc_filter_t* f,rc_vector_t num,rc_vector_t den,float dt,float w){
	int i,j,m,n;
	double a,c, A0;
	rc_vector_t numZ	= rc_empty_vector();
	rc_vector_t denZ	= rc_empty_vector();
	rc_vector_t p1		= rc_empty_vector();
	rc_vector_t p2		= rc_empty_vector();
	rc_vector_t temp	= rc_empty_vector();
	rc_vector_t v1		= rc_empty_vector();
	rc_vector_t v2		= rc_empty_vector();
	// sanity checks
	if(unlikely(!num.initialized||!den.initialized)){
		fprintf(stderr,"ERROR in rc_c2d_tustin, vector uninitialized\n");
		return -1;
	}
	if(unlikely(dt<=0.0f)){
		fprintf(stderr,"ERROR in rc_c2d_tustin, dt must be positive\n");
		return -1;
	}
	if(unlikely(w>(M_PI/dt))){
		fprintf(stderr,"ERROR in rc_c2d_tustin, w larger than nyquist frequency\n");
		return -1;
	}
	a = 2.0*(1.0 - cos(w*dt)) / (w*dt*sin(w*dt));
	c = 2.0/(a*dt);
	m = num.len - 1;			// order of num
	n = den.len - 1;			// order of den
	rc_vector_zeros(&numZ,n+1);
	rc_vector_zeros(&denZ,n+1);
	rc_alloc_vector(&p1,2);	// (z - 1)
	p1.d[0]		= 1.0f;
	p1.d[1]		= -1.0f;
	rc_alloc_vector(&p2,2);	// (z + 1)
	p2.d[0]		= 1.0f;
	p2.d[1]		= 1.0f;
	
	// from zeroth up to and including mth
	for(i=0;i<=m;i++){
		rc_poly_power(p1,m-i,&v1);
		rc_poly_power(p2,n-m+i,&v2);
		rc_poly_conv(v1,v2,&temp);
		for(j=0;j<n+1;j++) numZ.d[j] += num.d[i]*pow(c,m-i)*temp.d[j];
	}
	for(i=0;i<=n;i++){
		rc_poly_power(p1,n-i,&v1);
		rc_poly_power(p2,i,&v2);
		rc_poly_conv(v1,v2,&temp);
		for(j=0;j<n+1;j++) denZ.d[j] += den.d[i]*pow(c,n-i)*temp.d[j];
	}
	A0 = denZ.d[0];
	
	// normalize
	for(i=0;i<n+1;i++){
		numZ.d[i] = numZ.d[i]/A0;
		denZ.d[i] = denZ.d[i]/A0;
	}
	// free up memory
	rc_free_vector(&p1);
	rc_free_vector(&p2);
	rc_free_vector(&temp);
	rc_free_vector(&v1);
	rc_free_vector(&v2);
	// make the filter
	if(unlikely(rc_alloc_filter(f, numZ, denZ, dt))){
		fprintf(stderr, "ERROR in rc_c2d_tustin, failed to alloc filter\n");
		rc_free_vector(&numZ);
		rc_free_vector(&denZ);
		return -1;
	}
	rc_free_vector(&numZ);
	rc_free_vector(&denZ);
	return 0;
}

/*******************************************************************************
* int rc_first_order_lowpass(rc_filter_t* f, float dt, float time_constant)
*
* Creates a first order low pass filter. Any existing memory allocated for f is 
* freed safely to avoid memory leaks and new memory is allocated for the new 
* filter. dt is in units of seconds and time_constant is the number of seconds 
* it takes to rise to 63.4% of a steady-state input. This can be used alongside
* rc_first_order_highpass to make a complementary filter pair.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_first_order_lowpass(rc_filter_t* f, float dt, float time_constant){
	float lp_const = dt/time_constant;
	float num[2], den[2];
	// sanity checks
	if(unlikely(time_constant<=0.0f)){
		fprintf(stderr, "ERROR in rc_first_order_lowpass, time constant must be >0\n");
		return -1;
	}
	num[0] = lp_const;
	num[1] = 0.0f;
	den[0] = 1.0f;
	den[1] = lp_const -1.0f;
	// make the filter
	if(unlikely(rc_alloc_filter_from_arrays(f,1,dt,num,den))){
		fprintf(stderr, "ERROR in rc_first_order_lowpass, failed to alloc filter\n");
		return -1;
	}
	return 0;
}

/*******************************************************************************
* int rc_first_order_highpass(rc_filter_t* f, float dt, float time_constant)
*
* Creates a first order high pass filter. Any existing memory allocated for f is 
* freed safely to avoid memory leaks and new memory is allocated for the new 
* filter. dt is in units of seconds and time_constant is the number of seconds 
* it takes to decay by 63.4% of a steady-state input. This can be used alongside
* rc_first_order_lowpass to make a complementary filter pair.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_first_order_highpass(rc_filter_t* f, float dt, float time_constant){
	float hp_const = dt/time_constant;
	float num[2], den[2];
	// sanity checks
	if(unlikely(time_constant<=0.0f)){
		fprintf(stderr, "ERROR in rc_first_order_highpass, time constant must be >0\n");
		return -1;
	}
	num[0] = 1.0f-hp_const;
	num[1] = hp_const-1.0f;
	den[0] = 1.0f;
	den[1] = hp_const-1.0f;
	// make the filter
	if(unlikely(rc_alloc_filter_from_arrays(f,1,dt,num,den))){
		fprintf(stderr, "ERROR in rc_first_order_highpass, failed to alloc filter\n");
		return -1;
	}
	return 0;
}

/*******************************************************************************
* int rc_butterworth_lowpass(rc_filter_t* f, int order, float dt, float wc)
*
* Creates a Butterworth low pass filter of specified order and cutoff frequency
* wc in rad/s. The user must also specify the discrete filter's timestep dt
* in seconds. Any existing memory allocated for f is freed safely to avoid
* memory leaks and new memory is allocated for the new filter.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_butterworth_lowpass(rc_filter_t* f, int order, float dt, float wc){
	rc_vector_t num = rc_empty_vector();
	rc_vector_t den = rc_empty_vector();
	if(unlikely(order<1)){
		fprintf(stderr, "ERROR in rc_butterworth_lowpass, order must be >=1\n");
		return -1;
	}
	if(unlikely(rc_poly_butter(order,wc,&den))){
		fprintf(stderr, "ERROR in rc_butterworth_lowpass, failed to find butterworth polynomial\n");
		return -1;
	}
	rc_alloc_vector(&num,1);
	num.d[0] = 1.0f;
	if(unlikely(rc_c2d_tustin(f,num,den,dt,wc))){
		fprintf(stderr, "ERROR in rc_butterworth_lowpass, failed to c2d_tustin\n");
		rc_free_vector(&num);
		rc_free_vector(&den);
		return -1;
	}
	rc_free_vector(&num);
	rc_free_vector(&den);
	return 0;
}

/*******************************************************************************
* int rc_butterworth_highpass(rc_filter_t* f, int order, float dt, float wc)
*
* Creates a Butterworth high pass filter of specified order and cutoff frequency
* wc in rad/s. The user must also specify the discrete filter's timestep dt
* in seconds. Any existing memory allocated for f is freed safely to avoid
* memory leaks and new memory is allocated for the new filter.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_butterworth_highpass(rc_filter_t* f, int order, float dt, float wc){
	rc_vector_t num = rc_empty_vector();
	rc_vector_t den = rc_empty_vector();
	if(unlikely(order<1)){
		fprintf(stderr, "ERROR in rc_butterworth_highpass, order must be >=1\n");
		return -1;
	}
	if(unlikely(rc_poly_butter(order,wc,&den))){
		fprintf(stderr, "ERROR in rc_butterworth_highpass, failed to find butterwoth polynomial\n");
		return -1;
	}
	// numerator consists of all zeros at the origin
	rc_vector_zeros(&num,order+1);
	num.d[0] = 1.0f;
	
	if(unlikely(rc_c2d_tustin(f,num,den,dt,wc))){
		fprintf(stderr, "ERROR in rc_butterworth_highpass, failed to c2d_tustin\n");
		rc_free_vector(&num);
		rc_free_vector(&den);
		return -1;
	}
	rc_free_vector(&num);
	rc_free_vector(&den);
	return 0;
}

/*******************************************************************************
* int rc_moving_average(rc_filter_t* f, int samples, int dt)
*
* Makes a FIR moving average filter that averages over 'samples' which must be
* greater than or equal to 2 otherwise no averaging would be performed. Any
* existing memory allocated for f is freed safely to avoid memory leaks and new
* memory is allocated for the new filter. Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_moving_average(rc_filter_t* f, int samples, int dt){
	int i;
	rc_vector_t num = rc_empty_vector();
	rc_vector_t den = rc_empty_vector();
	// sanity checks
	if(unlikely(samples<2)){
		fprintf(stderr,"ERROR in rc_moving_average, samples must be >=2\n");
		return -1;
	}
	rc_alloc_vector(&num,samples);
	rc_alloc_vector(&den,samples);
	// fill in coefficients
	for(i=0;i<samples;i++){
		num.d[i] = 1.0f/samples;
		den.d[i] = 0.0;
	}
	den.d[0] = 1.0;
	// make the filter
	if(unlikely(rc_alloc_filter(f,num,den,dt))){
		fprintf(stderr, "ERROR in rc_moving_average, failed to alloc filter\n");
		rc_free_vector(&num);
		rc_free_vector(&den);
		return -1;
	}
	rc_free_vector(&num);
	rc_free_vector(&den);
	return 0;
}

/*******************************************************************************
* int rc_integrator(rc_filter_t *f, float dt)
*
* Creates a first order integrator. Like most functions here, the dynamics are
* only accurate if the filter is called with a timestep corresponding to dt.
* Any existing memory allocated for f is freed safely to avoid memory leaks and
* new memory is allocated for the new filter.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_integrator(rc_filter_t *f, float dt){
	rc_vector_t num = rc_empty_vector();
	rc_vector_t den = rc_empty_vector();
	// sanity checks
	if(unlikely(dt<=0.0f)){
		fprintf(stderr, "ERROR in rc_integrator, dt must be >0\n");
		return -1;
	}
	rc_alloc_vector(&num,1);
	rc_alloc_vector(&den,2);
	num.d[0] = dt;
	den.d[0] = 1.0;
	den.d[1] = -1.0;
	// make the filter
	if(unlikely(rc_alloc_filter(f,num,den,dt))){
		fprintf(stderr, "ERROR in rc_integrator, failed to alloc filter\n");
		rc_free_vector(&num);
		rc_free_vector(&den);
		return -1;
	}
	rc_free_vector(&num);
	rc_free_vector(&den);
	return 0;
}

/*******************************************************************************
* int rc_double_integrator(rc_filter_t* f, float dt)
*
* Creates a second order double integrator. Like most functions here, the 
* dynamics are only accurate if the filter is called with a timestep
* corresponding to dt. Any existing memory allocated for f is freed safely to
* avoid memory leaks and new memory is allocated for the new filter.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_double_integrator(rc_filter_t* f, float dt){
	rc_vector_t num = rc_empty_vector();
	rc_vector_t den = rc_empty_vector();
	// sanity checks
	if(unlikely(dt<=0.0f)){
		fprintf(stderr, "ERROR in rc_double_integrator, dt must be >0\n");
		return -1;
	}
	rc_alloc_vector(&num,1);
	rc_alloc_vector(&den,3);
	num.d[0] = dt*dt;
	den.d[0] = 1.0;
	den.d[1] = -2.0;
	den.d[2] = 1.0;
	// make the filter
	if(unlikely(rc_alloc_filter(f,num,den,dt))){
		fprintf(stderr, "ERROR in rc_double_integrator, failed to alloc filter\n");
		rc_free_vector(&num);
		rc_free_vector(&den);
		return -1;
	}
	rc_free_vector(&num);
	rc_free_vector(&den);
	return 0;
}

/*******************************************************************************
* int rc_pid_filter(rc_filter_t* f,float kp,float ki,float kd,float Tf,float dt)
*
* Creates a discrete-time implementation of a parallel PID controller with 
* high-frequency rolloff. This is equivalent to the Matlab function: 
* C = pid(Kp,Ki,Kd,Tf,Ts)
*
* We cannot implement a pure differentiator with a discrete transfer function
* so this filter has high frequency rolloff with time constant Tf. Smaller Tf
* results in less rolloff, but Tf must be greater than dt/2 for stability.
* Returns 0 on success or -1 on failure.
*******************************************************************************/
int rc_pid_filter(rc_filter_t* f,float kp,float ki,float kd,float Tf,float dt){
	rc_vector_t num = rc_empty_vector();
	rc_vector_t den = rc_empty_vector();
	// sanity checks
	if(unlikely(dt<0.0f)){
		fprintf(stderr,"ERROR in rc_pid_filter, dt must be >0\n");
		return -1;
	}
	if(unlikely(Tf <= dt/2)){
		printf("ERROR in rc_pid_filter, Tf must be > dt/2 for stability\n");
		return -1;
	}
	// if ki==0, return a 1st order PD filter with rolloff
	if(ki==0.0f){
		rc_alloc_vector(&num,2);
		rc_alloc_vector(&den,2);
		num.d[0] = (kp*Tf+kd)/Tf;
		num.d[1] = -(((ki*dt-kp)*(dt-Tf))+kd)/Tf;
		den.d[0] = 1.0f;
		den.d[1] = -(Tf-dt)/Tf;
	}
	//otherwise 2nd order PID with roll off
	else{
		rc_alloc_vector(&num,3);
		rc_alloc_vector(&den,3);
		num.d[0] = (kp*Tf+kd)/Tf;
		num.d[1] = (ki*dt*Tf + kp*(dt-Tf) - kp*Tf - 2.0*kd)/Tf;
		num.d[2] = (((ki*dt-kp)*(dt-Tf))+kd)/Tf;
		den.d[0] = 1.0f;
		den.d[1] = (dt-(2.0*Tf))/Tf;
		den.d[2] = (Tf-dt)/Tf;
	}
	// make the filter
	if(unlikely(rc_alloc_filter(f,num,den,dt))){
		fprintf(stderr, "ERROR in rc_pid_filter, failed to alloc filter\n");
		rc_free_vector(&num);
		rc_free_vector(&den);
		return -1;
	}
	rc_free_vector(&num);
	rc_free_vector(&den);
	return 0;
}
