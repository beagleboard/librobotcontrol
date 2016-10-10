/*******************************************************************************
* discrete_siso_filter.c
* James Strawson 2016
*
* This is a collection of functions for generating and implementing discrete 
* SISO filters for arbitrary transfer functions. 
*******************************************************************************/

#include "../roboticscape.h"
#include <stdio.h>
#include <math.h>
#include <string.h> // for memset


/*******************************************************************************
* d_filter_t create_filter(int order, float dt, float* num, float* den)
*
* Allocate memory for a filter of specified order & fill with transfer
* function constants. Use enable_saturation immediately after this if you want
* to enable automatic saturation.
*******************************************************************************/
d_filter_t create_filter(int order, float dt, float* num, float* den){
	d_filter_t filter;
	
	if(order<1){
		printf("ERROR: order must be >=1\n");
		return filter;
	}
	filter.order = order;
	filter.gain = 1;
	filter.newest_input = 0;
	filter.newest_output = 0;
	filter.saturation_en = 0;
	filter.saturation_min = 0;
	filter.saturation_max = 0;
	filter.saturation_flag = 0;
	filter.soft_start_en = 0;
	filter.soft_start_steps = 0;
	filter.numerator   = create_vector_from_array(order+1, num);
	filter.denominator = create_vector_from_array(order+1, den);
	filter.in_buf 	   = create_ring_buf(order+1);
	filter.out_buf     = create_ring_buf(order+1);
	filter.initialized = 1;
	filter.step = 0;
	return filter;
}

/*******************************************************************************
* int destroy_filter(d_filter_t* filter)
*
* free the memory allocated by the filter's buffers and coefficient vectors.
*******************************************************************************/
int destroy_filter(d_filter_t* filter){
	if(filter->initialized == 0) return -1;
	destroy_ring_buf(&(filter->in_buf));
	destroy_ring_buf(&(filter->out_buf));
	destroy_vector(&(filter->numerator));
	destroy_vector(&(filter->denominator));
	filter->initialized = 0;
	return 0;
}

/*******************************************************************************
* d_filter_t create_empty_filter()
*
* Returns an empty d_filter_t struct initialized to 0
*******************************************************************************/
d_filter_t create_empty_filter(){
	d_filter_t out;
	memset(&out,0,sizeof(d_filter_t));
	return out;
}

/*******************************************************************************
* float march_filter(d_filter_t* filter, float new_input)
*
* March the filter forward in time one step with new input data.
* Returns new output which could also be accessed with filter.current_output
* If saturation is enabled then the output will automatically be bound by the
* min and max values given to enable_saturation. The enable_saturation entry
* in the filter struct will also be set to 1 if saturation occurred. 
*******************************************************************************/
float march_filter(d_filter_t* filter, float new_input){
	int i = 0;
	
	if(filter->initialized != 1){
		printf("ERROR: filter not initialized yet\n");
		return -1;
	}
	
	insert_new_ring_buf_value(&filter->in_buf, new_input);
	filter->newest_input = new_input;

	// evaluate the difference equation
	float new_output = 0;
	float input_i, output_i;
	for(i=0; i<=(filter->order); i++){
		input_i = get_ring_buf_value(&filter->in_buf,i);
		new_output += filter->gain * filter->numerator.data[i] * input_i;
	}
	for(i=1; i<=(filter->order); i++){
		output_i = get_ring_buf_value(&filter->out_buf,i-1);
		new_output -= filter->denominator.data[i] * output_i; 
	}
	
	// scale in case denominator doesn't have a leading 1
	new_output = new_output/filter->denominator.data[0];
	
	// soft start limits
	if(filter->soft_start_en && filter->step < filter->soft_start_steps){
		float a=filter->saturation_max*(filter->step/filter->soft_start_steps);
		float b=filter->saturation_min*(filter->step/filter->soft_start_steps);
		if(new_output > a) new_output = a;
		if(new_output < b) new_output = b;
	}

	// saturate and set flag
	if(filter->saturation_en){
		if(new_output > filter->saturation_max){
			new_output = filter->saturation_max;
			filter->saturation_flag=1;
		}
		else if(new_output < filter->saturation_min){
			new_output = filter->saturation_min;
			filter->saturation_flag=1;
		}
		else{
			filter->saturation_flag=0;
		}
	}
	
	// record the output to filter struct and ring buffer
	filter->newest_output = new_output;
	insert_new_ring_buf_value(&filter->out_buf, new_output);
	// increment steps
	filter->step++;
	return new_output;
}

/*******************************************************************************
* int reset_filter(d_filter_t* filter)
*
* resets all inputs and outputs to 0
*******************************************************************************/
int reset_filter(d_filter_t* filter){
	reset_ring_buf(&filter->in_buf);
	reset_ring_buf(&filter->out_buf);
	filter->newest_input = 0;
	filter->newest_output = 0;
	filter->step = 0;
	return 0;
}

/*******************************************************************************
* int enable_saturation(d_filter_t* filter, float sat_min, float sat_max)
*
* If saturation is enabled for a specified filter, the filter will automatically
* bound the output between min and max. You may ignore this function if you wish
* the filter to run unbounded.
*******************************************************************************/
int enable_saturation(d_filter_t* filter, float min, float max){
	if(filter->initialized != 1){
		printf("ERROR: filter not initialized yet\n");
		return -1;
	}
	if(min>=max){
		printf("ERORR: saturation max must be > min\n");
		return -1;
	}
	filter->saturation_en = 1;
	filter->saturation_min = min;
	filter->saturation_max = max;
	return 0;
}

/*******************************************************************************
* int enable_soft_start(d_filter_t* filter, float seconds)
*
* Enables soft start function where the output limit is gradually increased 
* for the given number of seconds up to the normal saturation value. Saturation
* must already be enabled for this to work.
*******************************************************************************/
int enable_soft_start(d_filter_t* filter, float seconds){
	if(filter->initialized != 1){
		printf("ERROR: filter not initialized yet\n");
		return -1;
	}
	if(filter->saturation_en != 1){
		printf("ERROR: saturation must be enabled to use soft start.\n");
		return -1;
	}
	filter->soft_start_en = 1;
	filter->soft_start_steps = seconds/filter->dt;
	
	return 0;
}

/*******************************************************************************
* int did_filter_saturate(d_filter_t* filter)
*
* Returns 1 if the filter saturated the last time step. Returns 0 otherwise.
*******************************************************************************/
int did_filter_saturate(d_filter_t* filter){
	if(filter->initialized != 1){
		printf("ERROR: filter not initialized yet\n");
		return -1;
	}
	return filter->saturation_flag;
	return 0;
}


/*******************************************************************************
* float previous_filter_input(d_filter_t* filter, int steps)
*
* Returns the input 'steps' back in time. Steps = 0 returns most recent input.
*******************************************************************************/
float previous_filter_input(d_filter_t* filter, int steps){
	return get_ring_buf_value(&filter->in_buf, steps);
}

/*******************************************************************************
* float previous_filter_output(d_filter_t* filter, int steps)
*
* Returns the output 'steps' back in time. Steps = 0 returns most recent output.
*******************************************************************************/
float previous_filter_output(d_filter_t* filter, int steps){
	return get_ring_buf_value(&filter->out_buf, steps);
}

/*******************************************************************************
* float newest_filter_output(d_filter_t* filter)
*
* Returns the most recent output from the filter. Alternatively the user could
* access the value from their d_filter_t_t struct with filter.newest_output
*******************************************************************************/
float newest_filter_output(d_filter_t* filter){
	return filter->newest_output;
}

/*******************************************************************************
* float newest_filter_input(d_filter_t* filter)
*
* Returns the most recent input to the filter. Alternatively the user could
* access the value from their d_filter_t_t struct with filter.newest_input
*******************************************************************************/
float newest_filter_input(d_filter_t* filter){
	return filter->newest_input;
}

/*******************************************************************************
* prefill_filter_inputs(d_filter_t* filter, float in)
*
* fills all previous inputs to the filter as if they had been equal to 'in'
* used when initializing filters to non-zero values
*******************************************************************************/
int prefill_filter_inputs(d_filter_t* filter, float in){
	int i;
	if(filter->initialized != 1){
		printf("ERROR: filter not initialized yet\n");
		return -1;
	}
	for(i=0;i<filter->order;i++){
		insert_new_ring_buf_value(&(filter->in_buf), in);
	}
	filter->newest_input = in;
	return 0;
}

/*******************************************************************************
* prefill_filter_outputs(d_filter_t* filter, float out)
*
* fills all previous inputs to the filter as if they had been equal to 'in'
* used when initializing filters to non-zero values
*******************************************************************************/
int prefill_filter_outputs(d_filter_t* filter, float out){
	int i;
	if(filter->initialized != 1){
		printf("ERROR: filter not initialized yet\n");
		return -1;
	}
	for(i=0;i<filter->order;i++){
		insert_new_ring_buf_value(&(filter->out_buf), out);
	}
	filter->newest_output = out;
	return 0;
}

/*******************************************************************************
* int print_filter_details(d_filter_t* filter)
*
* Prints the order, numerator, and denominator coefficients for debugging.
*******************************************************************************/
int print_filter_details(d_filter_t* filter){
	int i;
	if(filter->initialized != 1){
		printf("ERROR: filter not initialized yet\n");
		return -1;
	}
	printf("\n");
	printf("\nOrder: %d\n", filter->order);
	
	printf("num: ");
	for(i=0; i<=filter->order; i++){
		printf("%0.3f  ", filter->numerator.data[i]);
	}
	
	printf("\n    ");
	for(i=0; i <= filter->order; i++){
		printf("-------");
	}
	
	printf("\nden: ");
	for(i=0; i <= filter->order; i++){
		printf("%0.3f  ", filter->denominator.data[i]);
	}
	printf("\n");
	return 0;
}

/*******************************************************************************
* d_filter_t multiply_filters(d_filter_t f1, d_filter_t f2)
*
* 
*******************************************************************************/
d_filter_t multiply_filters(d_filter_t f1, d_filter_t f2){
	d_filter_t out = create_empty_filter();
	if(f1.initialized!=1 || f2.initialized!=1){
		printf("ERROR: filter not initialized\n");
		return out;
	}
	if(f1.dt != f2.dt){
		printf("ERROR: filter timestep dt must match when multiplying.\n");
		return out;
	}
	// order of new system is sum of old orders
	int new_order = f1.order + f2.order;
	
	// multiply out the transfer function coefficients
	vector_t newnum = poly_conv(f1.numerator,f2.numerator);
	if(newnum.initialized != 1){
		printf("ERROR:failed to multiply numerator polynomials\n");
		return out;
	}
	vector_t newden = poly_conv(f1.denominator,f2.denominator);
	if(newden.initialized != 1){
		printf("ERROR:failed to multiply denominator polynomials\n");
		return out;
	}
	
	out = create_filter(new_order, f1.dt, &newnum.data[0], &newden.data[0]);
	out.gain = f1.gain * f2.gain;
	destroy_vector(&newden);
	destroy_vector(&newnum);
	return out;
}

/*******************************************************************************
* d_filter_t C2DTustin(vector_t num, vector_t den, float dt, float w)
* 
* Creates a discrete time filter with similar dynamics to a provided continuous
* time transfer function using tustin's approximation with prewarping. 
*
* arguments:
* vector_t num: 	continuous time numerator coefficients
* vector_t den: 	continuous time denominator coefficients
* float dt:			desired timestep of discrete filter
* float w:			prewarping frequency in rad/s
*******************************************************************************/
d_filter_t C2DTustin(vector_t num, vector_t den, float dt, float w){
	int i,j;
	d_filter_t out = create_empty_filter();
	if(!num.initialized || !den.initialized){
		printf("ERROR: vector not initialized yet\n");
		return out;
	}
	float f = 2*(1 - cos(w*dt)) / (w*dt*sin(w*dt));
	float c = 2/(f*dt);
	int   m = num.len - 1;			// highest order of num
	int   n = den.len - 1;			// highest order of den
	float A0;
	vector_t numZ = create_vector(n+1);	// make vectors with den order +1
	vector_t denZ = create_vector(n+1);
	vector_t p1  = create_vector(2);	// (z - 1)
	p1.data[0]   = 1;
	p1.data[1]   = -1;
	vector_t p2  = create_vector(2);	// (z + 1)
	p2.data[0]   = 1;
	p2.data[1]   = 1;
	vector_t temp, v1, v2;
	
	// from zeroth up to and including mth
	for(i=0;i<=m;i++){				
		v1 = poly_power(p1,m-i);
		v2 = poly_power(p2,n-m+i);
		temp = poly_conv(v1,v2);
		destroy_vector(&v1);
		destroy_vector(&v2);
		for(j=0;j<n+1;j++){
			numZ.data[j] += num.data[i]*pow(c,m-i)*temp.data[j];
		}
		destroy_vector(&temp);
	}
	for(i=0;i<=n;i++){
		v1 = poly_power(p1,n-i);
		v2 = poly_power(p2,i);
		temp = poly_conv(v1,v2);
		destroy_vector(&v1);
		destroy_vector(&v2);
		for(j=0;j<n+1;j++){
			denZ.data[j] += den.data[i]*pow(c,n-i)*temp.data[j];
		}
		destroy_vector(&temp);
	}
	A0 = denZ.data[0];
	for(i=0;i<n+1;i++){
		numZ.data[i] = numZ.data[i]/A0;
		denZ.data[i] = denZ.data[i]/A0;
	}
	out = create_filter(n,dt,numZ.data,denZ.data);
	destroy_vector(&numZ);
	destroy_vector(&denZ);
	destroy_vector(&p1);
	destroy_vector(&p2);
	return out;
}

/*******************************************************************************
* d_filter_t create_first_order_lowpass(float dt, float time_constant)
*
* Returns a configured and ready to use d_filter_t_t struct with a first order
* low pass transfer function. dt is in units of seconds and time_constant is 
* the number of seconds it takes to rise to 63.4% of a steady-state input.
*******************************************************************************/
d_filter_t create_first_order_lowpass(float dt, float time_constant){
	float lp_const = dt/time_constant;
	float numerator[]   = {lp_const, 0};
	float denominator[] = {1, lp_const-1};
	return create_filter(1,dt,numerator,denominator);
}

/*******************************************************************************
* d_filter_t create_first_order_highpass(float dt, float time_constant)
*
* Returns a configured and ready to use d_filter_t_t struct with a first order
* high pass transfer function. dt is in units of seconds and time_constant is 
* the number of seconds it takes to decay by 63.4% of a steady-state input.
*******************************************************************************/
d_filter_t create_first_order_highpass(float dt, float time_constant){
	float hp_const = dt/time_constant;
	float numerator[] = {1-hp_const, hp_const-1};
	float denominator[] = {1,hp_const-1};
	return create_filter(1,dt,numerator,denominator);
}

/*******************************************************************************
* d_filter_t create_butterworth_lowpass(int order, float dt, float wc)
*
* Returns a configured and ready to use d_filter_t_t struct with the transfer
* function for Butterworth low pass filter of order N and cutoff wc.
*******************************************************************************/
d_filter_t create_butterworth_lowpass(int order, float dt, float wc){
	vector_t A = poly_butter(order,wc);
	vector_t B = create_vector(1);
	B.data[0] = 1;
	return C2DTustin(B, A, dt, wc);
}

/*******************************************************************************
* d_filter_t create_butterworth_highpass(int order, float dt, float wc)
*
* Returns a configured and ready to use d_filter_t_t struct with the transfer
* function for Butterworth low pass filter of order N and cutoff wc.
*******************************************************************************/
d_filter_t create_butterworth_highpass(int order, float dt, float wc){
	vector_t A = poly_butter(order,wc);
	vector_t B = create_vector(order + 1);
	B.data[0] = 1;
	return C2DTustin(B, A, dt, wc);
}

/*******************************************************************************
* d_filter_t create_integrator(float dt)
*
* Returns a configured and ready to use d_filter_t_t struct with the transfer
* function for a first order time integral.
*******************************************************************************/
d_filter_t create_integrator(float dt){
	float numerator[]   = {0, dt};
	float denominator[] = {1, -1};
	return create_filter(1,dt,numerator,denominator);
}

/*******************************************************************************
* d_filter_t create_double_integrator(float dt)
*
* Returns a configured and ready to use d_filter_t_t struct with the transfer
* function for a first order time integral.
*******************************************************************************/
d_filter_t create_double_integrator(float dt){
	float numerator[]   = {0, 0, dt*dt};
	float denominator[] = {1, -2, 1};
	return create_filter(2,dt,numerator,denominator);
}

/*******************************************************************************
* d_filter_t create_pid(float kp, float ki, float kd, float Tf, float dt)
*
* discrete-time implementation of a parallel PID controller with rolloff.
* This is equivalent to the Matlab function: C = pid(Kp,Ki,Kd,Tf,Ts)
*
* We cannot implement a pure differentiator with a discrete transfer function
* so this filter has high frequency rolloff with time constant Tf. Smaller Tf
* results in less rolloff, but Tf must be greater than dt/2 for stability.
*******************************************************************************/
d_filter_t create_pid(float kp, float ki, float kd, float Tf, float dt){
	if(Tf <= dt/2){
		printf("WARNING: Tf must be > dt/2 for stability\n");
		Tf=dt; // set to reasonable value
	}
	// if ki==0, return a PD filter with rolloff
	if(ki==0){
		float numerator[] = {(kp*Tf+kd)/Tf, 
							-(((ki*dt-kp)*(dt-Tf))+kd)/Tf};
		float denominator[] = 	{1, 
								-(Tf-dt)/Tf};
		return create_filter(1,dt,numerator,denominator);
	}
	//otherwise PID with roll off
	else{
		float numerator[] = {(kp*Tf+kd)/Tf, 
							(ki*dt*Tf + kp*(dt-Tf) - kp*Tf - 2.0*kd)/Tf,
							(((ki*dt-kp)*(dt-Tf))+kd)/Tf};
		float denominator[] = 	{1, 
								(dt-(2.0*Tf))/Tf, 
								(Tf-dt)/Tf};
		return create_filter(2,dt,numerator,denominator);
	}
}






