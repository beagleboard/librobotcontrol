/*******************************************************************************
* discrete_siso_filter.c
* James Strawson 2016
*
* This is a collection of functions for generating and implementing discrete 
* SISO filters for arbitrary transfer functions. 
*******************************************************************************/

#include "../robotics_cape.h"
#include <stdio.h>

#define DEBUG

/*******************************************************************************
*
*
*
*******************************************************************************/
float march_filter(discrete_filter* filter, float new_input){
	int i = 0;
	
	insert_new_ring_buf_value(&filter->in_buf, new_input);
	filter->last_input = new_input;

	// evaluate the difference equation
	float new_output = 0;
	float input_i, output_i;
	for(i=0; i<=(filter->order); i++){
		input_i = get_ring_buf_value(&filter->in_buf,i);
		new_output += filter->prescaler * filter->numerator[i] * input_i;
	}
	for(i=1; i<=(filter->order); i++){
		output_i = get_ring_buf_value(&filter->out_buf,i-1);
		new_output -= filter->denominator[i] * output_i; 
	}
	
	// scale in case denominator doesn't have a leading 1
	new_output = new_output/filter->denominator[0];
	
	// saturate and set flag
	if(filter->saturation_en){
		if(new_output > filter->saturation_max){
			new_output = filter->saturation_max;
			filter->saturation_flag=1;
			return 1;
		}
		else if(new_output<filter->saturation_min){
			new_output = filter->saturation_min;
			filter->saturation_flag=1;
			return 1;
		}
		else{
			filter->saturation_flag=0;
		}
	}
	
	// record the output to filter struct and ring buffer
	filter->last_output=new_output;
	insert_new_ring_buf_value(&filter->out_buf, new_output);

	return new_output;
}

/*******************************************************************************
*
*
*
*******************************************************************************/
int reset_filter(discrete_filter* filter){
	reset_ring_buf(&filter->in_buf);
	reset_ring_buf(&filter->out_buf);
	filter->last_input = 0;
	filter->last_output = 0;
	return 0;
}


/*
get_previous_input
returns an input with offset 'steps'
steps = 0 returns last input
*/
/*******************************************************************************
*
*
*
*******************************************************************************/
int get_previous_input(discrete_filter* filter, int steps){
	return get_ring_buf_value(&filter->in_buf, steps);
}

/*
get_previous_output
returns a previous output with offset 'steps'
steps = 0 returns last output
*/
/*******************************************************************************
*
*
*
*******************************************************************************/
int get_previous_output(discrete_filter* filter, int steps){
	return get_ring_buf_value(&filter->out_buf, steps);
}


/*******************************************************************************
* discrete_filter generateFilter(int order,float dt,float num[],float den[])
*
* Allocate memory for a filter of specified order & fill with transfer
* function constants. Use enable_saturation immediately after this if you want
* to enable automatic saturation.
*******************************************************************************/
discrete_filter generate_filter(int order,float dt,float num[],float den[]){
	discrete_filter filter;
	filter.order = order;
	filter.prescaler = 1;
	filter.last_input = 0;
	filter.last_output = 0;
	filter.saturation_en = 0;
	filter.saturation_min = 0;
	filter.saturation_max = 0;
	filter.saturation_flag = 0;
	int i = 0;
	for(i=0; i<(order+1); i++){
		filter.numerator[i] = num[i];
		filter.denominator[i] = den[i];
	}
	reset_ring_buf(&filter.in_buf);
	reset_ring_buf(&filter.out_buf);
	return filter;
}

/*******************************************************************************
*
*
*
*******************************************************************************/
int enable_saturation(discrete_filter* filter, float sat_min, float sat_max){
	filter->saturation_en = 1;
	filter->saturation_min = sat_min;
	filter->saturation_max = sat_max;
	return 0;
}

/*******************************************************************************
*
*
*
*******************************************************************************/
discrete_filter generateFirstOrderLowPass(float dt, float time_constant){
	const float lp_const = dt/time_constant;
	float numerator[]   = {lp_const, 0};
	float denominator[] = {1, lp_const-1};
	return generate_filter(1,dt,numerator,denominator);
}

/*******************************************************************************
*
*
*
*******************************************************************************/
discrete_filter generateFirstOrderHighPass(float dt, float time_constant){
	float hp_const = dt/time_constant;
	float numerator[] = {1-hp_const, hp_const-1};
	float denominator[] = {1,hp_const-1};
	return generate_filter(1,dt,numerator,denominator);
}

/*******************************************************************************
*
*
*
*******************************************************************************/
discrete_filter generateIntegrator(float dt){
	float numerator[]   = {0, dt};
	float denominator[] = {1, -1};
	return generate_filter(1,dt,numerator,denominator);
}

/*******************************************************************************
*
*
*
*******************************************************************************/
discrete_filter generatePID(float kp, float ki, float kd, float Tf, float dt){
	if(Tf <= 2*dt){
		printf("Tf must be > 2kd for stability\n");
		return generate_filter(0,dt,0,0);
	}
	// if ki==0, return a PD filter with rolloff
	if(ki==0){
		float numerator[] = {(kp*Tf+kd)/Tf, 
							-(((ki*dt-kp)*(dt-Tf))+kd)/Tf};
		float denominator[] = 	{1, 
								-(Tf-dt)/Tf};
		return generate_filter(1,dt,numerator,denominator);
	}
	//otherwise PID with roll off
	else{
		float numerator[] = {(kp*Tf+kd)/Tf, 
							(ki*dt*Tf + kp*(dt-Tf) - kp*Tf - 2.0*kd)/Tf,
							(((ki*dt-kp)*(dt-Tf))+kd)/Tf};
		float denominator[] = 	{1, 
								(dt-(2.0*Tf))/Tf, 
								(Tf-dt)/Tf};
		return generate_filter(2,dt,numerator,denominator);
	}
}

/*******************************************************************************
*
*
*
*******************************************************************************/
int print_filter_details(discrete_filter* filter){
	int i;
	printf("\n");
	printf("\nOrder: %d\n", filter->order);
	
	printf("num: ");
	for(i=0; i<=filter->order; i++){
		printf("%0.3f  ", filter->numerator[i]);
	}
	
	printf("\n    ");
	for(i=0; i<=filter->order; i++){
		printf("-------");
	}
	
	printf("\nden: ");
	for(i=0; i<=filter->order; i++){
		printf("%0.3f  ", filter->denominator[i]);
	}
	printf("\n");
	return 0;
}
