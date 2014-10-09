/*
Copyright (c) 2014, James Strawson
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*/


/* 
See filter_lib.h for API
*/
#include "filter_lib.h" 




float marchFilter(discrete_filter* filter, float new_input){
	int i = 0;
	for(i=filter->order; i>0; i--){
		filter->inputs[i] = filter->inputs[i-1];
		filter->outputs[i] = filter->outputs[i-1];
		
	}
	filter->inputs[0] = new_input;

	// evaluate the difference equation
	float new_output = 0;
	for(i=0; i<(filter->order+1); i++){
		new_output += filter->prescaler * filter->numerator[i] * filter->inputs[i];
	}
	for(i=1; i<(filter->order+1); i++){
		new_output -= filter->denominator[i] * filter->outputs[i]; 
	}
	// scale in case denominator doesn't have a leading 1
	filter->outputs[0] = new_output/filter->denominator[0];	
	filter->current_output = filter->outputs[0];
	filter->last_input = new_input;
	
	return filter->current_output;
}


int saturateFilter(discrete_filter* filter, float min, float max){
	if(filter->outputs[0] > max){
		filter->outputs[0] = max;
		return 1;
	}
	else if(filter->outputs[0] < min){
		filter->outputs[0] = min;
		return 1;
	}
	else return 0;
}

int zeroFilter(discrete_filter* filter){
	int i = 0;
	
	for(i=filter->order; i>=0; i--){
		filter->inputs[i] = 0;
		filter->outputs[i] = 0;
	}
	filter->last_input = 0;
	filter->current_output = 0;
	return 0;
}

int preFillFilter(discrete_filter* filter, float input){
	int i = 0;
	for(i=filter->order; i>=0; i--){
		filter->inputs[i] = input;
	}
	filter->last_input = input; 
	return 0;

}

// allocate memory for a filter of specified order
// Fill with transfer function constants
discrete_filter generateFilter(int order,float dt,float num[],float den[]){
	discrete_filter filter;
	filter.order = order;
	filter.prescaler = 1;
	filter.last_input = 0;
	filter.current_output = 0;
	int i = 0;
	for(i=0; i<(order+1); i++){
		filter.numerator[i] = num[i];
		filter.denominator[i] = den[i];
		filter.inputs[i] = 0;
		filter.outputs[i] = 0;
	}
	return filter;
}

discrete_filter generateFirstOrderLowPass(float dt, float time_constant){
	const float lp_const = dt/time_constant;
	float numerator[]   = {lp_const, 0};
	float denominator[] = {1, lp_const-1};
	return generateFilter(1,dt,numerator,denominator);
}

discrete_filter generateFirstOrderHighPass(float dt, float time_constant){
	float hp_const = dt/time_constant;
	float numerator[] = {1-hp_const, hp_const-1};
	float denominator[] = {1,hp_const-1};
	return generateFilter(1,dt,numerator,denominator);
}


discrete_filter generateIntegrator(float dt){
	float numerator[]   = {0, dt};
	float denominator[] = {1, -1};
	return generateFilter(1,dt,numerator,denominator);
}

discrete_filter generatePID(float kp, float ki, float kd, float Tf, float dt){
	if(Tf <= 2*dt){
		printf("Tf must be > 2kd for stability\n");
		return generateFilter(0,dt,0,0);
	}
	// if ki==0, return a PD filter with rolloff
	if(ki==0){
		float numerator[] = {(kp*Tf+kd)/Tf, 
							-(((ki*dt-kp)*(dt-Tf))+kd)/Tf};
		float denominator[] = 	{1, 
								-(Tf-dt)/Tf};
		return generateFilter(1,dt,numerator,denominator);
	}
	//otherwise PID with roll off
	else{
		float numerator[] = {(kp*Tf+kd)/Tf, 
							(ki*dt*Tf + kp*(dt-Tf) - kp*Tf - 2.0*kd)/Tf,
							(((ki*dt-kp)*(dt-Tf))+kd)/Tf};
		float denominator[] = 	{1, 
								(dt-(2.0*Tf))/Tf, 
								(Tf-dt)/Tf};
		return generateFilter(2,dt,numerator,denominator);
	}
}

int printFilterDetails(discrete_filter* filter){
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

