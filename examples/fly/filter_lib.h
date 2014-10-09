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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define MAX_ARRAY_SIZE 32

// This is the heart of the library and contains constants and memory about IO data
// a discrete_filter instance has a fixed size but contains pointers to dynamic arrays
// thus it is best to generate an instance using one of the generate functions here
struct discrete_filter{
	int order;
	float dt;
	// input scaling factor usually =1, useful for fast controller tuning
	float prescaler; 
	float numerator[MAX_ARRAY_SIZE];	// points to array of numerator constants
	float denominator[MAX_ARRAY_SIZE];	// points to array 
	float inputs[MAX_ARRAY_SIZE];
	float outputs[MAX_ARRAY_SIZE];
	float last_input;
	float current_output;
};
typedef struct discrete_filter discrete_filter;


/* 
--- March Filter ---
march the filter forward in time one step with new input data
returns new output which could also be accessed with filter.current_output
*/
float marchFilter(discrete_filter* filter, float new_input);


/* 
--- Saturate Filter ---
limit the output of filter to be between min&max
returns 1 if saturation was hit 
returns 0 if output was within bounds
*/
int saturateFilter(discrete_filter* filter, float min, float max);


/*
--- Zero Filter ---
reset all input and output history to 0
*/
int zeroFilter(discrete_filter* filter);

/*
--- PreFill Filter ---
fill the past inputs with the curent input
use before marchFilter when starting to avoid ugly step input
*/
int preFillFilter(discrete_filter* filter, float input);


/*
--- Generate Filter ---
Dynamically allocate memory for a filter of specified order
and set transfer function constants.

Note: A normalized transfer function should have a leading 1 
in the denominator but can be !=1 in this library
*/
discrete_filter generateFilter(int order, float dt,float numerator[],float denominator[]);

						

// time_constant is seconds to rise 63.4% 
discrete_filter generateFirstOrderLowPass(float dt, float time_constant);

// time_constant is seconds to decay 63.4% 
discrete_filter generateFirstOrderHighPass(float dt, float time_constant);

// integrator scaled to loop dt
discrete_filter generateIntegrator(float dt);


// discrete-time implementation of a parallel PID controller with derivative filter
// similar to Matlab pid command
//
// N is the pole location for derivative filter. Must be greater than 2*DT
// smaller N gives faster filter decay
discrete_filter generatePID(float kp, float ki, float kd, float Tf, float dt);

// print order, numerator, and denominator constants
int printFilterDetails(discrete_filter* filter);