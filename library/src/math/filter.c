/**
 * @file filter.c
 * @brief      This is a collection of functions for generating and implementing
 *             discrete SISO filters for arbitrary transfer functions.
 *
 * @author     James Strawson
 * @date       2016
 */

#include <stdio.h>
#include <math.h>
#include <string.h> // for memset
#include <stdlib.h>
#include <stdint.h>

#include <rc/math/filter.h>
#include <rc/math/polynomial.h>

// preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)
#define likely(x)	__builtin_expect (!!(x), 1)


// local function
static int print_poly_z(rc_vector_t v)
{
	int i;
	// utf8 characters for superscript digits
	static char *super[] = {"\xe2\x81\xb0", "\xc2\xb9", "\xc2\xb2",
		"\xc2\xb3", "\xe2\x81\xb4", "\xe2\x81\xb5", "\xe2\x81\xb6",
		"\xe2\x81\xb7", "\xe2\x81\xb8", "\xe2\x81\xb9"};
	if(unlikely(v.len>10)){
		fprintf(stderr,"ERROR in rc_filter_print, filter order must be <=10\n");
		return -1;
	}
	for(i=0;i<(v.len-2);i++) printf("%7.4fz%s + ",v.d[i],super[v.len-i-1]);
	if(v.len>=2) printf("%7.4fz  + ",v.d[v.len-2]);
	printf("%7.4f\n", v.d[v.len-1]);
	return 0;
}


rc_filter_t rc_filter_empty()
{
	rc_filter_t f;
	f.order		= 0;
	f.dt		= 0.0f;
	f.gain		= 1.0f;
	f.num		= rc_vector_empty();
	f.den		= rc_vector_empty();
	f.sat_en	= 0;
	f.sat_min	= 0.0f;
	f.sat_max	= 0.0f;
	f.sat_flag	= 0;
	f.ss_en		= 0;
	f.ss_steps	= 0;
	f.in_buf	= rc_ringbuf_empty();
	f.out_buf	= rc_ringbuf_empty();
	f.newest_input	= 0.0f;
	f.newest_output = 0.0f;
	f.step		= 0;
	f.initialized	= 0;
	return f;
}

int rc_filter_alloc(rc_filter_t* f, rc_vector_t num, rc_vector_t den, float dt)
{
	// sanity checks
	if(unlikely(dt<=0.0)){
		fprintf(stderr,"ERROR in rc_filter_alloc, dt must be >0\n");
		return -1;
	}
	if(unlikely(!num.initialized||!den.initialized)){
		fprintf(stderr,"ERROR in rc_filter_alloc, vector uninitialized\n");
		return -1;
	}
	if(unlikely(num.len>den.len)){
		fprintf(stderr,"ERROR in rc_filter_alloc, improper transfer function\n");
		return -1;
	}
	if(unlikely(den.d[0]==0.0f)){
		fprintf(stderr,"ERROR in rc_filter_alloc, first coefficient in denominator is 0\n");
		return -1;
	}
	// free existing memory, this also zeros out all fields
	rc_filter_free(f);
	// move in vectors
	if(unlikely(rc_vector_duplicate(num,&f->num))){
		fprintf(stderr,"ERROR in rc_filter_alloc, failed to duplicate numerator\n");
		return -1;
	}
	if(unlikely(rc_vector_duplicate(den,&f->den))){
		fprintf(stderr,"ERROR in rc_filter_alloc, failed to duplicate denominator\n");
		rc_vector_free(&f->num);
		return -1;
	}
	// allocate buffers
	if(unlikely(rc_ringbuf_alloc(&f->in_buf,den.len))){
		fprintf(stderr,"ERROR in rc_filter_alloc, failed to allocate ring buffer\n");
		rc_vector_free(&f->num);
		rc_vector_free(&f->den);
		return -1;
	}
	if(unlikely(rc_ringbuf_alloc(&f->out_buf,den.len))){
		fprintf(stderr,"ERROR in rc_filter_alloc, failed to allocate ring buffer\n");
		rc_vector_free(&f->num);
		rc_vector_free(&f->den);
		rc_ringbuf_free(&f->in_buf);
		return -1;
	}
	// populate remaining values, everything else zero'd by rc_filter_free
	f->dt=dt;
	f->order=den.len-1;
	f->initialized=1;
	return 0;
}

int rc_filter_alloc_from_arrays(rc_filter_t* f,float dt,float* num,int numlen,\
							float* den,int denlen)
{
	// sanity checks
	if(unlikely(numlen<1 || denlen<1)){
		fprintf(stderr,"ERROR in rc_filter_alloc_from_arrays, numlen & denlen must be >=1\n");
		return -1;
	}
	if(unlikely(numlen>denlen)){
		fprintf(stderr,"ERROR in rc_filter_alloc_from_arrays, improper transfer function\n");
		return -1;
	}
	if(unlikely(num==NULL || den==NULL || f==NULL)){
		fprintf(stderr,"ERROR in rc_filter_alloc_from_arrays, received null pointer\n");
		return -1;
	}
	if(unlikely(dt<0.0f)){
		fprintf(stderr,"ERROR in rc_filter_alloc_from_arrays, dt must be >0\n");
		return -1;
	}
	if(unlikely(den[0]==0.0f)){
		fprintf(stderr,"ERROR in rc_filter_alloc_from_arrays, first coefficient in denominator is 0\n");
		return -1;
	}
	// free existing memory, this also zeros out all fields
	rc_filter_free(f);
	// copy numerator and denominators over
	if(unlikely(rc_vector_from_array(&f->num,num,numlen))){
		fprintf(stderr,"ERROR in rc_filter_alloc_from_arrays, failed to alloc vector\n");
		return -1;
	}
	if(unlikely(rc_vector_from_array(&f->den,den,denlen))){
		fprintf(stderr,"ERROR in rc_filter_alloc_from_arrays, failed to alloc vector\n");
		rc_vector_free(&f->num);
		return -1;
	}
	// allocate buffers
	if(unlikely(rc_ringbuf_alloc(&f->in_buf,denlen))){
		fprintf(stderr,"ERROR in rc_filter_alloc, failed to allocate ring buffer\n");
		rc_vector_free(&f->num);
		rc_vector_free(&f->den);
		return -1;
	}
	if(unlikely(rc_ringbuf_alloc(&f->out_buf,denlen))){
		fprintf(stderr,"ERROR in rc_filter_alloc, failed to allocate ring buffer\n");
		rc_vector_free(&f->num);
		rc_vector_free(&f->den);
		rc_ringbuf_free(&f->in_buf);
		return -1;
	}
	// populate remaining values, everything else zero'd by rc_filter_free
	f->dt=dt;
	f->order=denlen-1;
	f->initialized=1;
	return 0;
}


int rc_filter_free(rc_filter_t* f)
{
	if(unlikely(f==NULL)){
		fprintf(stderr, "ERROR in rc_filter_free, received NULL pointer\n");
		return -1;
	}
	rc_ringbuf_free(&f->in_buf);
	rc_ringbuf_free(&f->out_buf);
	rc_vector_free(&f->num);
	rc_vector_free(&f->den);
	*f = rc_filter_empty();
	return 0;
}


float rc_filter_march(rc_filter_t* f, float new_input)
{
	int i, rel_deg;
	float new_out = 0.0f;
	// sanity checks
	if(unlikely(!f->initialized)){
		printf("ERROR in rc_filter_march, filter uninitialized\n");
		return -1.0f;
	}
	// log new input
	rc_ringbuf_insert(&f->in_buf, new_input);
	f->newest_input = new_input;
	// relative degree should never be negative as rc_filter_alloc checks
	// for improper transfer functions
	rel_deg = f->den.len - f->num.len;
	// evaluate the difference equation
	for(i=0; i<(f->num.len); i++){
		new_out+=f->gain*f->num.d[i]*rc_ringbuf_get_value(&f->in_buf, i+rel_deg);
	}
	for(i=0; i<(f->order); i++){
		new_out-=f->den.d[i+1]*rc_ringbuf_get_value(&f->out_buf, i);
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
	rc_ringbuf_insert(&f->out_buf, new_out);
	// increment steps
	f->step++;
	return new_out;
}


int rc_filter_reset(rc_filter_t* f)
{
	if(unlikely(!f->initialized)){
		fprintf(stderr,"ERROR in rc_filter_reset, filter uninitialized\n");
		return -1;
	}
	rc_ringbuf_reset(&f->in_buf);
	rc_ringbuf_reset(&f->out_buf);
	f->newest_input	= 0.0f;
	f->newest_output = 0.0f;
	f->sat_flag = 0;
	f->step = 0;
	return 0;
}



int rc_filter_print(rc_filter_t f)
{
	int i;
	if(unlikely(!f.initialized)){
		fprintf(stderr,"ERROR in rc_filter_print, filter not initialized yet\n");
		return -1;
	}
	if(unlikely(f.order>9)){
		fprintf(stderr,"ERROR in rc_filter_print, filter order must be <=10\n");
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


int rc_filter_enable_saturation(rc_filter_t* f, float min, float max)
{
	if(unlikely(!f->initialized)){
		fprintf(stderr, "ERROR in rc_filter_enable_saturation, filter uninitialized\n");
		return -1;
	}
	if(unlikely(min>=max)){
		fprintf(stderr, "ERORR in rc_filter_enable_saturation, max must be > min\n");
		return -1;
	}
	f->sat_en	= 1;
	f->sat_min	= min;
	f->sat_max	= max;
	return 0;
}


int rc_filter_get_saturation_flag(rc_filter_t* f)
{
	if(unlikely(!f->initialized)){
		fprintf(stderr,"ERROR in rc_filter_get_saturation_flag, filter uninitialized\n");
		return -1;
	}
	return f->sat_flag;
}


int rc_filter_enable_soft_start(rc_filter_t* f, float seconds)
{
	if(unlikely(!f->initialized)){
		fprintf(stderr,"ERROR in rc_filter_enable_soft_start, filter uninitialized\n");
		return -1;
	}
	if(unlikely(seconds<=0.0f)){
		fprintf(stderr,"ERROR in rc_filter_enable_soft_start, seconds must be >=0\n");
		return -1;
	}
	if(unlikely(!f->sat_en)){
		fprintf(stderr,"ERROR in rc_filter_enable_soft_start, saturation must be enabled first\n");
		return -1;
	}
	f->ss_en	= 1;
	f->ss_steps	= seconds/f->dt;
	return 0;
}


float rc_filter_previous_input(rc_filter_t* f, int steps)
{
	if(unlikely(!f->initialized)){
		fprintf(stderr,"ERROR in rc_filter_previous_input, filter uninitialized\n");
		return -1.0f;
	}
	return rc_ringbuf_get_value(&f->in_buf, steps);
}


float rc_filter_previous_output(rc_filter_t* f, int steps)
{
	if(unlikely(!f->initialized)){
		fprintf(stderr,"ERROR in rc_filter_previous_output, filter uninitialized\n");
		return -1.0f;
	}
	return rc_ringbuf_get_value(&f->out_buf, steps);
}



int rc_filter_prefill_inputs(rc_filter_t* f, float in)
{
	int i;
	if(unlikely(!f->initialized)){
		fprintf(stderr,"ERROR in rc_filter_prefill_inputs, filter uninitialized\n");
		return -1;
	}
	for(i=0;i<=f->order;i++){
		rc_ringbuf_insert(&f->in_buf, in);
	}
	f->newest_input = in;
	return 0;
}


int rc_filter_prefill_outputs(rc_filter_t* f, float out)
{
	int i;
	if(unlikely(!f->initialized)){
		fprintf(stderr,"ERROR in rc_filter_prefill_outputs, filter uninitialized\n");
		return -1;
	}
	for(i=0;i<=f->order;i++){
		rc_ringbuf_insert(&(f->out_buf), out);
	}
	f->newest_output = out;
	return 0;
}


int rc_filter_multiply(rc_filter_t f1, rc_filter_t f2, rc_filter_t* f3)
{
	rc_vector_t newnum = rc_vector_empty();
	rc_vector_t newden = rc_vector_empty();
	// sanity checks
	if(unlikely(!f1.initialized||!f2.initialized)){
		fprintf(stderr,"ERROR in rc_filter_multiply, filter uninitialized\n");
		return -1;
	}
	if(unlikely(f1.dt!=f2.dt)){
		fprintf(stderr,"ERROR in rc_filter_multiply, timestep dt must match\n");
		return -1;
	}
	// multiply out the transfer function coefficients
	if(unlikely(rc_poly_conv(f1.num,f2.num,&newnum))){
		fprintf(stderr,"ERROR in rc_filter_multiply, failed to polyconv\n");
		return -1;
	}
	if(unlikely(rc_poly_conv(f1.den,f2.den,&newden))){
		fprintf(stderr,"ERROR in rc_filter_multiply, failed to polyconv\n");
		rc_vector_free(&newnum);
		return -1;
	}
	// create the filter
	if(unlikely(rc_filter_alloc(f3,newnum,newden,f1.dt))){
		fprintf(stderr,"ERROR in rc_filter_multiply, failed to alloc filter\n");
		return -1;
	}
	f3->gain = f1.gain * f2.gain;
	rc_vector_free(&newnum);
	rc_vector_free(&newden);
	return 0;
}


int rc_filter_c2d_tustin(rc_filter_t* f,rc_vector_t num,rc_vector_t den,float dt,float w)
{
	int i,j,m,n;
	double a,c, A0;
	rc_vector_t numZ	= rc_vector_empty();
	rc_vector_t denZ	= rc_vector_empty();
	rc_vector_t p1		= rc_vector_empty();
	rc_vector_t p2		= rc_vector_empty();
	rc_vector_t temp	= rc_vector_empty();
	rc_vector_t v1		= rc_vector_empty();
	rc_vector_t v2		= rc_vector_empty();
	// sanity checks
	if(unlikely(!num.initialized||!den.initialized)){
		fprintf(stderr,"ERROR in rc_filter_c2d_tustin, vector uninitialized\n");
		return -1;
	}
	if(unlikely(dt<=0.0f)){
		fprintf(stderr,"ERROR in rc_filter_c2d_tustin, dt must be positive\n");
		return -1;
	}
	if(unlikely(w>(M_PI/dt))){
		fprintf(stderr,"ERROR in rc_filter_c2d_tustin, w larger than nyquist frequency\n");
		return -1;
	}
	a = 2.0*(1.0 - cos(w*dt)) / (w*dt*sin(w*dt));
	c = 2.0/(a*dt);
	m = num.len - 1;			// order of num
	n = den.len - 1;			// order of den
	rc_vector_zeros(&numZ,n+1);
	rc_vector_zeros(&denZ,n+1);
	rc_vector_alloc(&p1,2);	// (z - 1)
	p1.d[0]		= 1.0f;
	p1.d[1]		= -1.0f;
	rc_vector_alloc(&p2,2);	// (z + 1)
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
	rc_vector_free(&p1);
	rc_vector_free(&p2);
	rc_vector_free(&temp);
	rc_vector_free(&v1);
	rc_vector_free(&v2);
	// make the filter
	if(unlikely(rc_filter_alloc(f, numZ, denZ, dt))){
		fprintf(stderr, "ERROR in rc_filter_c2d_tustin, failed to alloc filter\n");
		rc_vector_free(&numZ);
		rc_vector_free(&denZ);
		return -1;
	}
	rc_vector_free(&numZ);
	rc_vector_free(&denZ);
	return 0;
}


int rc_filter_first_order_lowpass(rc_filter_t* f, float dt, float time_constant)
{
	float lp_const;
	float num[1], den[2];
	// sanity checks
	if(unlikely(time_constant<=0.0f)){
		fprintf(stderr, "ERROR in rc_filter_first_order_lowpass, time constant must be >0\n");
		return -1;
	}
	if(unlikely(dt<=0.0f)){
		fprintf(stderr, "ERROR in rc_filter_first_order_lowpass, dt must be >0\n");
		return -1;
	}
	lp_const = dt/time_constant;
	num[0] = lp_const;
	den[0] = 1.0f;
	den[1] = lp_const -1.0f;
	// make the filter
	if(unlikely(rc_filter_alloc_from_arrays(f,dt,num,1,den,2))){
		fprintf(stderr, "ERROR in rc_filter_first_order_lowpass, failed to alloc filter\n");
		return -1;
	}
	return 0;
}


int rc_filter_first_order_highpass(rc_filter_t* f, float dt, float time_constant)
{
	float hp_const;
	float num[2], den[2];
	// sanity checks
	if(unlikely(time_constant<=0.0f)){
		fprintf(stderr, "ERROR in rc_filter_first_order_highpass, time constant must be >0\n");
		return -1;
	}
	if(unlikely(dt<=0.0f)){
		fprintf(stderr, "ERROR in rc_filter_first_order_highpass, dt must be >0\n");
		return -1;
	}
	hp_const = dt/time_constant;
	num[0] = 1.0f-hp_const;
	num[1] = hp_const-1.0f;
	den[0] = 1.0f;
	den[1] = hp_const-1.0f;
	// make the filter
	if(unlikely(rc_filter_alloc_from_arrays(f,dt,num,2,den,2))){
		fprintf(stderr, "ERROR in rc_filter_first_order_highpass, failed to alloc filter\n");
		return -1;
	}
	return 0;
}


int rc_filter_butterworth_lowpass(rc_filter_t* f, int order, float dt, float wc)
{
	rc_vector_t num = rc_vector_empty();
	rc_vector_t den = rc_vector_empty();
	if(unlikely(order<1)){
		fprintf(stderr, "ERROR in rc_filter_butterworth_lowpass, order must be >=1\n");
		return -1;
	}
	if(unlikely(rc_poly_butter(order,wc,&den))){
		fprintf(stderr, "ERROR in rc_filter_butterworth_lowpass, failed to find butterworth polynomial\n");
		return -1;
	}
	rc_vector_alloc(&num,1);
	num.d[0] = 1.0f;
	if(unlikely(rc_filter_c2d_tustin(f,num,den,dt,wc))){
		fprintf(stderr, "ERROR in rc_filter_butterworth_lowpass, failed to c2d_tustin\n");
		rc_vector_free(&num);
		rc_vector_free(&den);
		return -1;
	}
	rc_vector_free(&num);
	rc_vector_free(&den);
	return 0;
}


int rc_filter_butterworth_highpass(rc_filter_t* f, int order, float dt, float wc)
{
	rc_vector_t num = rc_vector_empty();
	rc_vector_t den = rc_vector_empty();
	if(unlikely(order<1)){
		fprintf(stderr, "ERROR in rc_filter_butterworth_highpass, order must be >=1\n");
		return -1;
	}
	if(unlikely(rc_poly_butter(order,wc,&den))){
		fprintf(stderr, "ERROR in rc_filter_butterworth_highpass, failed to find butterwoth polynomial\n");
		return -1;
	}
	// numerator consists of all zeros at the origin
	rc_vector_zeros(&num,order+1);
	num.d[0] = 1.0f;

	if(unlikely(rc_filter_c2d_tustin(f,num,den,dt,wc))){
		fprintf(stderr, "ERROR in rc_filter_butterworth_highpass, failed to c2d_tustin\n");
		rc_vector_free(&num);
		rc_vector_free(&den);
		return -1;
	}
	rc_vector_free(&num);
	rc_vector_free(&den);
	return 0;
}


int rc_filter_moving_average(rc_filter_t* f, int samples, int dt)
{
	int i;
	rc_vector_t num = rc_vector_empty();
	rc_vector_t den = rc_vector_empty();
	// sanity checks
	if(unlikely(samples<2)){
		fprintf(stderr,"ERROR in rc_filter_moving_average, samples must be >=2\n");
		return -1;
	}
	rc_vector_alloc(&num,samples);
	rc_vector_alloc(&den,samples);
	// fill in coefficients
	for(i=0;i<samples;i++){
		num.d[i] = 1.0f/samples;
		den.d[i] = 0.0;
	}
	den.d[0] = 1.0;
	// make the filter
	if(unlikely(rc_filter_alloc(f,num,den,dt))){
		fprintf(stderr, "ERROR in rc_filter_moving_average, failed to alloc filter\n");
		rc_vector_free(&num);
		rc_vector_free(&den);
		return -1;
	}
	rc_vector_free(&num);
	rc_vector_free(&den);
	return 0;
}


int rc_filter_integrator(rc_filter_t *f, float dt)
{
	rc_vector_t num = rc_vector_empty();
	rc_vector_t den = rc_vector_empty();
	// sanity checks
	if(unlikely(dt<=0.0f)){
		fprintf(stderr, "ERROR in rc_filter_integrator, dt must be >0\n");
		return -1;
	}
	rc_vector_alloc(&num,1);
	rc_vector_alloc(&den,2);
	num.d[0] = dt;
	den.d[0] = 1.0;
	den.d[1] = -1.0;
	// make the filter
	if(unlikely(rc_filter_alloc(f,num,den,dt))){
		fprintf(stderr, "ERROR in rc_filter_integrator, failed to alloc filter\n");
		rc_vector_free(&num);
		rc_vector_free(&den);
		return -1;
	}
	rc_vector_free(&num);
	rc_vector_free(&den);
	return 0;
}


int rc_filter_double_integrator(rc_filter_t* f, float dt)
{
	rc_vector_t num = rc_vector_empty();
	rc_vector_t den = rc_vector_empty();
	// sanity checks
	if(unlikely(dt<=0.0f)){
		fprintf(stderr, "ERROR in rc_filter_double_integrator, dt must be >0\n");
		return -1;
	}
	rc_vector_alloc(&num,1);
	rc_vector_alloc(&den,3);
	num.d[0] = dt*dt;
	den.d[0] = 1.0;
	den.d[1] = -2.0;
	den.d[2] = 1.0;
	// make the filter
	if(unlikely(rc_filter_alloc(f,num,den,dt))){
		fprintf(stderr, "ERROR in rc_filter_double_integrator, failed to alloc filter\n");
		rc_vector_free(&num);
		rc_vector_free(&den);
		return -1;
	}
	rc_vector_free(&num);
	rc_vector_free(&den);
	return 0;
}

int rc_filter_pid(rc_filter_t* f,float kp,float ki,float kd,float Tf,float dt)
{
	rc_vector_t num = rc_vector_empty();
	rc_vector_t den = rc_vector_empty();
	// sanity checks
	if(unlikely(dt<0.0f)){
		fprintf(stderr,"ERROR in rc_filter_pid, dt must be >0\n");
		return -1;
	}
	if(unlikely(Tf <= dt/2)){
		printf("ERROR in rc_filter_pid, Tf must be > dt/2 for stability\n");
		return -1;
	}
	// if ki==0, return a 1st order PD filter with rolloff
	if(ki==0.0f){
		rc_vector_alloc(&num,2);
		rc_vector_alloc(&den,2);
		num.d[0] = (kp*Tf+kd)/Tf;
		num.d[1] = -(((ki*dt-kp)*(dt-Tf))+kd)/Tf;
		den.d[0] = 1.0f;
		den.d[1] = -(Tf-dt)/Tf;
	}
	//otherwise 2nd order PID with roll off
	else{
		rc_vector_alloc(&num,3);
		rc_vector_alloc(&den,3);
		num.d[0] = (kp*Tf+kd)/Tf;
		num.d[1] = (ki*dt*Tf + kp*(dt-Tf) - kp*Tf - 2.0*kd)/Tf;
		num.d[2] = (((ki*dt-kp)*(dt-Tf))+kd)/Tf;
		den.d[0] = 1.0f;
		den.d[1] = (dt-(2.0*Tf))/Tf;
		den.d[2] = (Tf-dt)/Tf;
	}
	// make the filter
	if(unlikely(rc_filter_alloc(f,num,den,dt))){
		fprintf(stderr, "ERROR in rc_filter_pid, failed to alloc filter\n");
		rc_vector_free(&num);
		rc_vector_free(&den);
		return -1;
	}
	rc_vector_free(&num);
	rc_vector_free(&den);
	return 0;
}
