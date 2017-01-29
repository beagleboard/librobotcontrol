
/*******************************************************************************
* float rc_mult_accumulate(float * __restrict__ a, float * __restrict__ b, int n)
* 
* Performs a vector dot product on the contents of a and b over n values.
* This is a dangerous function that could segfault if not used properly. Hence
* it is only for internal use in the RC library. the 'restrict' attributes tell
* the C compiler that the pointers are not aliased which helps the vectorization
* process for optimization with the NEON FPU.
*******************************************************************************/
float rc_mult_accumulate(float * __restrict__ a, float * __restrict__ b, int n){
	int i;
	float sum = 0.0f;
	for(i=0;i<n;i++){
		sum+=a[i]*b[i];
	}
	return sum;
}