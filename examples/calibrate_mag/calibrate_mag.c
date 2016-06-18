/*******************************************************************************
* calibrate_mag.c
* James Strawson - 2016
*
* This routine samples the magnetometer and saves a file containing
* the sensor offsets and scaling correction factors
*******************************************************************************/


#include <robotics_cape.h>
#include <useful_includes.h>

// mag data, later split this into 8 quadrants
float X2[] = {  -30, -748,  112,  746,  548, -440 };
float Y2[] = { -783,  105,  815,  119,   93,  205 };
float Z2[] = {  370,  432,  387, -421,  680, -651 };

// reads 3 vectors of x,y,z readings. Returns scaling offsets to array S
// and offsets to array O.
int fit_ellipse(float *x, float *y, float *z, float *S, float *O ){
  int i;
  float A[25], *p;
  float f[5], X[5];
  float k1, k2;

  /* Fill in matrix A */

  p = A;
  for(i = 0; i < 5; i++)
    {
      *p++ = 2.0 * (x[i] - x[i+1]);
      *p++ = y[i+1]*y[i+1] - y[i]*y[i];
      *p++ = 2.0 * (y[i] - y[i+1]);
      *p++ = z[i+1]*z[i+1] - z[i]*z[i];
      *p++ = 2.0 * (z[i] - z[i+1]);
      f[i] = x[i]*x[i] - x[i+1]*x[i+1];
    }
  /* Solve AX=f */

  if(  LinearEquationsSolving( 5, A, f, X) )
    {
      fprintf(stderr, "System not solvable\n");
      return -1;
    }

  /* Compute sensitivities and offsets */

  k1 = X[1];
  k2 = X[3];
  O[0] = X[0];
  O[1] = X[2]/k1;
  O[2] = X[4]/k2;

  S[0] = sqrt((x[5]-O[0]) * (x[5]-O[0]) +
	      k1*(y[5]-O[1])*(y[5]-O[1]) +
	      k2*(z[5]-O[2])*(z[5]-O[2]));
  S[1] = sqrt(S[0]*S[0]/k1);
  S[2] = sqrt(S[0]*S[0]/k2);
  return 0;
}


/*
  Simple test of calibration code. Compute the calibration values for
  each series and then process each data point.
 */
main()
{
  int i;
  float Sens[3], Offset[3];
  float x, y, z;

  if(cal(X1, Y1, Z1, Sens, Offset) == 0)
    {
      printf(" X       Y       Z\n");
      printf("Sens:   %10.5f %10.5f %10.5f\n", Sens[0], Sens[1], Sens[2]);
      printf("Offset: %10.5f %10.5f %10.5f\n", Offset[0], Offset[1], Offset[2]);

      for(i = 0; i < 6; i++)
	{
	  x = (X1[i] - Offset[0])/Sens[0];
	  y = (Y1[i] - Offset[1])/Sens[1];
	  z = (Z1[i] - Offset[2])/Sens[2];

	  printf("%7.2f %7.2f %7.2f %10f\n",
		 acos(x)*180/M_PI,
		 acos(y)*180/M_PI,
		 acos(z)*180/M_PI, sqrt(x*x + y*y + z*z));
	}
    }
  if( cal(X2, Y2, Z2, Sens, Offset) == 0)
    {
      printf(" X       Y       Z\n");
      printf("Sens:   %10.5f %10.5f %10.5f\n", Sens[0], Sens[1], Sens[2]);
      printf("Offset: %10.5f %10.5f %10.5f\n", Offset[0], Offset[1], Offset[2]);

      for(i = 0; i < 6; i++)
	{
	  x = (X2[i] - Offset[0])/Sens[0];
	  y = (Y2[i] - Offset[1])/Sens[1];
	  z = (Z2[i] - Offset[2])/Sens[2];

	  printf("%7.2f %7.2f %7.2f %10f\n",
		 acos(x)*180/M_PI,
		 acos(y)*180/M_PI,
		 acos(z)*180/M_PI, sqrt(x*x + y*y + z*z));
	}
    }
}








int main(){
	if(initialize_cape()<0){
		printf("Failed to initialize cape, exiting\n");
		return -1;
	}
	
	printf("\nThis program will generate a new gyro calibration file\n");
	printf("keep your beaglebone very still and hit enter to calibrate\n");
	while( getchar() != '\n' );
	
	if(calibrate_mag_routine()<0){
		printf("Failed to complete magnetometer calibration\n");
		return -1;
	}
	
	printf("\nMagnetometer calibration file written\n");
	printf("run test_imu to check performance\n");
		
	cleanup_cape();
	return 0;
}
