// BeagleMIP Balance - James Strawson 2013

#include <robotics_cape.h>
#include "c_i2c.h"
#include "MPU6050.h"

#define DT 0.005       		// 5ms loop (200hz)
#define WHEEL_RADIUS 0.035  // meters
#define TRACK_WIDTH 0.1 	// meters, width between contact patches
#define PI 3.14159265358

#define LEAN_THRESHOLD 0.6  // radians lean before killing motors
#define THETA_REF_MAX 0.5	// Maximum reference theta set point for inner loop
#define START_THRESHOLD 0.2 // how close to vertical before it will start balancing

// complementary high and low pass filter constants, plus integrator trim
#define THETA_MIX_TC  2   // t_seconds timeconstant on filter
const float HP_CONST = THETA_MIX_TC/(THETA_MIX_TC + DT);
const float LP_CONST = DT/(THETA_MIX_TC + DT);


// i2c declarations
static i2c_t MPU6050;
static i2c_t* pi2c = &MPU6050;

// Estimator variables
float xAccel, zAccel, yGyro, yGyroOffset, accLP, gyroHP, theta;

// Encoder Variables
long int encoderCountsL, encoderCountsR;
long int encoderOffsetL, encoderOffsetR;

// Controller & State Variables
float prescaler = 0.7;  // SLC prescaler to correct inner loop steady state error
float theta, phi[2], eTheta[3], ePhi[2], u[3], thetaRef[2], phiRef;

//Turn controller
float kpTurn = 1.0;
float kdTurn = 0.1;
float dutyLeft, dutyRight;
float gammaRef, torqueSplit,Gamma[2], eGamma[2];

// Balancing Control constants
// Discrete time transfer function constants
float numD1[] = {-6.0977, 11.6581, -5.5721};
float denD1[] = {1.0000,   -1.6663,    0.6663};
float numD2[] = {0.0987,   -0.0985};
float denD2[] = {1.0000,   -0.9719};
float kInner = 1.8;	 // inner loop feedback gain
float kOuter = 2.2;	 // outer loop feedback gain

// Remote control things for driving around
#define MAXTURNRATE 3	
#define MAXDRIVERATE 10	
float turnRate, driveRate; //radians per second

// Theta trim to correct for imbalance
float kTrim = -0.2;  // outer loop integrator constant
float thetaTrim = 0;

// Other
#define VNOMINAL 7.4 //tune controller assuming 7.4v battery
float vBatt=VNOMINAL; //battery voltage for scaling motor inputs.

// start I2C communication with MPU-9150/6050
void i2cStart(){
	//printf("MPU6050 test\n\n");
	pi2c->bus = 1;
	i2c_init(pi2c, pi2c->bus, 0x68);
	openConnection(pi2c);
	bool mpu6050TestResult = MPU_testConnection(pi2c, pi2c->buffer);
	if(mpu6050TestResult) {
		printf("MPU6050 test passed \n");
	} else {
		printf("MPU6050 test failed \n");
	}
}

//returns theta after a fresh configuration and warmup
float initializeEstimator(){
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	int i = 0;
	float xAccelCount, zAccelCount, yGyroCount;
	uint8_t buffer[14];
	
	MPU_setSleepEnabled(false, pi2c);  			  	// Wake up from sleep
	MPU_setClockSource(MPU6050_CLOCK_PLL_XGYRO, pi2c);		// setup MPU6050	
	MPU_setDLPFMode(0, pi2c); 					// as little filtering as possible	
	MPU_setFullScaleGyroRange(MPU6050_GYRO_FS_1000, pi2c); 	// GYRO_FS_1000  +-1000 deg/s range	
	MPU_setFullScaleAccelRange(MPU6050_ACCEL_FS_2, pi2c);	// MPU6050_ACCEL_FS_2    +- 2g range

	usleep(10000); // let the gyro settle

	// warm up loop, sample data 100 times
	for (i = 0; i < 100; i++){ 
		MPU_getMotion6(&ax, &ay, &az, &gx, &gy, &gz, pi2c, buffer); 
		xAccelCount += (float)ax; 
		zAccelCount += (float)az;
		yGyroCount += (float)gy;
		usleep(5000);
		//printf("gy: %f  ax: %f  az: %f\n", (float)gy, (float)ax, (float)ay);            
	}
		
	yGyroOffset = yGyroCount/100;		// offset to correct for steady-state gyro error
    accLP = -atan2(zAccelCount, -xAccelCount); 	// initialize accLP at current theta
	theta=accLP;
	printf("yGyroOffset = %f\n", yGyroOffset);
	printf("Theta = %f\n", theta);
	return accLP;
}


// Complementary Filter
float Complementary_Filter(){
	int16_t ax, ay, az;
	int16_t gx, gy, gz;

	MPU_getMotion6(&ax, &ay, &az, &gx, &gy, &gz, pi2c, pi2c->buffer); 
      
	// Apply preScale for +-2g range
	xAccel = (float)ax*2/32768;  
	zAccel = (float)az*2/32768; 

	//subtract the steady-state gyro offset error
	//then multiply by the gyro prescaler
	yGyro = (gy-yGyroOffset)*1000*2*PI/(360*32768);            

	//first order filters
	accLP = accLP + LP_CONST * (-atan2(zAccel,-xAccel) - accLP);
	gyroHP = HP_CONST*(gyroHP + .005*yGyro);

	// diagnostic print to console
	//printf("gyroHP: %f accelLP: %f theta: %f Phi: %f\n", gyroHP, accLP, gyroHP+accLP, phi); 
	return (gyroHP+accLP)+thetaTrim;
}

//If the user holds select for 2 seconds, the program exits cleanly
int on_start_press(){
	sleep(2);
	if(get_start_button() == HIGH){
		set_state(EXITING);
	}
	return 0;
}

///////////////////////////////////////////////////////
// 10hz Loop checking battery, buttons, and tipover ///
///////////////////////////////////////////////////////
void* slow_loop_func(void* ptr){
	do{
		switch (get_state()){
		case RUNNING:	
			vBatt = getBattVoltage();
			break;
			
		case PAUSED:
			if(fabs(theta)<START_THRESHOLD){
				setGRN(HIGH); //tell user it's upright enough to start
				sleep(1); //wait a second before starting balancing
				if(fabs(theta)<START_THRESHOLD){
					set_state(RUNNING);
					setRED(LOW);
				}
				else{
					setGRN(LOW);
				}
			}
			break;
			
		default:
			break;
		}
		printf("\rtheta: %0.2f", theta);
		printf(" u: %0.2f", u[0]);
		printf(" phi: %0.2f", phi[0]);
		printf(" phiRef: %0.2f", phiRef);
		printf(" gamma: %0.2f", Gamma[0]);
		printf(" gammaRef: %0.2f", gammaRef);
		printf(" thetaRef: %0.2f", thetaRef[0]);
		printf(" trim: %0.2f", thetaTrim);
		printf(" driveRate: %0.2f", driveRate);
		printf(" turnRate: %0.2f", turnRate);
		printf("   ");
		
		fflush(stdout);
		usleep(100000); //check buttons at roughly 10 hz,not very accurate)
	}while(get_state() != EXITING);
	return NULL;
}

////////////////////////////////
/// 200hz discrete controller //
////////////////////////////////
void* control_loop_func(void* ptr){
	timespec t1, t2, t2minust1, sleepRequest, deltaT;
	deltaT.tv_sec = 0;		// create time struct for 5ms (200hz)
	deltaT.tv_nsec = 5000000;

 	while(get_state() != EXITING){
		clock_gettime(CLOCK_MONOTONIC, &t1);  //record the time at the beginning.
		
		// Filter angle theta
		theta = Complementary_Filter();
		
		 //convert encoders to radians, 352 ticks per revolution
		encoderCountsR = get_encoder(1);
		encoderCountsL = -get_encoder(2); 
		phi[1] = phi[0];
		phi[0] = ((encoderCountsL-encoderOffsetL+encoderCountsR-encoderOffsetR)*PI/352)+theta; 
		
		//turning estimation
		int encoder_dif;
		encoder_dif = (encoderCountsL-encoderOffsetL)-(encoderCountsR-encoderOffsetR);
		Gamma[1] = Gamma[0];
		Gamma[0] = WHEEL_RADIUS*2*(encoder_dif)*PI/(352*TRACK_WIDTH);

		
		switch (get_state()){
		case RUNNING:
			// check for a tipover
			if(fabs(theta)>LEAN_THRESHOLD){
				set_state(PAUSED);
				kill_pwm();
				setRED(HIGH);
				setGRN(LOW);
				break;
			}
			//check if wheels are free spinning
			if(fabs(phi[0]-phi[1])>.3){
				set_state(PAUSED);
				kill_pwm();
				setRED(HIGH);
				setGRN(LOW);
				break;
			}
			//check for new RC data
			if(get_rc_new_flag()){	
				// Read normalized (+-1) inputs from RC radio right stick
				float turnInput = -get_rc_channel(2);	// pos turn right
				float driveInput = get_rc_channel(3);	// pos go forward
				//Check for bad data with 0.1 fudge
				if(fabs(driveInput)>1.1 || fabs(turnInput)>1.1){
					driveRate = 0;
					turnRate = 0;
				}
				else{ //scale rates appropriately 
					driveRate = driveInput*MAXDRIVERATE;
					turnRate = turnInput*MAXTURNRATE;
				}
			}
			//move the controller set points based on user input
			phiRef = phiRef+driveRate*DT;
			gammaRef = gammaRef+turnRate*DT;
  
			// evaluate outer loop controller D2z
			ePhi[1]=ePhi[0];
			ePhi[0] = phiRef-phi[0];
			thetaRef[1]=thetaRef[0];
			thetaRef[0] = kOuter*(numD2[0]*ePhi[0] + numD2[1]*ePhi[1]) - denD2[1]*thetaRef[1];
			//check saturation of outer loop
			if(thetaRef[0]>THETA_REF_MAX) thetaRef[0]=THETA_REF_MAX;
			else if(thetaRef[0]<-THETA_REF_MAX) thetaRef[0]=-THETA_REF_MAX;
					
			//evaluate inner loop controller D1z
			eTheta[2]=eTheta[1]; eTheta[1]=eTheta[0];
			eTheta[0] = (prescaler * thetaRef[0]) - theta;
			u[2]=u[1]; u[1]=u[0];
			u[0] = kInner*(numD1[0]*eTheta[0]+numD1[1]*eTheta[1] + numD1[2]*eTheta[2]) - denD1[1]*u[1] - denD1[2]*u[2]; 
			//check saturation of inner loop
			if(u[0]>1)	u[0]=1;	
			else if(u[0]<-1) u[0]=-1;
			
			
			//integrate the reference theta to correct for imbalance or sensor error
			if(thetaRef == 0)thetaTrim += kTrim * thetaRef[0]*DT;
			
			//steering controller
			eGamma[1] = eGamma[0];
			eGamma[0] = gammaRef - Gamma[0];
			torqueSplit = kpTurn*(eGamma[0]+kdTurn*(eGamma[0]-eGamma[1]));
			dutyLeft = (u[0]+torqueSplit)*VNOMINAL/vBatt;
			dutyRight = (u[0]-torqueSplit)*VNOMINAL/vBatt;			
			
			// Final output of controller
			set_motor(1,-dutyRight); //motor is flipped on chassis
			set_motor(3,dutyLeft); 
			
			break;
			
		case PAUSED:
			encoderOffsetL = encoderCountsL;
			encoderOffsetR = encoderCountsR;
			ePhi[1]=0; ePhi[0]=0;
			thetaRef[1]=0; thetaRef[0]=0;
			eTheta[2]=0; eTheta[1]=0; eTheta[0]=0;
			u[2]=0; u[1]=0; u[0]=0;
			phi[0]=0; phi[1]=0;
			eGamma[0]=0; eGamma[1]=0;
			gammaRef=0; phiRef=0;
			break;
			
		default:
			break;
		}
		
		//Sleep for the necessary time to maintain 200hz
		clock_gettime(CLOCK_MONOTONIC, &t2);
		t2minust1 = diff(t1, t2);
		sleepRequest = diff(t2minust1, deltaT);
		nanosleep(&sleepRequest, NULL);
	}
	return NULL;
}

//////////////////////////////////////////////////////////////////
/// Main function used for initialization and thread management ///
//////////////////////////////////////////////////////////////////
int main(){
	initialize_cape();
	initialize_spektrum();
	set_start_pressed_func(&on_start_press); //hold select for 2 seconds to close program
	i2cStart();
	initializeEstimator();
	setRED(1);
	setGRN(0);
	set_state(PAUSED);
	
	//start threads
	pthread_t control_thread, slow_thread;
	struct sched_param params;
	// We'll set the priority to the maximum for control_thread
	pthread_create(&control_thread, NULL, control_loop_func, (void*) NULL);
	params.sched_priority = sched_get_priority_max(SCHED_FIFO);
	pthread_setschedparam(control_thread, SCHED_FIFO, &params);
	pthread_create(&slow_thread, NULL, slow_loop_func, (void*) NULL);
	printf("\nHold your MIP upright to begin balancing\n");
	
	while(get_state()!=EXITING){
		sleep(1);
	}

	//nothing get executed here until runTrue == 0 which terminates the threads
	closeConnection(pi2c);
	cleanup_cape();
	return 0;
}