// Fly
// James Strawson - 2013

#include <robotics_cape.h>
#define CONTROL_HZ 200		// Run the main control loop at this rate
#define DT .005   			// timestep seconds MUST MATCH CONTROL_HZ
#define	SATE_LEN 32			// number of timesteps to retain data
#define TIP_THRESHOLD 0.6	// Kill propellers if it goes into a roll
//#define THETA_REF_MAX 0.4	// Maximum reference theta set point for inner loop
#define SYSTEM_STATES 4		// Altitude, Yaw, Pitch, Roll
#define STATE_HISTORY 2	
#define USER_YAW_RATE 4		// Max rad/s the unit will yaw for the user.
#define MAX_COMPONENT 0.3	// Max duty from each roll/pitch/yaw controller
#define MAX_SETPOINT  0.4	// Max range for setpoint
#define MAX_THROTTLE  0.6	// maximum value for throttle component before mixing
#define INT_CUTOFF_TH 0.1	// prevent integrators from running unless flying

int on_start_press();
void* control_loop(void* ptr);
void* io_loop(void* ptr);
mpudata_t mpu;

//Global state and control Variables. [altitude, roll, pitch, yaw]
float x[SYSTEM_STATES][STATE_HISTORY];
float state_error[SYSTEM_STATES][STATE_HISTORY];
float set_point[4], integrator[4], derivative[4], u[4], esc[4];
//control gains P	I	D 
float K[4][3]={{.0,  .0,  .0},	// throttle
			   {.08,  .2,  .2}, // roll
			   {.12,  .3,  .2}, // pitch 
			   {.2,   1,  .4}};	// yaw
			    
float imu_offset[3]; //stead state error in raw angles set when armed

int main(){
	//Initialize
	initialize_cape();
	initialize_spektrum();
	set_start_pressed_func(&on_start_press); //hold start for 2 seconds to close program
	if(initialize_imu(CONTROL_HZ)){
		return -1;
	}
	setRED(1);
	setGRN(0);
	set_state(PAUSED);
	printf("hello and welcome to the BeagleQuad fly program.\n");

	/// Begin the threads!
	pthread_t control_thread, io_thread;
	struct sched_param params;
	pthread_create(&control_thread, NULL, control_loop, (void*) NULL);
	params.sched_priority = sched_get_priority_max(SCHED_FIFO);
	pthread_setschedparam(control_thread, SCHED_FIFO, &params);
	pthread_create(&io_thread, NULL, io_loop, (void*) NULL);

	//chill
	while(get_state()!=EXITING){
		usleep(100000);
	}
	//cleanup
	sleep(1);
	cleanup_cape();
	return 0;
}

//If the user holds start for 2 seconds, the program exits cleanly
int on_start_press(){
	sleep(2);
	if(get_start_button() == HIGH){
		set_state(EXITING);
	}
	return 0;
}

// Flight Control Loop //
void* control_loop(void* ptr){
	timespec beginTime, endTime, executionTime, sleepRequest, deltaT;
	deltaT.tv_sec = 0;	// create time struct for control loop
	deltaT.tv_nsec = (int)(1000000000/CONTROL_HZ);
	memset(&mpu, 0, sizeof(mpudata_t));
	int i;
	do{
		clock_gettime(CLOCK_MONOTONIC, &beginTime);  //record the time at the beginning.
		mpu9150_read(&mpu);

		//update system state estimate
		for(i=0;i<SYSTEM_STATES;i++){
			x[i][1]=x[i][0];
			state_error[i][1]=state_error[i][0];
		}
		//x[0][0] = get_Sonar_Range;
		x[1][0] = mpu.fusedEuler[VEC3_X] - imu_offset[0];
		x[2][0] = -mpu.fusedEuler[VEC3_Y] + imu_offset[1];
		x[3][0] = -mpu.fusedEuler[VEC3_Z] + imu_offset[2];
		
		switch (get_state()){
		case RUNNING:	
		
			//check for kill switch
			if(get_rc_channel(6)==1){
				set_state(PAUSED);
				set_all_esc(0);
				setRED(HIGH);
				setGRN(LOW);
				printf("Kill Switch Hit\n");
			}
			
			//update controller set points
			set_point[1]=get_rc_channel(2)*MAX_SETPOINT;
			set_point[2]=-get_rc_channel(3)*MAX_SETPOINT;
			set_point[3]=set_point[3] + USER_YAW_RATE*DT*get_rc_channel(4);
			
			//PID Up in Here
			for(i=0;i<SYSTEM_STATES;i++){
				state_error[i][0]=set_point[i]-x[i][0];
				if(u[0] > INT_CUTOFF_TH){
					integrator[i]=DT*state_error[i][0] + integrator[i];}
				derivative[i]=(state_error[i][0]-state_error[i][1])/DT;
				u[i]=K[i][0]*(state_error[i][0]+K[i][1]*integrator[i]+K[i][2]*derivative[i]);
				if(u[i]>MAX_COMPONENT) u[i] = MAX_COMPONENT;
				else if(u[i]<-MAX_COMPONENT) u[i] = -MAX_COMPONENT;
			}
			
			//direct throttle for now
			u[0] = ((get_rc_channel(1)+1)/2)*MAX_THROTTLE;
			
			// mixing
			esc[0]=u[0]-u[1]-u[2]-u[3];
			esc[1]=u[0]+u[1]-u[2]+u[3];
			esc[2]=u[0]+u[1]+u[2]-u[3];
			esc[3]=u[0]-u[1]+u[2]+u[3];
			for(i=0;i<4;i++){
				set_esc(i+1,esc[i]);
			}		
			break;
			
		case PAUSED:
			break;
			
		default:
			break;
		}
		//Sleep for the necessary time to maintain loop frequency
		clock_gettime(CLOCK_MONOTONIC, &endTime);
		executionTime = diff(beginTime, endTime);
		sleepRequest = diff(executionTime, deltaT);
		nanosleep(&sleepRequest, NULL);
	}while(get_state() != EXITING);
	return NULL;
}

void* io_loop(void* ptr){
	int i;
	do{
		switch (get_state()){
		case RUNNING:	
			// detect a tip-over pitch and roll
			if(abs(x[1][0])>TIP_THRESHOLD || abs(x[2][0])>TIP_THRESHOLD){
				set_state(PAUSED);
				set_all_esc(0);
				setRED(HIGH);
				setGRN(LOW);
			}
			printf("\rx: "); //print state
			for(i=0; i<SYSTEM_STATES; i++){
				printf("%0.2f ", x[i][0]);
			}
			printf(" set: "); //print controller setpoint
			for(i=0; i<4; i++){
				printf("%0.2f ", set_point[i]);
			}
			// printf(" RC: "); //print RC Radio Inputs
			// for(i=0; i<4; i++){
				// printf("%0.2f ", get_rc_channel(i));
			// }
			printf(" u: ");//print control outputs u
			for(i=0; i<4; i++){
				printf("%0.2f ", u[i]);
			}
			fflush(stdout);		
			break;
			
		case PAUSED:
			set_all_esc(0);
			printf("\nTurn on your transmitter kill switch UP\n");
			printf("Move throttle UP then DOWN to arm\n");
			while(get_rc_new_flag()==0){ //wait for radio connection
				usleep(100000);
				if(get_state()==EXITING)
					break;}
			while(get_rc_channel(6)!=-1){ //wait for kill switch up
				usleep(100000);
				if(get_state()==EXITING)
					return 0;}
			while(get_rc_channel(1)!=-1){ //wait for throttle down
				usleep(100000);
				if(get_state()==EXITING)
					break;}
			while(get_rc_channel(1)!=1){ //wait for throttle up
				usleep(100000);
				if(get_state()==EXITING)
					break;}
			setGRN(HIGH);
			while(get_rc_channel(1)!=-1){ //wait for throttle down
				usleep(100000);
				if(get_state()==EXITING)
				break;}
			imu_offset[0] = mpu.fusedEuler[VEC3_X]; 
			imu_offset[1] = mpu.fusedEuler[VEC3_Y]; 
			imu_offset[2] = mpu.fusedEuler[VEC3_Z]; 
			set_point[3] = 0;
			printf("ARMED!!\n\n");
			set_state(RUNNING);
			setRED(LOW);
			break;
			
		default:
			break;
		}
		usleep(100000); //check buttons at roughly 10 hz,not very accurate)
	}while(get_state() != EXITING);
	return NULL;
}