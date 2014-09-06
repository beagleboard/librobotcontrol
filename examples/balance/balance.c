// BeagleMIP Balance 
// James Strawson 2014

#include <robotics_cape.h>

#define SAMPLE_RATE_HZ 200	// main filter and control loop
#define DT 0.005       		
#define WHEEL_RADIUS 0.035  // meters
#define TRACK_WIDTH 0.1 	// meters, width between contact patches

#define LEAN_THRESHOLD 0.6  // radians lean before killing motors
#define THETA_REF_MAX 0.5	// Maximum reference theta set point for inner loop
#define START_THRESHOLD 0.2 // how close to vertical before it will start balancing

// complementary high and low pass filter constants, plus integrator trim
#define THETA_MIX_TC  2   // t_seconds time constant on filter
const float HP_CONST = THETA_MIX_TC/(THETA_MIX_TC + DT);
const float LP_CONST = DT/(THETA_MIX_TC + DT);

// Encoder Variables
long int encoderCountsL, encoderCountsR;

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
float kInner = 1.5;	 // inner loop feedback gain
float kOuter = 2.2;	 // outer loop feedback gain

// Remote control things for driving around
#define MAXTURNRATE 4
#define MAXDRIVERATE 16
float turnRate, driveRate; //radians per second

// Theta trim to correct for imbalance
float kTrim = -0.2;  // outer loop integrator constant
float thetaTrim = 0;

// Battery Monitoring
#define VNOMINAL 7.4 //tune controller assuming 7.4v battery
float vBatt=VNOMINAL; //battery voltage for scaling motor inputs.

// Mavlink
#define DEFAULT_MAV_ADDRESS "192.168.7.1"
int sock;
struct sockaddr_in gcAddr;

// IMU
mpudata_t mpu; //struct to read IMU data into

// discrete-time balance controller
int control_func(){
	if (mpu9150_read(&mpu) == 0) {
		theta = -mpu.fusedEuler[VEC3_Y]; // positive theta tips forward in MIP model
		 //convert encoders to radians, 352 ticks per revolution
		encoderCountsR = get_encoder_pos(1);
		encoderCountsL = -get_encoder_pos(2); 
		phi[1] = phi[0];
		phi[0] = ((encoderCountsL+encoderCountsR)*PI/352)+theta; 
		
		//turning estimation
		int encoder_dif;
		encoder_dif = encoderCountsL - encoderCountsR;
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
			if(is_new_dsm2_data()){	
				// Read normalized (+-1) inputs from RC radio right stick
				float turnInput = -get_dsm2_ch_normalized(2);	// pos turn right
				float driveInput = get_dsm2_ch_normalized(3);	// pos go forward
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
			//keep everything zero'd
			set_encoder_pos(1,0);
			set_encoder_pos(2,0);
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
	}
	return 0;
}

// 10hz loop checking battery, uprighting, and printing data
void* slow_loop_func(void* ptr){
	int i;
	do{
		switch (get_state()){
		case RUNNING:	
			vBatt = getBattVoltage();
			break;
			
		case PAUSED:
			// check if the user has picked MIP upright before starting again
			if(fabs(theta)<START_THRESHOLD){
				setGRN(HIGH); //tell user it's upright enough to start
				// check for half a second to see if it stays up
				for(i=0;i<5;i++){
					usleep(100000);
					if(fabs(theta)>START_THRESHOLD){
						break; // not held upright long enough
					}
				}
				// upright, start balancing
				if(fabs(theta)<START_THRESHOLD){
					setRED(LOW);
					set_state(RUNNING);
				}
				else{
					setGRN(LOW);
				}
			}
			break;
			
		default:
			break;
		}

		printf("\r");
		printf("theta: %0.2f", theta);
		printf(" u: %0.2f", u[0]);
		printf(" phi: %0.2f", phi[0]);
		//printf(" phiRef: %0.2f", phiRef);
		//printf(" gamma: %0.2f", Gamma[0]);
		//printf(" gammaRef: %0.2f", gammaRef);
		//printf(" thetaRef: %0.2f", thetaRef[0]);
		//printf(" trim: %0.2f", thetaTrim);
		//printf(" driveRate: %0.2f", driveRate);
		//printf(" turnRate: %0.2f", turnRate);
		//printf("   ");
		
		fflush(stdout);
		usleep(100000); //run at roughly 10 hz
	}while(get_state() != EXITING);
	return NULL;
}


// If the user holds the pause button for a second, exit cleanly
int on_pause_press(){
	int i=0;
	do{
		usleep(100000);
		if(get_pause_button_state() == LOW){
			return 0; //user let go before time-out
		}
		i++;
	}while(i<10);
	//user held the button down long enough, exit cleanly
	set_state(EXITING);
	return 0;
}

// listen for RC control for driving around
void* dsm2_listener(void* ptr){
	float turnInput, driveInput;
	
	while(get_state()!=EXITING){
		if(is_new_dsm2_data()){	
			// Read normalized (+-1) inputs from RC radio right stick
			turnInput = -get_dsm2_ch_normalized(2);	// pos turn right
			driveInput = get_dsm2_ch_normalized(3);	// pos go forward
			//Check for bad data with 0.1 fudge factor
			if(fabs(driveInput)>1.1 || fabs(turnInput)>1.1){
				driveRate = 0;
				turnRate = 0;
			}
			else{ //scale rates appropriately 
				driveRate = driveInput*MAXDRIVERATE;
				turnRate = turnInput*MAXTURNRATE;
			}
		}
		usleep(10000);
	}
	return 0;
}
	
	
// listen for RC mavlink packets for driving around
void* mavlink_listener(void* ptr){
	ssize_t recsize;
	socklen_t fromlen;
	uint8_t buf[MAV_BUF_LEN];
	int i;
	
	int16_t chan3_scaled, chan4_scaled;
	
	while(get_state() != EXITING){
		recsize = recvfrom(sock, (void *)buf, MAV_BUF_LEN, 0, (struct sockaddr *)&gcAddr, &fromlen);
		if (recsize > 0){
			// Something received - print out all bytes and parse packet
			mavlink_message_t msg;
			mavlink_status_t status;
			for (i = 0; i < recsize; ++i){
				if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)){
					// Packet received, do something
					printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
					// if the packet is scaled RC channels, drive around!!
					if(msg.msgid == MAVLINK_MSG_ID_RC_CHANNELS_SCALED){
						chan3_scaled =  mavlink_msg_rc_channels_scaled_get_chan3_scaled(&msg);
						chan4_scaled =  mavlink_msg_rc_channels_scaled_get_chan4_scaled(&msg);
						driveRate = (float)chan3_scaled*MAXDRIVERATE/10000.0;
						turnRate = (float)chan4_scaled*MAXTURNRATE/10000.0;
					}
				}
			}
		}
		else{
			printf("%d ",recsize);
			printf("error in recvfrom\n");
		}
		usleep(10000);
	}
	return NULL;
}

// send mavlink heartbeat and IMU attitude packets
void* mavlink_sender(void* ptr){
	uint8_t buf[MAV_BUF_LEN];
	mavlink_message_t msg;
	uint16_t len;
	while(get_state() != EXITING){
		
		// send heartbeat
		memset(buf, 0, MAV_BUF_LEN);
		mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
		//send attitude
		memset(buf, 0, MAV_BUF_LEN);
		mavlink_msg_attitude_pack(1, 200, &msg, microsSinceEpoch(), 
											mpu.fusedEuler[VEC3_X], 
											mpu.fusedEuler[VEC3_Y],
											mpu.fusedEuler[VEC3_Z], 
											0, 0, 0); //set gyro rates to 0 for simplicity
		len = mavlink_msg_to_send_buffer(buf, &msg);
		sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
		usleep(100000); // 10 hz
	}
	return NULL;
}

int main(int argc, char* argv[]){
	initialize_cape();
	setRED(1);
	setGRN(0);
	set_state(PAUSED);
	signed char orientation[9] = ORIENTATION_UPRIGHT; //upright for MIP
	initialize_imu(SAMPLE_RATE_HZ, orientation);
	
	// time the start button to see if a user wants to exit
	set_pause_pressed_func(&on_pause_press);
	
	// start slow state management thread
	pthread_t  slow_thread;
	pthread_create(&slow_thread, NULL, slow_loop_func, (void*) NULL);
	
	// start listening for RC control from dsm2 radio
	initialize_dsm2();
	pthread_t  dsm2_thread;
	pthread_create(&dsm2_thread, NULL, dsm2_listener, (void*) NULL);
	
	// see if the user gave an IP address as argument
	char target_ip[100];
	if (argc == 2){
		strcpy(target_ip, argv[1]);
    }
	else{ //otherwise use default address 
		strcpy(target_ip, DEFAULT_MAV_ADDRESS);
	}
	// open a udp port for mavlink
	// sock and gcAddr are global variables needed to send and receive
	gcAddr = initialize_mavlink_udp(target_ip, &sock);
	
	// start a thread listening for incoming packets
	pthread_t  mav_listen_thread;
	pthread_create(&mav_listen_thread, NULL, mavlink_listener, (void*) NULL);
	printf("Listening for Packets\n");
	
	// Start thread sending heartbeat and IMU attitude packets
	pthread_t  mav_send_thread;
	pthread_create(&mav_send_thread, NULL, mavlink_sender, (void*) NULL);
	printf("Sending Heartbeat Packets\n");
	
	
	// Finally start the real-time interrupt driven control thread
	set_imu_interrupt_func(&control_func);
	printf("\nHold your MIP upright to begin balancing\n");
	
	//chill until something exits the program
	while(get_state()!=EXITING){
		usleep(100000);
	}
	
	close(sock); //close network socket
	cleanup_cape();
	return 0;
}