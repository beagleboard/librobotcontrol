/*******************************************************************************
* test_cape.c
*
* James Strawson 2016
* This is the routine used by the factory for testing capes as they come off the
* assembly line. It requires a factory testing jig and is not meant for the 
* end-user.
*******************************************************************************/

#include "../../libraries/roboticscape-usefulincludes.h"
#include "../../libraries/roboticscape.h"

fail_test();
pass_test();

int passes, fails;

// main() has a brief setup and starts one background thread.
// then it enters one big while loop for testing multiple capes.
int main(){
	int ret, i, chg_pass;

	// use defaults for now, except also enable magnetometer.
	imu_data_t data; 
	imu_config_t conf = get_default_imu_config();
	conf.enable_magnetometer=1;

	// initialize_cape, this should never fail unless software is not set up
	// in which case a useful error message should be printed out.
	if(initialize_cape()<0){
		printf("initialize_cape() failed, this is a software issue,\n");
		printf("not a hardware issue. Try running install.sh and restart\n");
		return -1;
	}

	printf("Welcome to the Simplified Robotics Cape tester!\n\n");
	

	set_led(RED,OFF);
	set_led(GREEN,OFF);

	/***************************************************************************
	* Begin main while loop
	***************************************************************************/
	while(get_state()!=EXITING){

		// make sure 12V DC supply is disconnected
		printf("Waiting for new cape and DC power supply\n");
		while(get_dc_jack_voltage()<10.0){
			if(get_state()==EXITING) goto END;
			usleep(10000);
		}
		set_led(RED,OFF);
		set_led(GREEN,OFF);
		printf("DC power connected\n");


		// test imu
		ret = initialize_imu(&data, conf);
		power_off_imu();
		if(ret<0){
			printf("FAILED	MPU9250 IMU\n");
			fail_test();
			continue; // go to beginning to test next cape
		}
		printf("PASSED	MPU9250 IMU\n");


		// test barometer
		ret = initialize_barometer(BMP_OVERSAMPLE_16,BMP_FILTER_OFF);
		power_off_barometer();
		if(ret<0){
			printf("FAILED	BMP280 BAROMETER\n");
			fail_test();
			continue; // go to beginning to test next cape
		}
		printf("PASSED	BMP280 BAROMETER\n");


		// check charger by checking every half second for 2.5 seconds 
		// for the right voltage on the batt line
		chg_pass = 0;
		for(i=0; i<5; i++){
			float volt = get_dc_jack_voltage();
			if(volt<8.7 && volt>6.0){
				chg_pass = 1;
				break;
			} 
			if(get_state()==EXITING) goto END;
			usleep(500000);
		}
		
		if(!chg_pass){
			printf("FAILED	CHARGER TEST\n");
			fail_test();
			continue;
		}

		
		// END OF TESTING THIS CAPE, PASSED!!!
		pass_test();
	} // end while(get_state()!= EXITING)

		
	// if we got here there was a critical error or user hit ctrl+c
END:
	
	cleanup_cape();
	return 0;
}



void fail_test(){
	set_led(RED,ON);
	set_led(GREEN,OFF);
	num_fails++;
	printf("COMPLETE TEST FAILED\n");
	printf("passes: %d  fails: %d\n\n", num_passes, num_fails);
	return;
}

void pass_test(){
	set_led(RED,OFF);
	set_led(GREEN,ON);
	num_passes++;
	printf("COMPLETE TEST PASSED\n");
	printf("passes: %d  fails: %d\n\n", num_passes, num_fails);
	return;
}


