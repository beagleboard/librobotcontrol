/*******************************************************************************
* test_cape_extended.c
*
* James Strawson 2016
* This is the extended routine for testing It requires a factory testing jig and is not meant 
* for the 
* end-user.
*******************************************************************************/

#include "../../libraries/roboticscape-usefulincludes.h"
#include "../../libraries/roboticscape.h"

#define INSTRUCTION_LINE 16


void fail_test();
int on_pause_pressed();
int on_pause_released();
int on_mode_pressed();
int on_mode_released();
void clear_screen();
void goto_line(int line);
void clear_line();
void clear_instruction_area();

// thread for blinking 6V regulator line
pthread_t blinking_thread;
void* blinking_function(void* ptr);

int num_passes, num_fails, line;



// main() has a brief setup and starts one background thread.
// then it enters one big while loop for testing multiple capes.
int main(){
	int ret;
	float volt;
	imu_data_t data; // not really used, just necessary to test imu
	// use defaults for now, except also enable magnetometer.
	imu_config_t conf = get_default_imu_config();
	conf.enable_magnetometer=1;

	// counters for how many pass and fail
	num_passes = 0;
	num_fails = 0;

	// initialize_cape, this should never fail unless software is not set up
	// in which case a useful error message should be printed out.
	if(initialize_roboticscape()<0){
		printf("initialize_roboticscape() failed, this is a software issue,\n");
		printf("not a hardware issue. Try running install.sh and restart\n");
		return -1;
	}

	// set up the button handlers once
	set_pause_pressed_func(&on_pause_pressed);
	set_pause_released_func(&on_pause_released);
	set_mode_pressed_func(&on_mode_pressed);
	set_mode_released_func(&on_mode_released);

	// start blinking thread for 6V test
	pthread_create(&blinking_thread, NULL, blinking_function, (void*) NULL);

	// print welcome
	clear_screen();
	goto_line(0);
	printf("Welcome to the Robotics Cape tester!\n\n");
	printf("this will walk you through testing multiple capes and keep\n");
	printf("track of how many pass and fail.\n");
	printf("Closing the program erases the pass/fail count.\n\n");
	printf("Press enter to begin, anything else to quit.\n");
	if(continue_or_quit()<1){
		goto END;
	}

	/***************************************************************************
	* Begin main while loop
	***************************************************************************/
	while(rc_get_state()!=EXITING){
		line = 0; // reset current printing line to top of terminal
		rc_set_led(RED,OFF);
		rc_set_led(GREEN,OFF);
		
		// clear screen and print pass/fail header
		clear_screen();
		goto_line(line);
		printf("passes: %d  fails: %d\n", num_passes, num_fails);
		line+=2;
		
		
		goto_line(INSTRUCTION_LINE-1);
		printf("*******************************************************************\n");
		printf("Place a new cape in the test jig but don't connect anything else.\n");
		printf("Press any key to start test.\n");
		
		// wait to start test
		if(continue_or_quit()<0){
			goto END;
		}

		/***********************************************************************
		* begin list of tests
		***********************************************************************/
		// make sure 12V DC supply is disconnected
CHECK_DC_DISCONNECT:
		volt = get_dc_jack_voltage();
		if(volt>2.0){
			clear_instruction_area();
			printf("Voltage detected on the DC jack input. This is supposed to be\n");
			printf("disconnected for this part of the test.\n");
			printf("Disconnect and hit ENTER to continue\n");
			printf("If the DC supply was disconnected, there may be a problem with resistors\n");
			printf("R1 or R14, press any key other than ENTER to FAIL this test.\n");
			ret = continue_or_quit();
			if(ret==1) goto CHECK_DC_DISCONNECT;
			else if(ret<0) goto END;
			else{
				goto_line(line);
				printf("FAILED	DC JACK VOLTAGE TEST\n");
				line++;
				fail_test();
				continue;
			}
		}

		// test imu
		ret = initialize_imu(&data, conf);
		power_off_imu();
		goto_line(line);
		line++;
		if(ret<0){
			printf("FAILED	MPU9250 IMU\n");
			fail_test();
			continue; // go to beginning to test next cape
		}
		printf("PASSED	MPU9250 IMU\n");

		// test barometer
		ret = initialize_barometer(BMP_OVERSAMPLE_16,BMP_FILTER_OFF);
		power_off_barometer();
		goto_line(line);
		line++;
		if(ret<0){
			printf("FAILED	BMP280 BAROMETER\n");
			fail_test();
			continue; // go to beginning to test next cape
		}
		printf("PASSED	BMP280 BAROMETER\n");

		// test buttons/LEDS
		clear_instruction_area();
		printf("Press the PAUSE button on cape, the RED led should light up.\n");
		printf("Press the MODE button on cape, the GREEN led should light up.\n");
		printf("Press ENTER to indicate the buttons/leds work\n");
		printf("Press any other key to indicate a failure\n");
		ret = continue_or_quit();
		goto_line(line);
		line++;
		if(ret==0){
			printf("FAILED	BUTTON/LED\n");
			fail_test();
			continue;
		}
		else if(ret<0) goto END;
		printf("PASSED	BUTTON/LED\n");

		// DC power jack ADC check
		clear_instruction_area();
		printf("Plug in the 12V power supply, GREEN CHG LED should turn on.\n");
		printf("Press ENTER if the GREEN CHG LED turns on\n");
		printf("Press any other key if not\n");
		ret = continue_or_quit();
		if(ret<0) goto END;
		goto_line(line);
		line++;
		if(ret==0){
			printf("FAILED	CHARGER\n");
			printf("CHG_IC may be bad.\n");
			fail_test();
			continue;
		}
		printf("PASSED	CHARGER\n");
		volt = get_dc_jack_voltage();
		if(volt<11.0 || volt>13.0){
			printf("FAILED	12V DC VOLTAGE\n");
			printf("measuring %0.2fV at DC jack, should be roughly 12V\n", volt);
			printf("Resistors R1 or R14 may be bad, shorted, or missing.\n");
			fail_test();
			continue;
		}
		line++;
		printf("PASSED	12V DC VOLTAGE\n");

		// 5V regulator test
		clear_instruction_area();
		printf("Plug in the 4-pin dongle to PWR socket\n");
		printf("Press ENTER if the dongle LED lights up, any other key if not.\n");
		ret = continue_or_quit();
		goto_line(line);
		line++;
		if(ret==0){
			printf("FAILED	5V REGULATOR\n");
			printf("Diode D3 or IC 5VREG may be bad\n");
			fail_test();
			continue;
		}
		else if(ret<0) goto END;
		printf("PASSED	5V REGULATOR\n");

		// battery ADC check
		clear_instruction_area();
		printf("Plug in 2-cell battery and press any key to continue\n");
		ret = continue_or_quit();
		if(ret<0) goto END;
		volt = get_battery_voltage();
		goto_line(line);
		line++;
		if(volt<5.0 || volt>9.0){
			printf("FAILED	BATTERY VOLTAGE\n");
			printf("measuring %0.2fV at battery, should be between 6 and 8.4\n", volt);
			printf("Resistors R19 or R25 may be bad, shorted, or missing.\n");
			fail_test();
			continue;
		}
		printf("PASSED	BATTERY VOLTAGE\n");

		// battery discharge check
		clear_instruction_area();
		printf("Disconnect the 12V DC power supply.\n");
		printf("If 4-pin dongle LED is still lit, press ENTER\n");
		printf("Otherwise press any other key.\n");
		ret = continue_or_quit();
		if(ret<0) goto END;
		goto_line(line);
		line++;
		if(ret==0){
			printf("FAILED	BATTERY DISCHARGE\n");
			printf("Diode D2, or mosfet Q3 are bad.\n");
			fail_test();
			continue;
		}
		printf("PASSED	BATTERY DISCHARGE\n");

		// 6V regulator check
		clear_instruction_area();
		printf("Plug in the 3-pin dongle into any of the 8 servo channels.\n");
		printf("If the dongle LED is blinking press ENTER.\n");
		printf("Otherwise press any other key.\n");
		ret = continue_or_quit();
		if(ret<0) goto END;
		goto_line(line);
		line++;
		if(ret==0){
			printf("FAILED	6VREG\n");
			printf("AOZ1284PI 6VREG or supporting components are bad.\n");
			fail_test();
			continue;
		}
		printf("PASSED	6VREG CHECK\n");

		// END OF TESTING THIS CAPE, PASSED!!!
		num_passes++;
		printf("COMPLETE TEST PASSED\n");
		goto_line(0);
		printf("passes: %d  fails: %d\n", num_passes, num_fails);
		clear_instruction_area();
		printf("Press any key to continue with next cape\n");
		continue_or_quit();
		if(ret<0) goto END;
		// now loop back to test next cape

	} // end while(rc_get_state()!= EXITING)

		
	// if we got here there was a critical error or user hit ctrl+c
END:
	pthread_join(blinking_thread, NULL);
	disable_servo_power_rail();
	cleanup_roboticscape();
	clear_screen();
	return 0;
}

void clear_screen(){
	printf("\033[2J");
}

void goto_line(int line){
	printf("\033[%d;0H", line);
}

void clear_line(){
	printf("\33[2K\r");
}

void clear_instruction_area(){
	goto_line(INSTRUCTION_LINE);
	clear_line();
	goto_line(INSTRUCTION_LINE+1);
	clear_line();
	goto_line(INSTRUCTION_LINE+2);
	clear_line();
	goto_line(INSTRUCTION_LINE+3);
	clear_line();
	goto_line(INSTRUCTION_LINE+4);
	clear_line();
	goto_line(INSTRUCTION_LINE+5);
	clear_line();
	goto_line(INSTRUCTION_LINE+6);
	clear_line();
	goto_line(INSTRUCTION_LINE);
}

// called once a test fails to indicate to the user 
void fail_test(){
	num_fails++;
	printf("TEST FAILED\n");
	printf("press any key to continue with next cape\n");
	clear_instruction_area();
	printf("TEST FAILED\n");
	printf("press any key to continue with next cape\n");
	continue_or_quit();
}

// pause button pressed interrupt function
int on_pause_pressed(){
	rc_set_led(RED, ON);
	fflush(stdout);
	return 0;
}

// pause button released interrupt function
int on_pause_released(){
	rc_set_led(RED, OFF);
	fflush(stdout);
	return 0;
}

// mode button pressed interrupt function
int on_mode_pressed(){
	rc_set_led(GREEN, ON);
	fflush(stdout);
	return 0;
}

// mode button released interrupt function
int on_mode_released(){
	rc_set_led(GREEN,OFF);
	fflush(stdout);
	return 0;
}

void* blinking_function(void* ptr){
	int toggle = 1;

	while(rc_get_state()!=EXITING){
		if(toggle){
			enable_servo_power_rail();
			toggle = 0;
		}
		else{
			disable_servo_power_rail();
			toggle = 1;
		}
		usleep(500000);
	}

	return NULL;
}
