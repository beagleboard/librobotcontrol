/**
 * @file rc_battery_monitor.c
 *
 * @author     James Strawson
 * @date       4/5/2018
 */

#include <stdio.h>
#include <stdlib.h>
#include <error.h>
#include <unistd.h>
#include <signal.h>

#include <rc/adc.h>
#include <rc/led.h>
#include <rc/math/filter.h>
#include <rc/math/ring_buffer.h>
#include <rc/time.h>
#include <rc/model.h>

#define BATTPIDFILE	"/run/rc_battery_monitor.pid"
#define START_BLINK_US	100000	// 0.1 second

// Critical Max voltages of packs used to detect number of cells in pack
#define CELL_MAX	4.25 // set 1er than actual to detect num cells
#define CELL_FULL	3.90 // minimum V to consider battery full
#define CELL_75		3.80 // minimum V to consider battery 75%
#define CELL_50		3.60 // minimum V to consider battery 50%
#define CELL_25		3.25 // minimum V to consider battery 25%
#define CELL_DIS	2.70 // Threshold for detecting disconnected battery
#define V_CHG_DETECT	4.15 // above this assume finished charging

// filter
#define LOOP_HZ		3
#define LOOP_SLEEP_US	1000000/LOOP_HZ
#define FITLER_SAMPLES	6	// average over 6 samples, 3 seconds
#define STD_DEV_TOL	0.04	// above 0.1 definitely charging

// functions
static void illuminate_leds(int i);
static int kill_existing_instance();
static void shutdown_signal_handler(int signo);
static int startup_blink();

static int running;


// main() takes only one optional argument: -k (kill)
int main(int argc, char *argv[])
{
	FILE* fd;
	double new_v_pack, new_v_jack;
	double v_pack;	// 2S pack voltage on JST XH 2S balance connector
	double v_jack;	// could be dc power supply or another battery
	double cell_voltage;	// cell voltage from either 2S or external pack
	int toggle = 0;
	int printing = 0;
	int num_cells = 0;
	int chg_leds = 0;
	int charging = 0;
	int pack_connected = 0;
	int c;
	double stddev;
	rc_model_t model;
	rc_filter_t filterB = rc_filter_empty();
	rc_filter_t filterJ = rc_filter_empty(); // battery and jack filters

	// ensure root privaleges until we sort out udev rules
	if(geteuid()!=0){
		fprintf(stderr,"ERROR: rc_battery_monitor must be run as root\n");
		return -1;
	}

	// parse arguments to check for kill mode
	opterr = 0;
	while ((c = getopt(argc, argv, "k")) != -1){
		switch (c){
		case 'k':  // kill mode
			return kill_existing_instance();
			break;

		default:
			fprintf(stderr,"\nInvalid Argument \n");
			return -1;
			break;
		}
	}

	// whitelist blue, black, and black wireless only when RC device tree is in use
	model = rc_model();
	if(model!=MODEL_BB_BLACK_RC && model!=MODEL_BB_BLACK_W_RC && model!=MODEL_BB_BLUE){
		if(system("grep -q roboticscape /boot/uEnv.txt")!=0){
			fprintf(stderr,"rc_battery_monitor can only run on BB Blue, Black, and Black wireless when the roboticscape device tree is in use.\n");
			return 0;
		}
	}

	// we only want one instance running, so check is a pid file already exists
	if(access(BATTPIDFILE, F_OK ) == 0){
		// PID file exists
		fprintf(stderr,"WARNING: instance of rc_battery_monitor already running\n");
		fprintf(stderr,"Killing it and starting a new instance\n");
		kill_existing_instance();
	}

	// make new pid file
	fd = fopen(BATTPIDFILE, "w");
	if(fd==NULL){
		perror("ERROR in rc_battery_monitor opening PID file");
		return -1;
	}
	fprintf(fd,"%d",(int)getpid());
	fflush(fd);
	fclose(fd);

	// set up signal handler
	signal(SIGINT, shutdown_signal_handler);
	signal(SIGTERM, shutdown_signal_handler);

	// set all pins off and check that is works
	if(rc_led_set(RC_LED_BAT25,0)) return -1;
	if(rc_led_set(RC_LED_BAT50,0)) return -1;
	if(rc_led_set(RC_LED_BAT75,0)) return -1;
	if(rc_led_set(RC_LED_BAT100,0)) return -1;

	// enable adc
	if(rc_adc_init()) return -1;

	// start filters
	v_pack = rc_adc_batt();
	if(rc_filter_moving_average(&filterB, FITLER_SAMPLES, 1000000.0/LOOP_HZ)){
		fprintf(stderr,"ERROR in rc_battery_monitor, failed to create filter\n");
		remove(BATTPIDFILE);
		return -1;
	}
	rc_filter_prefill_outputs(&filterB, v_pack);
	rc_filter_prefill_inputs(&filterB, v_pack);
	v_jack = rc_adc_dc_jack();
	if(rc_filter_moving_average(&filterJ, FITLER_SAMPLES, 1000000.0/LOOP_HZ)){
		fprintf(stderr,"ERROR in rc_battery_monitor, failed to create filter\n");
		remove(BATTPIDFILE);
		return -1;
	}
	rc_filter_prefill_outputs(&filterJ, v_jack);
	rc_filter_prefill_inputs(&filterJ, v_jack);

	// first decide if the user has called this from a terminal
	// or as a startup process
	if(isatty(fileno(stdout))){
		printing = 1;
		printf("\n2S Pack   Jack   #Cells   Cell\n");
	}

	// blink for fun!
	startup_blink();

	// run until running==0 which is set by signal handler
	running = 1;
	while(running){
		charging = 0;
		// read in the voltage of the 2S pack and DC jack
		new_v_pack = rc_adc_batt();
		new_v_jack = rc_adc_dc_jack();

		if(new_v_pack<0.0 || new_v_jack<0.0){
			fprintf(stderr,"ERROR in rc_battery_monitor, can't read ADC voltages\n");
			remove(BATTPIDFILE);
			return -1;
		}

		// if there is a sudden jump due to connection or disconnection
		// reset the filters
		if(abs(new_v_pack-v_pack)>2){
			rc_filter_prefill_outputs(&filterB, new_v_pack);
			rc_filter_prefill_inputs(&filterB, new_v_pack);
			startup_blink();
		}
		if(abs(new_v_jack-v_jack)>2){
			rc_filter_prefill_outputs(&filterJ, new_v_jack);
			rc_filter_prefill_inputs(&filterJ, new_v_jack);
			startup_blink();
		}
		// marge moving average filter
		v_pack = rc_filter_march(&filterB, new_v_pack);
		v_jack = rc_filter_march(&filterJ, new_v_jack);


		// find standard deviation of battery signal to determine
		// if a 2S pack is connected or not
		if(v_pack>(2*CELL_DIS)) stddev = rc_ringbuf_std_dev(filterB.in_buf);

		// check if 2s pack if connected
		if(v_pack>(2*CELL_DIS) && stddev<STD_DEV_TOL){
			pack_connected = 1;
			cell_voltage = v_pack/2;
			num_cells = 2;
		}
		else pack_connected = 0;

		// check charging condition
		if(pack_connected && v_jack>10.0 && cell_voltage<V_CHG_DETECT){
			charging = 1;
		}
		else charging = 0;

		// no 2S pack on the White 3-pin connector, check for dc jack batteries
		if(!pack_connected){
			// check 4S condition
			if(v_jack>CELL_DIS*4 && v_jack<CELL_MAX*4){
				num_cells = 4;
				cell_voltage = v_jack/4;
			}
			// check for 3S condition
			else if(v_jack>CELL_DIS*3 && v_jack<CELL_MAX*3){
				num_cells = 3;
				cell_voltage = v_jack/3;
			}
			// check for 2S condition
			else if(v_jack>CELL_DIS*2 && v_jack<CELL_MAX*2){
				num_cells = 2;
				cell_voltage = v_jack/2;
			}
			// no pack connected
			else {
				cell_voltage = 0;
				num_cells = 0;
			}
		}

		// done sensing, start outputting
		if(printing){
			printf("\r %0.2fV   %0.2fV	 %d	 %0.2fV   ", \
				v_pack, v_jack, num_cells, cell_voltage);
			fflush(stdout);
		}

		// if charging, blink in a charging pattern
		if(charging){
			chg_leds += 1;
			if(chg_leds>4) chg_leds=0;
			illuminate_leds(chg_leds);
		}
		// illuminate LEDs properly if not charging
		else if(num_cells==0) illuminate_leds(0);
		// turn off LEDs if obviously 12v power supply
		else if(num_cells!=2 && v_jack>11.5 && v_jack<12.5){
			illuminate_leds(0);
		}
		// normal battery discharging
		else if(cell_voltage<CELL_DIS)	illuminate_leds(0);
		else if(cell_voltage>CELL_FULL)	illuminate_leds(4);
		else if(cell_voltage>CELL_75)	illuminate_leds(3);
		else if(cell_voltage>CELL_50)	illuminate_leds(2);
		else if(cell_voltage>CELL_25)	illuminate_leds(1);
		// battery is extremely 0, blink all 4
		else{
			if(toggle) toggle=0;
			else toggle=4;
			illuminate_leds(toggle);
		}
		// sleepy time
		rc_usleep(LOOP_SLEEP_US);
	}

	// exit
	illuminate_leds(0);
	printf("battery_monitor exiting cleanly\n");
	remove(BATTPIDFILE);
	return 0;
}


/**
 * shutdown_signal_handler(int signo)
 *
 * catch Ctrl-C signal and change running state to 0 indicating exiting
 *
 * @param[in]  signo  The signo
 */
void shutdown_signal_handler(int signo)
{
	if (signo == SIGINT){
		running = 0;
 	}else if (signo == SIGTERM){
		running = 0;
		printf("\nreceived SIGTERM\n");
 	}
}


/**
 * @brief      kill existing instance found in PID file
 *
 * @return     -1 if program had to be killed, 0 it wasn't running or exited
 *             cleanly. 1 on weird behavior like malformed PID file.
 */
int kill_existing_instance()
{
	FILE* fd;
	int old_pid, i;

	// attempt to open PID file
	fd = fopen(BATTPIDFILE, "r");
	// if the file didn't open, no proejct is runnning in the background
	// so return 0
	if (fd == NULL) {
		return 0;
	}

	// otherwise try to read the current process ID
	fscanf(fd,"%d", &old_pid);
	fclose(fd);

	// if the file didn't contain a PID number, remove it and
	// return -1 indicating weird behavior
	if(old_pid == 0){
		remove(BATTPIDFILE);
		return 1;
	}

	// attempt a clean shutdown
	kill((pid_t)old_pid, SIGINT);

	// check every 0.1 seconds to see if it closed
	for(i=0; i<20; i++){
		if(getpgid(old_pid) >= 0) rc_usleep(100000);
		else{ // succcess, it shut down properly
			remove(BATTPIDFILE);
			return 1;
		}
	}

	// otherwise force kill the program if the PID file never got cleaned up
	kill((pid_t)old_pid, SIGKILL);

	// close and delete the old file
	remove(BATTPIDFILE);

	// return -1 indicating the program had to be killed
	return -1;
}


/**
 * @brief      illuminates 0-4 leds
 *
 * @param[in]  i     leds to illuminate, 0-4
 */
void illuminate_leds(int i)
{
	switch(i){
	// now illuminate LEDs properly
	case 4:
		rc_led_set(RC_LED_BAT25,1);
		rc_led_set(RC_LED_BAT50,1);
		rc_led_set(RC_LED_BAT75,1);
		rc_led_set(RC_LED_BAT100,1);
		break;
	case 3:
		rc_led_set(RC_LED_BAT25,1);
		rc_led_set(RC_LED_BAT50,1);
		rc_led_set(RC_LED_BAT75,1);
		rc_led_set(RC_LED_BAT100,0);
		break;
	case 2:
		rc_led_set(RC_LED_BAT25,1);
		rc_led_set(RC_LED_BAT50,1);
		rc_led_set(RC_LED_BAT75,0);
		rc_led_set(RC_LED_BAT100,0);
		break;
	case 1:
		rc_led_set(RC_LED_BAT25,1);
		rc_led_set(RC_LED_BAT50,0);
		rc_led_set(RC_LED_BAT75,0);
		rc_led_set(RC_LED_BAT100,0);
		break;
	case 0:
		rc_led_set(RC_LED_BAT25,0);
		rc_led_set(RC_LED_BAT50,0);
		rc_led_set(RC_LED_BAT75,0);
		rc_led_set(RC_LED_BAT100,0);
		break;
	default:
		fprintf(stderr,"ERROR in rc_battery_monitor, can only illuminate between 0 and 4 leds\n");
		fprintf(stderr,"attempted %d\n", i);
		break;
	}
	return;
}


/**
 * @brief      slides one illuinated LED state from bottom to top and back again.
 *
 * @return     0 on success, -1 on failure
 */
static int startup_blink()
{
	// start with lighting up the bottom led
	rc_led_set(RC_LED_BAT25,1);
	rc_led_set(RC_LED_BAT50,0);
	rc_led_set(RC_LED_BAT75,0);
	rc_led_set(RC_LED_BAT100,0);
	rc_usleep(START_BLINK_US);

	rc_led_set(RC_LED_BAT25,0);
	rc_led_set(RC_LED_BAT50,1);
	rc_usleep(START_BLINK_US);

	rc_led_set(RC_LED_BAT50,0);
	rc_led_set(RC_LED_BAT75,1);
	rc_usleep(START_BLINK_US);

	rc_led_set(RC_LED_BAT75,0);
	rc_led_set(RC_LED_BAT100,1);
	rc_usleep(START_BLINK_US);

	rc_led_set(RC_LED_BAT100,0);
	rc_led_set(RC_LED_BAT75,1);
	rc_usleep(START_BLINK_US);

	rc_led_set(RC_LED_BAT75,0);
	rc_led_set(RC_LED_BAT50,1);
	rc_usleep(START_BLINK_US);

	rc_led_set(RC_LED_BAT50,0);
	rc_led_set(RC_LED_BAT25,1);
	rc_usleep(START_BLINK_US);

	rc_led_set(RC_LED_BAT25,0);
	rc_usleep(START_BLINK_US);
	return 0;
}


