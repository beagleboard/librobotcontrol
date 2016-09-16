/*******************************************************************************
* configure_robotics_pins.c
*
* James Strawson 2016
* This sets up the pins needed by the robotics cape via cape-universal and
* should be called first thing on boot by robot.service before the cape
* library is used.
*******************************************************************************/

#include <useful_includes.h>
// #include <robotics_cape.h>
// #include <robotics_cape_defs.h>

#define TIMEOUT_S 30
#define SLOTS_DIR " /sys/devices/platform/bone_capemgr/slots"
int load_cape_universal();
int configure_pins();
//int check_timeout();




/*******************************************************************************
* int main()
*
*******************************************************************************/
int main(){
	
	// log start time 
	//start_us = micros_since_epoch();

	if(load_cape_universal()<0){
		printf("failed to load cape-universal\n");
		return -1;
	}
	
	if(configure_pins()<0){
		printf("failed configure_pins\n");
		return -1;
	}
		
	printf("success, cape ready\n");
	return 0;
}

/*******************************************************************************
* int load_cape_universal()
*
* makes sure the right cape overlays are loaded. cape-universala exports all
* pins and is sufficient. Alternatively the combination of cape-univeraln
* and cape-univ-hdmi would work. First check for a and n, if n is there then
* try exporting cape-univ-hdmi to fill in the last pins.
*
* returns 0 on success, -1 on failure.
*******************************************************************************/
int load_cape_universal(){
	int ret;
	
	// check cape-universal first
	ret = system("grep -q 'cape-universal'" SLOTS_DIR);
	if(ret == 0) return 0; // success
	
	// now check cape-universaln
	ret = system("grep -q 'cape-universaln'" SLOTS_DIR);
	// if cape-universaln is not there, try exporting it
	if(ret != 0){
		ret = system("echo cape-universaln >" SLOTS_DIR);
		if(ret!=0){
			printf("can't export cape-universaln\n");
			return -1;
		}
	}
	
	// now need cape-univ-hdmi, check if it's there first
	ret = system("grep -q 'cape-univ-hdmi'" SLOTS_DIR);
	// if it's not there, try exporting it
	if(ret != 0){
		ret = system("echo 'cape-univ-hdmi' >" SLOTS_DIR);
		if(ret!=0){
			printf("can't export cape-univ-hdmi\n");
			return -1;
		}
	}
	
	return 0;
}

/*******************************************************************************
* int configure_pins()
*
* pinmux everything with cape-universal
*
* returns 0 on success, -1 on failure.
*******************************************************************************/
int configure_pins(){
	//P8
	if(system("config-pin P8.7 gpio")){
		printf("failed to configure P8.7\n");
		return -1;
	}
	if(system("config-pin P8.8 gpio")){
		printf("failed to configure P8.8\n");
		return -1;
	}
	if(system("config-pin P8.9 gpio_pu")){
		printf("failed to configure P8.7\n");
		return -1;
	}
	if(system("config-pin P8.10 gpio_pu")){
		printf("failed to configure P8.10\n");
		return -1;
	}
	if(system("config-pin P8.11 qep")){
		printf("failed to configure P8.11\n");
		return -1;
	}
	if(system("config-pin P8.12 qep")){
		printf("failed to configure P8.12\n");
		return -1;
	}
	if(system("config-pin P8.13 pwm")){
		printf("failed to configure P8.13\n");
		return -1;
	}
	if(system("config-pin P8.14 gpio")){
		printf("failed to configure P8.14\n");
		return -1;
	}
	if(system("config-pin P8.15 pruin")){
		printf("failed to configure P8.15\n");
		return -1;
	}
	if(system("config-pin P8.16 pruin")){
		printf("failed to configure P8.16\n");
		return -1;
	}
	if(system("config-pin P8.17 gpio")){
		printf("failed to configure P8.17\n");
		return -1;
	}
	if(system("config-pin P8.18 gpio")){
		printf("failed to configure P8.18\n");
		return -1;
	}
	if(system("config-pin P8.19 pwm")){
		printf("failed to configure P8.19\n");
		return -1;
	}
	if(system("config-pin P8.26 gpio")){
		printf("failed to configure P8.26\n");
		return -1;
	}
	if(system("config-pin P8.27 pruout")){
		printf("failed to configure P8.27\n");
		return -1;
	}
	if(system("config-pin P8.28 pruout")){
		printf("failed to configure P8.28\n");
		return -1;
	}
	if(system("config-pin P8.29 pruout")){
		printf("failed to configure P8.29\n");
		return -1;
	}
	if(system("config-pin P8.30 pruout")){
		printf("failed to configure P8.30\n");
		return -1;
	}
	if(system("config-pin P8.33 qep")){
		printf("failed to configure P8.33\n");
		return -1;
	}
	if(system("config-pin P8.34 gpio")){
		printf("failed to configure P8.34\n");
		return -1;
	}
	if(system("config-pin P8.35 qep")){
		printf("failed to configure P8.35\n");
		return -1;
	}
	if(system("config-pin P8.36 gpio")){
		printf("failed to configure P8.36\n");
		return -1;
	}
	if(system("config-pin P8.37 uart")){
		printf("failed to configure P8.37\n");
		return -1;
	}
	if(system("config-pin P8.38 uart")){
		printf("failed to configure P8.38\n");
		return -1;
	}
	if(system("config-pin P8.39 pruout")){
		printf("failed to configure P8.39\n");
		return -1;
	}
	if(system("config-pin P8.40 pruout")){
		printf("failed to configure P8.40\n");
		return -1;
	}
	if(system("config-pin P8.41 pruout")){
		printf("failed to configure P8.41\n");
		return -1;
	}
	if(system("config-pin P8.42 pruout")){
		printf("failed to configure P8.42\n");
		return -1;
	}
	if(system("config-pin P8.43 gpio")){
		printf("failed to configure P8.43\n");
		return -1;
	}
	if(system("config-pin P8.44 gpio")){
		printf("failed to configure P8.44\n");
		return -1;
	}
	if(system("config-pin P8.45 gpio")){
		printf("failed to configure P8.45\n");
		return -1;
	}
	if(system("config-pin P8.46 gpio")){
		printf("failed to configure P8.46\n");
		return -1;
	}
	
	
	// P9
	if(system("config-pin P9.11 uart")){
		printf("failed to configure P9.11\n");
		return -1;
	}
	if(system("config-pin P9.12 gpio")){
		printf("failed to configure P9.12\n");
		return -1;
	}
	if(system("config-pin P9.13 gpio")){
		printf("failed to configure P9.13\n");
		return -1;
	}
	if(system("config-pin P9.14 pwm")){
		printf("failed to configure P9.14\n");
		return -1;
	}
	if(system("config-pin P9.15 gpio")){
		printf("failed to configure P9.15\n");
		return -1;
	}
	if(system("config-pin P9.16 uart")){
		printf("failed to configure P9.11\n");
		return -1;
	}
	if(system("config-pin P9.17 i2c")){
		printf("failed to configure P9.17\n");
		return -1;
	}
	if(system("config-pin P9.18 i2c")){
		printf("failed to configure P9.18\n");
		return -1;
	}
	if(system("config-pin P9.19 i2c")){
		printf("failed to configure P9.19\n");
		return -1;
	}
	if(system("config-pin P9.20 i2c")){
		printf("failed to configure P9.20\n");
		return -1;
	}
	if(system("config-pin P9.21 uart")){
		printf("failed to configure P9.21\n");
		return -1;
	}
	if(system("config-pin P9.22 uart")){
		printf("failed to configure P9.22\n");
		return -1;
	}
	if(system("config-pin P9.23 gpio")){
		printf("failed to configure P9.23\n");
		return -1;
	}
	if(system("config-pin P9.24 uart")){
		printf("failed to configure P9.24\n");
		return -1;
	}
	if(system("config-pin P9.25 gpio_pu")){
		printf("failed to configure P9.17\n");
		return -1;
	}
	if(system("config-pin P9.26 uart")){
		printf("failed to configure P9.26\n");
		return -1;
	}
	if(system("config-pin P9.27 qep")){
		printf("failed to configure P9.27\n");
		return -1;
	}
	if(system("config-pin P9.28 gpio")){
		printf("failed to configure P9.28\n");
		return -1;
	}
	if(system("config-pin P9.29 spi")){
		printf("failed to configure P9.29\n");
		return -1;
	}
	if(system("config-pin P9.30 spi")){
		printf("failed to configure P9.30\n");
		return -1;
	}
	if(system("config-pin P9.31 spi")){
		printf("failed to configure P9.31\n");
		return -1;
	}
	if(system("config-pin P9.41 gpio")){
		printf("failed to configure P9.41\n");
		return -1;
	}
	if(system("config-pin P9.42 qep")){
		printf("failed to configure P9.42\n");
		return -1;
	}
	
	return 0;
}



// /*******************************************************************************
// * int check_timeout()
// *
// * looks and the current time to decide if the timeout has been reached.
// * returns 1 if timeout has been reached.
// *******************************************************************************/
// int check_timeout(){
	// uint64_t new_us = micros_since_epoch();
	// int seconds = (new_us-start_us)/1000000;
	// if(seconds>TIMEOUT_S){
		// printf("TIMEOUT REACHED\n");
		// return 1;
	// }
	// return 0;
// }