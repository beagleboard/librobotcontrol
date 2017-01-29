/*******************************************************************************
* rc_test_drivers.c
*
* without calling initialize_cape, this checks that all the drivers necessary
* for libroboticscape to work
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

int main(){
	
	printf("\n");
	printf("Kernel: ");
	fflush(stdout);
	system("uname -r");
	system("cat /etc/dogtag");
	printf("Debian: ");
	fflush(stdout);
	system("cat /etc/debian_version");
	printf("\n");

	// gpio
	if(access("/sys/class/gpio/export", F_OK ) != 0){
		printf("ERROR:  gpio driver not loaded\n");
	} else printf("PASSED: gpio\n");

	// pwm 1,2
	if(access("/sys/class/pwm/pwmchip2/export", F_OK ) != 0){
		printf("ERROR:  ti-pwm driver not loaded for hrpwm1\n");
	} else printf("PASSED: hrpwm1\n");
	
	if(access("/sys/class/pwm/pwmchip4/export", F_OK ) != 0){
		printf("ERROR:  ti-pwm driver not loaded for hrpwm2\n");
	} else printf("PASSED: hrpwm2\n");

	// eqep 0,1,2
	if(access("/sys/devices/platform/ocp/48300000.epwmss/48300180.eqep/position", F_OK ) != 0){
		printf("ERROR:  ti-eqep driver not loaded for eqep0\n");
	} else printf("PASSED: eqep0\n");

	if(access("/sys/devices/platform/ocp/48302000.epwmss/48302180.eqep/position", F_OK ) != 0){
		printf("ERROR:  ti-eqep driver not loaded for eqep1\n");
	} else printf("PASSED: eqep1\n");
	
	if(access("/sys/devices/platform/ocp/48304000.epwmss/48304180.eqep/position", F_OK ) != 0){
		printf("ERROR:  ti-eqep driver not loaded for eqep2\n");
	} else printf("PASSED: eqep2\n");

	// rproc
	if(access("/sys/bus/platform/drivers/pru-rproc/bind", F_OK ) != 0){
		printf("ERROR:  pru-rproc driver not loaded\n");
	} else printf("PASSED: pru-rproc\n");

	// uart
	if(access("/dev/ttyO1", F_OK ) != 0){
		printf("ERROR:  uart1 driver not loaded\n");
	} else printf("PASSED: uart1\n");
	if(access("/dev/ttyO2", F_OK ) != 0){
		printf("ERROR:  uart2 driver not loaded\n");
	} else printf("PASSED: uart2\n");
	if(access("/dev/ttyO4", F_OK ) != 0){
		printf("ERROR:  uart4 driver not loaded\n");
	} else printf("PASSED: uart4\n");
	if(access("/dev/ttyO5", F_OK ) != 0){
		printf("ERROR:  uart5 driver not loaded\n");
	} else printf("PASSED: uart5\n");

	// i2c, spi
	if(access("/dev/i2c-1", F_OK ) != 0){
		printf("ERROR:  i2c1 driver not loaded\n");
	} else printf("PASSED: i2c1\n");
	if(access("/dev/i2c-2", F_OK ) != 0){
		printf("ERROR:  i2c2 driver not loaded\n");
	} else printf("PASSED: i2c2\n");
	if(access("/dev/spidev1.0", F_OK ) != 0){
		printf("ERROR:  spi driver not loaded\n");
	} else printf("PASSED: spi\n");


	printf("\n");


	return 0;
}
