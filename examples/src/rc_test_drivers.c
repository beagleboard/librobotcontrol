/**
 * @example    rc_test_drivers.c
 *
 * without actually calling library functions, this checks that all the drivers
 * necessary for the robot control library to fully work.
 **/

#include <stdio.h>
#include <stdlib.h> // for system()
#include <unistd.h> // for access()
#include <rc/model.h>
#include <rc/version.h>


int main()
{
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
	if(access("/dev/gpiochip0", F_OK ) != 0){
		printf("ERROR:  gpio 0 driver not loaded\n");
	} else printf("PASSED: gpio 0\n");
	if(access("/dev/gpiochip1", F_OK ) != 0){
		printf("ERROR:  gpio 1 driver not loaded\n");
	} else printf("PASSED: gpio 1\n");
	if(access("/dev/gpiochip2", F_OK ) != 0){
		printf("ERROR:  gpio 2 driver not loaded\n");
	} else printf("PASSED: gpio 2\n");
	if(access("/dev/gpiochip3", F_OK ) != 0){
		printf("ERROR:  gpio 3 driver not loaded\n");
	} else printf("PASSED: gpio 3\n");

	// pwm 0, 1, 2
	if(access("/sys/devices/platform/ocp/48300000.epwmss/48300200.pwm/pwm/", F_OK ) != 0){
		printf("ERROR:  ti-pwm driver not loaded for hrpwm0\n");
	} else printf("PASSED: pwm0\n");

	if(access("/sys/devices/platform/ocp/48302000.epwmss/48302200.pwm/pwm/", F_OK ) != 0){
		printf("ERROR:  ti-pwm driver not loaded for hrpwm1\n");
	} else printf("PASSED: pwm1\n");

	if(access("/sys/devices/platform/ocp/48304000.epwmss/48304200.pwm/pwm/", F_OK ) != 0){
		printf("ERROR:  ti-pwm driver not loaded for hrpwm2\n");
	} else printf("PASSED: pwm2\n");

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
	if(access("/dev/remoteproc/pruss-core0/state", F_OK ) != 0){
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

	// i2c
	if(access("/dev/i2c-1", F_OK ) != 0){
		printf("ERROR:  i2c1 driver not loaded\n");
	} else printf("PASSED: i2c1\n");
	if(access("/dev/i2c-2", F_OK ) != 0){
		printf("ERROR:  i2c2 driver not loaded\n");
	} else printf("PASSED: i2c2\n");

	// spi
	if(access("/dev/spidev1.0", F_OK ) != 0){
		if(access("/dev/spidev2.0", F_OK ) != 0){
			printf("ERROR: spi driver not loaded\n");
		}
		else{
			printf("WARNING: SPI1 loaded but incorrectly enumerated as /dev/spidev2\n");
		}
	} else printf("PASSED: spi\n");

	// LEDs
	if(access("/sys/class/leds/green/brightness", F_OK ) != 0){
		printf("ERROR:  LED driver not loaded\n");
	} else printf("PASSED: LED\n");

	// ADC
	if(access("/sys/bus/iio/devices/iio:device0/in_voltage0_raw", F_OK ) != 0){
		printf("ERROR:  ADC iio driver not loaded\n");
	} else printf("PASSED: ADC iio\n");

	printf("\nCurrently running on a:\n");
	rc_model_print();
	printf("\nRobot Control library Version:\n");
	rc_version_print();
	printf("\n");


	return 0;
}
