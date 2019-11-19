/**
 * @file model.c
 *
 * @author     James Strawson
 * @date       2/23/2018
 */


#include <stdio.h>
#include <string.h>
#include <stdlib.h> // for system()
#include <rc/model.h>

#define MODEL_DIR "/proc/device-tree/model"
#define BUF_SIZE 128

// current model stored in memory as enum for fast access
static rc_model_t current_model;
static rc_model_category_t current_category;

// set to 1 once the model id has been pulled from /proc/
static int has_checked = 0;



static void __check_model(void)
{
	char c[BUF_SIZE];
	int ret;
	FILE *fd;

	// start as unknown until finding out more info.
	current_model = MODEL_UNKNOWN;
	current_category = CATEGORY_UNKNOWN;
	has_checked = 1;

	// check for x86/x86_64 personal comptuer
	ret=system("uname -m | grep -q x86");
	if(ret==0){
		current_model = MODEL_PC;
		current_category = CATEGORY_PC;
		return;
	}

	// read device tree, all other models have a device tree
	fd = fopen(MODEL_DIR, "r");
	if(fd == NULL){
		// on fail, leave model as unknown
		return;
	}

	// read model fom device tree
	memset(c, 0, BUF_SIZE);
	if(fgets(c, BUF_SIZE, fd)==NULL){
		perror("ERROR in rc_model reading /proc/device-tree/model");
		fclose(fd);
		return;
	}
	fclose(fd);

	// now do the checks for known BB models
	if(strstr(c, "BeagleBone")!=NULL || strstr(c, "PocketBeagle")!=NULL){

		current_category = CATEGORY_BEAGLEBONE;

		if(strcmp(c, "TI AM335x BeagleBone Black")==0){
			current_model = MODEL_BB_BLACK;
			return;
		}
		if(strcmp(c, "TI AM335x BeagleBone Black RoboticsCape")==0){
			current_model = MODEL_BB_BLACK_RC;
			return;
		}
		if(strcmp(c, "TI AM335x BeagleBone Black Wireless")==0){
			current_model = MODEL_BB_BLACK_W;
			return;
		}
		if(strcmp(c, "TI AM335x BeagleBone Black Wireless RoboticsCape")==0){
			current_model = MODEL_BB_BLACK_W_RC;
			return;
		}
		if(strcmp(c, "TI AM335x BeagleBone Green")==0){
			current_model = MODEL_BB_GREEN;
			return;
		}
		if(strcmp(c, "TI AM335x BeagleBone Green Wireless")==0){
			current_model = MODEL_BB_GREEN_W;
			return;
		}
		if(strcmp(c, "TI AM335x BeagleBone Blue")==0){
			current_model = MODEL_BB_BLUE;
			return;
		}
		if(strcmp(c, "TI AM335x PocketBeagle")==0) {
			current_model = MODEL_BB_POCKET;
			return;
		}
		return;
	}

	// rpi models
	// use strstr instead of strcmp because the string may contain revision
	// number as well as model name. Therefore order of checks is important.
	if(strstr(c, "Raspberry Pi")!=NULL){

		current_category=CATEGORY_RPI;

		if(strstr(c, "Raspberry Pi Model B+")!=NULL){
			current_model = MODEL_RPI_B_PLUS;
			return;
		}
		if(strstr(c, "Raspberry Pi Model B")!=NULL){
			current_model = MODEL_RPI_B;
			return;
		}

		if(strstr(c, "Raspberry Pi 2 Model B")!=NULL){
			current_model = MODEL_RPI2_B;
			return;
		}

		if(strstr(c, "Raspberry Pi 3 Model B+")!=NULL){
			current_model = MODEL_RPI3_B_PLUS;
			return;
		}
		if(strstr(c, "Raspberry Pi 3 Model")!=NULL){
			current_model = MODEL_RPI3_B;
			return;
		}

		if(strstr(c, "Raspberry Pi Zero W")!=NULL){
			current_model = MODEL_RPI0_W;
			return;
		}
		if(strstr(c, "Raspberry Pi Zero")!=NULL){
			current_model = MODEL_RPI0;
			return;
		}

		if(strstr(c, "Raspberry Pi Computer Module 3")!=NULL){
			current_model = MODEL_RPI_CM3;
			return;
		}
		if(strstr(c, "Raspberry Pi Compute Module")!=NULL){
			current_model = MODEL_RPI_CM;
			return;
		}
		return;
	}

	// if we've gotten here, no specific model was found.

	return;
}


rc_model_t rc_model(void)
{
	if(has_checked) return current_model;

	__check_model();

	return current_model;
}

rc_model_category_t rc_model_category(void)
{
	if(has_checked) return current_category;

	__check_model();
	has_checked = 1;
	return current_model;
}


#define caseprint(X) case(X): printf(#X); break;

void rc_model_print(void)
{
	rc_model_t model = rc_model();

	switch(model){

	caseprint(MODEL_UNKNOWN)
	caseprint(MODEL_BB_BLACK)
	caseprint(MODEL_BB_BLACK_RC)
	caseprint(MODEL_BB_BLACK_W)
	caseprint(MODEL_BB_BLACK_W_RC)
	caseprint(MODEL_BB_GREEN)
	caseprint(MODEL_BB_GREEN_W)
	caseprint(MODEL_BB_BLUE)
	caseprint(MODEL_BB_POCKET)
	caseprint(MODEL_RPI_B)
	caseprint(MODEL_RPI_B_PLUS)
	caseprint(MODEL_RPI2_B)
	caseprint(MODEL_RPI3_B)
	caseprint(MODEL_RPI3_B_PLUS)
	caseprint(MODEL_RPI0)
	caseprint(MODEL_RPI0_W)
	caseprint(MODEL_RPI_CM)
	caseprint(MODEL_RPI_CM3)
	caseprint(MODEL_PC)

	default:
		fprintf(stderr, "ERROR: in rc_model_print, invalid model detected\n");
		break;
	}
	return;
}

void rc_model_category_print(void)
{
	rc_model_category_t category = rc_model_category();

	switch(category){

	caseprint(CATEGORY_UNKNOWN)
	caseprint(CATEGORY_BEAGLEBONE)
	caseprint(CATEGORY_RPI)
	caseprint(CATEGORY_PC)


	default:
		fprintf(stderr, "ERROR: in rc_model_category_print, invalid category detected\n");
		break;
	}
	return;
}

#undef caseprint