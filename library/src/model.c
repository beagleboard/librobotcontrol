/**
 * @file model.c
 *
 * @author     James Strawson
 * @date       2/23/2018
 */


#include <stdio.h>
#include <string.h>
#include <rc/model.h>

#define MODEL_DIR "/proc/device-tree/model"
#define BUF_SIZE 128

// current model stored in memory as enum for fast access
static rc_model_t current_model;

// set to 1 once the model id has been pulled from /proc/
static int has_checked = 0;



static rc_model_t model_from_device_tree()
{
	rc_model_t model;
	char c[BUF_SIZE];
	FILE *fd;

	fd = fopen(MODEL_DIR, "r");
	if(fd == NULL){
		fprintf(stderr,"ERROR: in rc_model_get, can't open %s \n", MODEL_DIR);
		return UNKNOWN_MODEL;
	}
	// read model
	memset(c, 0, BUF_SIZE);
	if(fgets(c, BUF_SIZE, fd)==NULL){
		perror("ERROR reading model");
		fclose(fd);
		return UNKNOWN_MODEL;
	}
	fclose(fd);
	// now do the checks
	if(     strcmp(c, "TI AM335x BeagleBone Black"			)==0) model=BB_BLACK;
	else if(strcmp(c, "TI AM335x BeagleBone Black RoboticsCape"	)==0) model=BB_BLACK_RC;
	else if(strcmp(c, "TI AM335x BeagleBone Blue"			)==0) model=BB_BLUE;
	else if(strcmp(c, "TI AM335x BeagleBone Black Wireless"		)==0) model=BB_BLACK_W;
	else if(strcmp(c, "TI AM335x BeagleBone Black Wireless RoboticsCape")==0) model=BB_BLACK_W_RC;
	else if(strcmp(c, "TI AM335x BeagleBone Green"			)==0) model=BB_GREEN;
	else if(strcmp(c, "TI AM335x BeagleBone Green Wireless"		)==0) model=BB_GREEN_W;
	else model = UNKNOWN_MODEL;

	return model;
}


rc_model_t rc_model()
{
	if(has_checked) return current_model;

	current_model = model_from_device_tree();
	has_checked = 1;
	return current_model;
}


void rc_model_print()
{
	rc_model_t model = rc_model();

	switch(model){
	case(UNKNOWN_MODEL):
		printf("UNKNOWN_MODEL");
		break;
	case(BB_BLACK):
		printf("BB_BLACK");
		break;
	case(BB_BLACK_RC):
		printf("BB_BLACK_RC");
		break;
	case(BB_BLACK_W):
		printf("BB_BLACK_W");
		break;
	case(BB_BLACK_W_RC):
		printf("BB_BLACK_W_RC");
		break;
	case(BB_GREEN):
		printf("BB_GREEN");
		break;
	case(BB_GREEN_W):
		printf("BB_GREEN_W");
		break;
	case(BB_BLUE):
		printf("BB_BLUE");
		break;
	default:
		fprintf(stderr, "ERROR: in rc_model_print, invalid model detected\n");
		break;
	}
	return;
}
