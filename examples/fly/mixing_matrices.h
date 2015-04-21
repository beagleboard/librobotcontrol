// mixing_matrices.h
// James Strawson - 2015
// MultiRotors are controlled by mixing roll, pitch, yaw, and throttle
// control oututs, a linear combination of which forms the control output
// to each motor. The coefficients to this combination is stored in a 
// mixing matrix based on rotor layout.
//
// Also included here are functions to parse configuration strings 
// and do the actual mixing

#define MIX_ROWS 4	// 4 control inputs	
#define MIX_COLS 8	// up to 8 rotors

// global variable to this c file only
float mix_matrix[MIX_ROWS][MIX_COLS]; 

/************************************************************************
* 	parse_mix_layout()
*	this contains all configuration data for the flight_core
*	the global instance core_config is populated before launching 
*	the flight core. It can be modified while flying, eg to adjust gains
*	an instance should be declared in your own C file
************************************************************************/
int parse_mix_layout(int rotors, char layout){
	// start with sanity checks
	// current 4, 6, 8 rotors supported with + and X layouts
	switch(rotors){
		case 4: break;
		case 6: break;
		case 8: break;
		default:
			printf("ERROR, rotors must be 4, 6, or 8\n");
			return -1;
	}
	switch(layout){
		case 'x': layout='X';
		case 'X': break;
		case '+': break;
		default:
			printf("ERROR, layout must be X or +\n");
			return -1;
	}
	
	//wipe matrix
	memset(&mix_matrix, 0, MIX_ROWS*MIX_COLS*sizeof(float));

	// now fill the the mix matrix appropriately
	// order: roll, pitch, yaw, throttle
	if(rotors==4 && layout=='X'){
		float new_matrix[MIX_ROWS][MIX_COLS] = \
				   {{-1, 1, 1,-1},
					{ 1, 1,-1,-1},
					{-1, 1,-1, 1},
					{ 1, 1, 1, 1}};
		memcpy(&mix_matrix,&new_matrix, sizeof(mix_matrix));
	}
	else if(rotors==4 && layout=='+'){
		float new_matrix[MIX_ROWS][MIX_COLS] = \
				   {{-1,0,1,0},
					{0,1,0,-1},
					{-1,1,-1,1},
					{1,1,1,1}};
		memcpy(&mix_matrix,&new_matrix, sizeof(mix_matrix));
	}
	else if(rotors==6 && layout=='X'){
		float new_matrix[MIX_ROWS][MIX_COLS] = \
				   {{-1,-0.5,0.5,1, 0.5, -0.5},
					{0, 1,1,0,-1,-1},
					{1,-1,1,-1,1,-1},
					{1,1,1,1,1,1}};
		memcpy(&mix_matrix,&new_matrix, sizeof(mix_matrix));
	}
	else if(rotors==6 && layout=='+'){
		float new_matrix[MIX_ROWS][MIX_COLS] = \
				   {{-1,0,1,1, 0, -1},
					{0.5, 1,0.5,-0.5,-1,-0.5},
					{-1,1,-1,1,-1,1},
					{1,1,1,1,1,1}};
		memcpy(&mix_matrix,&new_matrix, sizeof(mix_matrix));
	}
	else if(rotors==8 && layout=='X'){
		float new_matrix[MIX_ROWS][MIX_COLS] = \
				   {{-1,-0.4142,0.4142,1, 1, 0.4142, -0.4142, -1},
					{0.4142,1,1,0.4142,-0.4142,-1,-1,-0.4142},
					{-1,1,-1,1,-1,1,-1,1},
					{1,1,1,1,1,1,1,1}};
		memcpy(&mix_matrix,&new_matrix, sizeof(mix_matrix));
	}
	else if(rotors==8 && layout=='+'){
		float new_matrix[MIX_ROWS][MIX_COLS] = \
				   {{-1, -0.7071,0, 0.7071, 1,  0.7071,0,-0.7071},
					{0,0.7071,1,0.7071,0,-0.7071,-1,-0.707},
					{1,-1,1,-1,1,-1,1,-1},
					{1,1,1,1,1,1,1,1}};
		memcpy(&mix_matrix,&new_matrix, sizeof(mix_matrix));
	}
	else{
		printf("ERROR: incompatible frame layout\n");
		return -1;
	}
	
	return 0;
}

/************************************************************************
*  mix_controls()
*	fills the vector esc with the linear combination of roll pitch
*	yaw and throttle based on mixing matrix
************************************************************************/
int mix_controls(float r, float p, float y, float t, float* esc, int rotors){
	int i = 0;
	if(mix_matrix==0){
		printf("mix matrix not configured yet\n");
		printf("use parse_layout()\n");
		return -1;
	}
	// sum control inputs
	for(i=0; i<rotors; i++){
		esc[i]=0;
		esc[i]+=r*mix_matrix[0][i];
		esc[i]+=p*mix_matrix[1][i];
		esc[i]+=y*mix_matrix[2][i];
		esc[i]+=t*mix_matrix[3][i];			
	}
	return 0;
}


