//prints normalized Spektrum DSM2 Satellite Reciever Data
//James Strawson 2014

#include <robotics_cape.h>



int main(){
	initialize_cape();
	initialize_spektrum();
	int i;
	while(get_state()!=EXITING){
		printf("\r");
		if(get_rc_new_flag()){	
			for(i=0;i<RC_CHANNELS;i++){
				printf("ch%d %0.2f  ", i+1, get_rc_channel(i+1));
			}
		}
		else{
			printf("No New Radio Packets     ");
		}
		fflush(stdout);
		usleep(100000);
	}
	return 0;
}