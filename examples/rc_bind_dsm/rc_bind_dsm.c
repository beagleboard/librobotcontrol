/*******************************************************************************
* rc_bind_dsm.c
*
* routine to put a DSM2/DSMX satellite receiver into pairing mode
* James Strawson 2014
*
* DSM satellite receivers are put into bind mode by sending them a sequence of
* pulses right after it receives power and starts up. This program puts the 
* normally UART signal pin into GPIO pulldown mode temporarily, detects when the 
* user unplugs and plugs back in the receiver, then sends the binding pulses. 
*
* the number of pulses dictates the mode the satellite receiver will request
* the transmitter to use. The transmitter may bind but use a different mode.
* I suggest configuring your radio to use DSMX 11ms fast mode if it allows that.
*
* 2048 & 1024 indicates 10 or 11 bit resolution.
* 11ms & 22ms indicates the time period between the transmitter sending frames.
* 11ms is required for transmitters with 8 or more channels.
* 
* Testing done with DX7s, DX6i, DX8, and Orange T-SIX
* 
* Table of Bind Modes
*  pulses      mode        
*   3      DSM2 1024/22ms 
*   5  	DSM2 2048/11ms
*   7  	DSMX 1024/22ms: 
*   9  	DSMx 2048/11ms: 
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

// just run the bind function
int main(){
	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}
	// run the built-in bind routine
	rc_bind_dsm();
	// all done, cleanup
	rc_cleanup();
	return 0;
}
