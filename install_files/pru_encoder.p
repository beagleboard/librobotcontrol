/*
Copyright (c) 2015, James Strawson
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*/

// PRU setup definitions
#define CONST_PRUCFG         C4
#define CONST_PRUSHAREDRAM   C28
#define PRU0_CTRL            0x22000
#define PRU1_CTRL            0x24000
#define CTPPR0               0x28
#define OWN_RAM              0x000
#define OTHER_RAM            0x020
#define SHARED_RAM           0x100


// Encoder counting definitions
#define CHA r31.t14	// these pin definitions are specific to SD-101D Robotics Cape
#define CHB r31.t15
#define OLD r0
#define CHA_OLD r0.t14 // keep last known values of chA and B in memory
#define CHB_OLD r0.t15 //
//#define CNT_OFFSET 32
#define CNT_OFFSET 32


.origin 0
.entrypoint START

.macro increment
	LBCO	r2, CONST_PRUSHAREDRAM, CNT_OFFSET, 4	// load existing counter from shared memory
	ADD 	r2, r2, 1		// increment
	SBCO	r2, CONST_PRUSHAREDRAM, CNT_OFFSET, 4	// write to shared memory
.endm

.macro decrement
	LBCO	r2, CONST_PRUSHAREDRAM, CNT_OFFSET, 4	// load existing counter from shared memory
	SUB 	r2, r2, 1		// subtract 1
	SBCO	r2, CONST_PRUSHAREDRAM, CNT_OFFSET, 4	// write to shared memory
.endm




START:
	LBCO	r0, CONST_PRUCFG, 4, 4		// Enable OCP master port
	CLR 	r0, r0, 4					// Clear SYSCFG[STANDBY_INIT] to enable OCP master port
	SBCO	r0, CONST_PRUCFG, 4, 4
	MOV		r0, 0x00000120				// Configure the programmable pointer register for PRU0 by setting c28_pointer[15:0]
	MOV     r0, SHARED_RAM              // Set C28 to point to shared RAM
	MOV     r1, PRU0_CTRL + CTPPR0		// Note we use beginning of shared ram unlike example which
	SBBO    r0, r1, 0, 4				// has arbitrary 2048 offset
	
// initialize by setting current state of two channels		
	MOV 	OLD, r31				
	zero	&r2, 4
	SBCO	r2, CONST_PRUSHAREDRAM, CNT_OFFSET, 4	// write 0 to shared memory
	
	
	
// CHECKPINS here forever looking for pin changes
CHECKPINS:
	XOR r1, OLD, r31
//	increment
//	MOV 	r0, r31
//	SBCO	r0, CONST_PRUSHAREDRAM, CNT_OFFSET, 4	// write to shared memory
	QBBS A_CHANGED, r1.t14	// Branch if CHA has toggled
	QBBS B_CHANGED, r1.t15	// Branch if CHB has toggled
	QBA CHECKPINS
	
A_CHANGED:
	MOV OLD, r31		// update old value now that something changed
	QBBC A_FELL,  CHA 	// Branch if CHA has fallen
	QBA  A_ROSE, 	 	// Branch if CHA has risen
	
B_CHANGED:
	MOV OLD, r31		// update old value now that something changed
	QBBC B_FELL,  CHB 	// Branch if CHB has toggled
	QBA B_ROSE	
	
	
	
A_FELL:							// CHA has fallen, check CHB
	QBBC B_LOW, CHB				// if CHB is clear (low) jump to B_LOW
	decrement					// CHB must be high, so decrement counter
	QBA CHECKPINS				// jump back to main CHECKPINS
B_LOW:							// CHB is low, increment counter
	increment
	QBA CHECKPINS				// jump back to the main CHECKPINS
	

A_ROSE:							// CHA has risen, check CHB
	QBBS B_HIGH, CHB			// if CHB is set jump to B_HIGH
	decrement
	QBA CHECKPINS				// jump back to main CHECKPINS
B_HIGH:							// CHB is high, increment counter
	increment
	QBA CHECKPINS				// jump back to the main CHECKPINS
	
	
B_FELL:							// CHB has fallen, check CHA
	QBBC A_LOW, CHA				// if CHA is clear (low) jump to A_LOW
	increment
	QBA CHECKPINS				// jump back to main CHECKPINS
A_LOW:							// CHA is low, decrement counter
	decrement
	QBA CHECKPINS				// jump back to the main CHECKPINS
	

B_ROSE:							// CHB has risen, check CHB
	QBBS A_LOW, CHA				// if CHA is clear (low) jump to A_LOW
	increment
	QBA CHECKPINS				// jump back to main CHECKPINS
A_HIGH:							// CHB is low, increment counter
	decrement
	QBA CHECKPINS				// jump back to the main CHECKPINS
	

		
	HALT	// we should never actually get here
	