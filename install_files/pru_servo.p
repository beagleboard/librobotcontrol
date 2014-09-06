/*
Copyright (c) 2014, James Strawson
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

// these pin definitions are specific to SD-101C Robotics Cape
#define CH1BIT r30.t8
#define CH2BIT r30.t10
#define CH3BIT r30.t9
#define CH4BIT r30.t11
#define CH5BIT r30.t6
#define CH6BIT r30.t7
#define CH7BIT r30.t4
#define CH8BIT r30.t5

#define CONST_PRUCFG         C4
#define CONST_PRUSHAREDRAM   C28
 
#define PRU0_CTRL            0x22000
#define PRU1_CTRL            0x24000
 
#define CTPPR0               0x28
 
#define OWN_RAM              0x000
#define OTHER_RAM            0x020
#define SHARED_RAM           0x100


.origin 0
.entrypoint START

START:
	LBCO	r0, CONST_PRUCFG, 4, 4		// Enable OCP master port
	CLR 	r0, r0, 4					// Clear SYSCFG[STANDBY_INIT] to enable OCP master port
	SBCO	r0, CONST_PRUCFG, 4, 4
	MOV		r0, 0x00000120				// Configure the programmable pointer register for PRU0 by setting c28_pointer[15:0]
	MOV     r0, SHARED_RAM              // Set C28 to point to shared RAM
	MOV     r1, PRU1_CTRL + CTPPR0		// Note we use beginning of shared ram unlike example which
	SBBO    r0, r1, 0, 4				// has arbitrary 2048 offset
	MOV		r9, 0x00000000				// erase r9 to use to use later
	
// Beginning of loop, should always take 48 instructions to complete
CH1:			
		QBEQ	CLR1, r0, 0						// If timer is 0, jump to clear channel
		SET		CH1BIT							// If non-zero turn on the corresponding channel
		SUB		r0, r0, 1						// Subtract one from timer
		MOV		r9, 0x00000000					// waste a cycle for timing
		SBCO	r9, CONST_PRUSHAREDRAM, 0, 4	// write 0 to shared memory
CH2:
		QBEQ	CLR2, r1, 0					
		SET		CH2BIT						
		SUB		r1, r1, 1
		MOV		r9, 0x00000000					
		SBCO	r9, CONST_PRUSHAREDRAM, 4, 4
CH3:
		QBEQ	CLR3, r2, 0					
		SET		CH3BIT						
		SUB		r2, r2, 1
		MOV		r9, 0x00000000
		SBCO	r9, CONST_PRUSHAREDRAM, 8, 4
CH4:
		QBEQ	CLR4, r3, 0					
		SET		CH4BIT						
		SUB		r3, r3, 1
		MOV		r9, 0x00000000
		SBCO	r9, CONST_PRUSHAREDRAM, 12, 4
CH5:
		QBEQ	CLR5, r4, 0					
		SET		CH5BIT					
		SUB		r4, r4, 1
		MOV		r9, 0x00000000
		SBCO	r9, CONST_PRUSHAREDRAM, 16, 4
CH6:
		QBEQ	CLR6, r5, 0					
		SET		CH6BIT						
		SUB		r5, r5, 1
		MOV		r9, 0x00000000
		SBCO	r9, CONST_PRUSHAREDRAM, 20, 4
CH7:
		QBEQ	CLR7, r6, 0					
		SET		CH7BIT						
		SUB		r6, r6, 1
		MOV		r9, 0x00000000
		SBCO	r9, CONST_PRUSHAREDRAM, 24, 4
CH8:
		QBEQ	CLR8, r7, 0					
		SET		CH8BIT						
		SUB		r7, r7, 1
		MOV		r9, 0x00000000
		SBCO	r9, CONST_PRUSHAREDRAM, 28, 4
		QBA		CH1								// return to beginning of loop

		
		
		
CLR1:
		CLR		CH1BIT							// turn off the corresponding channel
		LBCO	r0, CONST_PRUSHAREDRAM, 0, 4	// Load new timer register
		QBA		CH2								// go back to check next channel
CLR2:
		CLR		CH2BIT	
		LBCO	r1, CONST_PRUSHAREDRAM, 4, 4	
		QBA		CH3
CLR3:
		CLR		CH3BIT
		LBCO	r2, CONST_PRUSHAREDRAM, 8, 4		
		QBA		CH4
CLR4:
		CLR		CH4BIT	
		LBCO	r3, CONST_PRUSHAREDRAM, 12, 4		
		QBA		CH5
CLR5:
		CLR		CH5BIT	
		LBCO	r4, CONST_PRUSHAREDRAM, 16, 4
		QBA		CH6
CLR6:
		CLR		CH6BIT
		LBCO	r5, CONST_PRUSHAREDRAM, 20, 4		
		QBA		CH7
CLR7:
		CLR		CH7BIT
		LBCO	r6, CONST_PRUSHAREDRAM, 24, 4
		QBA		CH8
CLR8:
		CLR		CH8BIT		
		LBCO	r7, CONST_PRUSHAREDRAM, 28, 4
		QBA		CH1								// return to beginning of loop
		
		
		
	HALT	// we should never actually get here
	