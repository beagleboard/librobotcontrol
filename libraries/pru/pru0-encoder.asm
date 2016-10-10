; Converted to clpru by Mark A. Yoder
;	26-July-2016

; pru_0_encoder.p
; assembly code for counting quadrature encoder signal on inputs R31_14 & 15

; Copyright (c) 2015, James Strawson
; All rights reserved.

; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions are met:

; 1. Redistributions of source code must retain the above copyright notice, this
;   list of conditions and the following disclaimer. 
; 2. Redistributions in binary form must reproduce the above copyright notice,
;   this list of conditions and the following disclaimer in the documentation
;   and/or other materials provided with the distribution.

; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
; ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
; WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
; DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
; ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
; (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
; ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
; SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

; The views and conclusions contained in the software and documentation are those
; of the authors and should not be interpreted as representing official policies, 
; either expressed or implied, of the FreeBSD Project.

	.cdecls "robotics_cape_defs.h"

; PRU setup definitions
	; .asg    C4,     CONST_SYSCFG         
	.asg    C28,    CONST_PRUSHAREDRAM   
 
	.asg	0x22000,	PRU0_CTRL
	.asg    0x24000,    PRU1_CTRL       ; page 19
	; .asg    0x28,       CTPPR0          ; page 75
 
	.asg	0x000,	OWN_RAM
	.asg	0x020,	OTHER_RAM
	.asg    0x100,	SHARED_RAM       ; This is so prudebug can find it.

; Encoder counting definitions
; these pin definitions are specific to SD-101D Robotics Cape
	.asg	r31,		CH		; CHA: P8_16, CHB: P8_15
	.asg	r0,			OLD		; keep last known values of chA and B in memory
	.asg	r0.t14,		OLD_A
	.asg	r0.t15,		OLD_B
	.asg	r1,			EXOR	; place to store the XOR of old with new AB vals
	.asg	14,			A
	.asg	15,			B

increment	.macro 
	LBCO	&r2, CONST_PRUSHAREDRAM, CNT_OFFSET, 4	; load existing counter from shared memory
	ADD 	r2, r2, 1		; increment
	SBCO	&r2, CONST_PRUSHAREDRAM, CNT_OFFSET, 4	; write to shared memory
	QBA CHECKPINS				; jump back to main CHECKPINS
	.endm

decrement	.macro
	LBCO	&r2, CONST_PRUSHAREDRAM, CNT_OFFSET, 4	; load existing counter from shared memory
	SUB 	r2, r2, 1		; subtract 1
	SBCO	&r2, CONST_PRUSHAREDRAM, CNT_OFFSET, 4	; write to shared memory
	QBA CHECKPINS				;/ jump back to main CHECKPINS
	.endm

	.clink
	.global start
start:
; 	LBCO	&r0, CONST_SYSCFG, 4, 4		; Enable OCP master port
; 	CLR 	r0, r0, 4					; Clear SYSCFG[STANDBY_INIT] to enable OCP master port
; 	SBCO	&r0, CONST_SYSCFG, 4, 4
; Configure the programmable pointer register for PRU0 by setting c28_pointer[15:0]
	; LDI     r0, SHARED_RAM              ; Set C28 to point to shared RAM
	; LDI32   r1, PRU0_CTRL + CTPPR0		; Note we use beginning of shared ram unlike example which
	; SBBO    &r0, r1, 0, 4				; has arbitrary 2048 offset
	
; initialize by setting current state of two channels		
	MOV 	OLD, r31				
	zero	&r2, 4
	SBCO	&r2, CONST_PRUSHAREDRAM, CNT_OFFSET, 4	; write 0 to shared memory
	
; CHECKPINS here forever looking for pin changes
CHECKPINS:
	XOR EXOR, OLD, r31
	QBBS A_CHANGED, EXOR, A	; Branch if CHA has toggled
	QBBS B_CHANGED, EXOR, B ; Branch if CHB has toggled
	QBA CHECKPINS
	
	
A_CHANGED:
	MOV OLD, r31			; update old value now that something changed
	QBBC A_FELL,  CH, A 		; Branch if CHA has fallen
	QBBS DECREMENT, CH, B		; A has risen, if B is HIGH, decrement
	increment				; otherwise increment
	
B_CHANGED:
	MOV OLD, r31			; update old value now that something changed
	QBBC B_FELL,  CH, B 		; Branch if CHB has fallen
	QBBS INCREMENT, CH, A		; ch B has risen, if A is HIGH, increment
	decrement				; otherwise decrement
	
A_FELL:						; CHA has fallen, check CHB
	QBBC DECREMENT, CH, B		; if CHB is clear (low) decrement
	increment				; CHB must be high, so decrement counter
	
B_FELL:						; CHB has fallen, check CHA
	QBBC INCREMENT, CH, A		; if CHA is clear (low) decrement
	decrement

	
DECREMENT:
	decrement

INCREMENT:
	increment

		
	HALT	; we should never actually get here
	