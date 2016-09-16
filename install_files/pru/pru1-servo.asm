;*
;* Copyright (C) 2016 Zubeen Tolani <ZeekHuge - zeekhuge@gmail.com>
;*
;* This file is as an example to show how to develope
;* and compile inline assembly code for PRUs
;*
;* This program is free software; you can redistribute it and/or modify
;* it under the terms of the GNU General Public License version 2 as
;* published by the Free Software Foundation.


	.cdecls "main_pru1.c"

DELAY	.macro time, reg
	LDI32	reg, time
	QBEQ	$E?, reg, 0
$M?:	SUB	reg, reg, 1
	QBNE	$M?, reg, 0
$E?:	
	.endm
	

	.clink
	.global start
start:
	LDI 	R30, 0xFFFF
	DELAY 	10000000, r11
	LDI		R30, 0x0000
	DELAY 	10000000, r11
; 	JMP	start	

; 	HALT


; these pin definitions are specific to SD-101C Robotics Cape
    .asg    r30.t8,     CH1BIT  ; P8_27
	.asg    r30.t10,    CH2BIT	; P8_28
	.asg    r30.t9,     CH3BIT	; P8_29
	.asg	r30.t11,	CH4BIT	; P8_30
	.asg	r30.t6,		CH5BIT	; P8_39
	.asg	r30.t7,		CH6BIT	; P8_40
	.asg	r30.t4,		CH7BIT	; P8_41
	.asg	r30.t5,		CH8BIT	; P8_42

	.asg    C4,     CONST_SYSCFG         
	.asg    C28,    CONST_PRUSHAREDRAM   
 
	.asg	0x22000,	PRU0_CTRL
	.asg    0x24000,    PRU1_CTRL       ; page 19
	.asg    0x28,       CTPPR0          ; page 75
 
	.asg	0x000,	OWN_RAM
	.asg	0x020,	OTHER_RAM
	.asg    0x100,	SHARED_RAM       ; This is so prudebug can find it.

	LBCO	&r0, CONST_SYSCFG, 4, 4		; Enable OCP master port
	CLR 	r0, r0, 4					; Clear SYSCFG[STANDBY_INIT] to enable OCP master port
	SBCO	&r0, CONST_SYSCFG, 4, 4
	
; Configure the programmable pointer register for PRU0 by setting c28_pointer[15:0]
	LDI     r0, SHARED_RAM              ; Set C28 to point to shared RAM
	LDI32   r1, PRU1_CTRL + CTPPR0		; Note we use beginning of shared ram unlike example which
	SBBO    &r0, r1, 0, 4				;  page 25
	
	LDI		r9, 0x0				; erase r9 to use to use later
	
	LDI 	r0, 0x0				; clear internal counters
	LDI 	r1, 0x0	
	LDI 	r2, 0x0
	LDI 	r3, 0x0
	LDI 	r4, 0x0
	LDI 	r5, 0x0
	LDI 	r6, 0x0
	LDI32 	r7, 0x0
	LDI 	r30, 0x0				; turn off GPIO outputs
	

; Beginning of loop, should always take 48 instructions to complete
CH1:			
	QBEQ	CLR1, r0, 0						; If timer is 0, jump to clear channel
	SET		r30, CH1BIT						; If non-zero turn on the corresponding channel
	SUB		r0, r0, 1						; Subtract one from timer
	CLR		r9, r9.t1						; waste a cycle for timing
	SBCO	&r9, CONST_PRUSHAREDRAM, 0, 4	; write 0 to shared memory
CH2:			
	QBEQ	CLR2, r1, 0
	SET		r30, CH2BIT
	SUB		r1, r1, 1
	CLR		r9, r9.t1
	SBCO	&r9, CONST_PRUSHAREDRAM, 4, 4
CH3:			
	QBEQ	CLR3, r2, 0
	SET		r30, CH3BIT
	SUB		r2, r2, 1
	CLR		r9, r9.t1
	SBCO	&r9, CONST_PRUSHAREDRAM, 8, 4
CH4:			
	QBEQ	CLR4, r3, 0
	SET		r30, CH4BIT
	SUB		r3, r3, 1
	CLR		r9, r9.t1
	SBCO	&r9, CONST_PRUSHAREDRAM, 12, 4
CH5:			
	QBEQ	CLR5, r4, 0
	SET		r30, CH5BIT
	SUB		r4, r4, 1
	CLR		r9, r9.t1
	SBCO	&r9, CONST_PRUSHAREDRAM, 16, 4
CH6:			
	QBEQ	CLR6, r5, 0
	SET		r30, CH6BIT
	SUB		r5, r5, 1
	CLR		r9, r9.t1
	SBCO	&r9, CONST_PRUSHAREDRAM, 20, 4
CH7:			
	QBEQ	CLR7, r6, 0
	SET		r30, CH7BIT
	SUB		r6, r6, 1
	CLR		r9, r9.t1
	SBCO	&r9, CONST_PRUSHAREDRAM, 24, 4
CH8:			
	QBEQ	CLR8, r7, 0
	SET		r30, CH8BIT
	SUB		r7, r7, 1
	SBCO	&r9, CONST_PRUSHAREDRAM, 28, 4

	QBA		CH1								; return to beginning of loop
	; no need to waste a cycle for timing here because of the QBA above
	
		
CLR1:
	CLR		r30, CH1BIT						; turn off the corresponding channel
	LBCO	&r0, CONST_PRUSHAREDRAM, 0, 4	; Load new timer register
	QBA		CH2
CLR2:
	CLR		r30, CH2BIT
	LBCO	&r1, CONST_PRUSHAREDRAM, 4, 4
	QBA		CH3
CLR3:
	CLR		r30, CH3BIT
	LBCO	&r2, CONST_PRUSHAREDRAM, 8, 4
	QBA		CH4
CLR4:
	CLR		r30, CH4BIT
	LBCO	&r3, CONST_PRUSHAREDRAM, 12, 4
	QBA		CH5
CLR5:
	CLR		r30, CH5BIT
	LBCO	&r4, CONST_PRUSHAREDRAM, 16, 4
	QBA		CH6
CLR6:
	CLR		r30, CH6BIT
	LBCO	&r5, CONST_PRUSHAREDRAM, 20, 4
	QBA		CH7
CLR7:
	CLR		r30, CH7BIT
	LBCO	&r6, CONST_PRUSHAREDRAM, 24, 4
	QBA		CH8
CLR8:
	CLR		r30, CH8BIT
	LBCO	&r7, CONST_PRUSHAREDRAM, 28, 4
	QBA		CH1								; return to beginning of loop
