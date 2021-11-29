;*
;* Copyright (C) 2016 Zubeen Tolani <ZeekHuge - zeekhuge@gmail.com>
;*
;* This file is as an example to show how to develope
;* and compile inline assembly code for PRUs
;*
;* From Jason: note, this is not "inline" assembly. This is just calling
;* assembly from a C function. Also, does it make any sense to hold a
;* copyright over a program by simply providing an example that is really
;* very much known to those practiced in the art? How about a thanks in
;* the comments rather than a copyright?
;*
;* This program is free software; you can redistribute it and/or modify
;* it under the terms of the GNU General Public License version 2 as
;* published by the Free Software Foundation.


	.cdecls "main_pru.c"

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


; these pin definitions are specific to Robotics Cape rev B and BB AI-64
	.asg    r30.t5,		CH1BIT	; P8_04, AC29
	.asg    r30.t13,	CH2BIT	; P9_36B, AH29
	.asg    r30.t16,	CH3BIT	; P8_12, AH28
	.asg	r30.t17,	CH4BIT	; P8_11, AB24

	.asg    C4,     CONST_SYSCFG
	.asg    C28,    CONST_PRUSHAREDRAM

	.asg	0x22000,    PRU0_CTRL
	.asg    0x24000,    PRU1_CTRL       ; page 19
	.asg    0x28,       CTPPR0          ; page 75

	.asg	0x000,	OWN_RAM
	.asg	0x020,	OTHER_RAM
	.asg    0x100,	SHARED_RAM       ; This is so prudebug can find it.

	LBCO	&r0, CONST_SYSCFG, 4, 4		; Enable OCP master port
	CLR 	r0, r0, 4			; Clear SYSCFG[STANDBY_INIT] to enable OCP master port
	SBCO	&r0, CONST_SYSCFG, 4, 4

; Configure the programmable pointer register for PRU0 by setting c28_pointer[15:0]
	LDI     r0, SHARED_RAM			; Set C28 to point to shared RAM
	LDI32   r1, PRU1_CTRL + CTPPR0		; Note we use beginning of shared ram unlike example which
	SBBO    &r0, r1, 0, 4			;  page 25

	LDI 	r0, 0x0				; clear internal counters
	LDI 	r1, 0x0
	LDI 	r2, 0x0
	LDI 	r3, 0x0
	LDI 	r4, 0x0
	LDI 	r30, 0x0			; turn off GPIO outputs

; Beginning of loop, should always take 27??? instructions to complete, plus reload time between starts
CH1:
	QBLT	SET1, r1, r0			; if timer (r0) < CH1 (r1) pulse duration, jump to set
	CLR	r30, CH1BIT			; clear the channel
	QBA	CH2				; test next channel
SET1:
	SET	r30, CH1BIT			; set the channel
CH2:
	QBLT	SET2, r2, r0			; if timer (r0) < CH2 (r2) pulse duration, jump to set
	CLR	r30, CH2BIT			; clear the channel
	QBA	CH3				; test next channel
SET2:
	SET	r30, CH2BIT			; set the channel
CH3:
	QBLT	SET3, r3, r0			; if timer (r0) < CH3 (r3) pulse duration, jump to set
	CLR	r30, CH3BIT			; clear the channel
	QBA	CH4				; test next channel
SET3:
	SET	r30, CH3BIT			; set the channel
CH4:
	QBLT	SET4, r4, r0			; if timer (r0) < CH4 (r4) pulse duration, jump to set
	CLR	r30, CH4BIT			; clear the channel
	QBA	DUTY				; test next channel
SET4:
	SET	r30, CH4BIT			; set the channel
DUTY:
	QBEQ	RELOAD, r0, 0			; if done, jump to reload
	SUB	r0, r0, 1			; decrement timer
	QBA	CH1				; return to beginning of loop
RELOAD:
	LBCO	&r0, CONST_PRUSHAREDRAM, 0, 20	; load r0-r5 with values from shared memory
	QBA	CH1				; return to beginning of loop

