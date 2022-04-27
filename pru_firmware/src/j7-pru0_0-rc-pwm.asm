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

; these pin definitions are specific to Robotics Cape rev B and BB AI-64
; HACK: TODO: grabbing values from R29 to set them in R30 all at once
	.asg    r29.t5,		CH1BIT	; P8_04, AC29
	.asg    r29.t13,	CH2BIT	; P9_36B, AH29
	.asg    r29.t16,	CH3BIT	; P8_12, AH28
	.asg	r29.t17,	CH4BIT	; P8_11, AB24

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
	LDI32   r1, PRU1_CTRL + CTPPR0		; Note we use beginning of shared ram
	SBBO    &r0, r1, 0, 4

	LDI 	r0, 0x0				; clear internal counters
	LDI 	r1, 0x0
	LDI 	r2, 0x0
	LDI 	r3, 0x0
	LDI 	r4, 0x0

; Beginning of loop
; PWM frequency is approximately 1/((mem[0]+1)*72ns), mem[0] must be > 0
; Duty cycle is roughly m[CH]/(m[0]+1) for a sufficiently large m[0]

; TODO: Does TDA4VM PRU have 4ns cycles?
RELOAD:
	LDI	r29, 0				; set new value (r29) to 0
	LBCO	&r0, CONST_PRUSHAREDRAM, 0, 20	; load r0-r4 with values from shared memory (7 cycles?)
CH1:
	QBGE	CLRCH1, r1, r0			; if timer (r0) > CH1 (r1) pulse duration, skip setting
	SET	r29, CH1BIT			; set channel 1 bit
CH2:
	QBGE	CLRCH2, r2, r0			; if timer (r0) > CH2 (r2) pulse duration, skip setting
	SET	r29, CH2BIT			; set channel 2 bit
CH3:
	QBGE	CLRCH3, r3, r0			; if timer (r0) > CH3 (r3) pulse duration, skip setting
	SET	r29, CH3BIT			; set channel 3 bit
CH4:
	QBGE	CLRCH4, r4, r0			; if timer (r0) > CH4 (r4) pulse duration, skip setting
	SET	r29, CH4BIT			; set channel 4 bit
UPDATE:
	MOV	r30, r29			; copy new value (r29) to output (r30)
	QBEQ	RELOAD, r0, 0			; reload when timer reaches 0
	SUB	r0, r0, 1			; decrement timer (r0)
	QBA	NEXT				; test channel pulse durations again
ENDLOOP:

; Extra cycles to balance out SET operation
CLRCH1:
	QBA	CH2
CLRCH2:
	QBA	CH3
CLRCH3:
	QBA	CH4
CLRCH4:
	QBA	UPDATE

; Extra cycles to balance out RELOAD operation
NEXT:
	NOP
	NOP
	NOP
	NOP
	NOP
	QBA	CH1
