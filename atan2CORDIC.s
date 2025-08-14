	; constant definitions
	.equ 	NEG_PI_BY_2,	0xC000
	.equ	PI_BY_2,		0x3FFF
	.equ	PI,				0x7FFF
	.equ	NEG_PI,			0x8000
	.equ	ACCUM_PHASE, w7
	
; in this table are the values of atan(n) scaled into the range 0 to PI
; and then converted into fractional format
	.section .data
CORDIC_DATA:
	.word 0x2000	; atan(1.0) / PI
	.word 0x12E4	; atan(0.5) / PI
	.word 0x09FB	; atan(0.25) / PI
	.word 0x0511	; atan(0.125) / PI
	.word 0x028B	; atan(0.0625) / PI
	.word 0x0146	; etc
	.word 0x00A3 
	.word 0x0051
	.word 0x0029
	.word 0x0014
	.word 0x000A
	.word 0x0005
	.word 0x0003
	.word 0x0001
	.word 0x0001
	.word 0x0000

	.section .text
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;	_atan2CORDIC: Perform an inverse tangent operation using
;	CORDIC iterative approximation. The parameters are assumed
;	to be dsPIC fractionals and the return value is a dsPIC fractional
;	with a value from -PI to PI scaled into the range -1.0 to 0.999
;	(to allow full scale use of the dsPIC fractional type)
;
;	On entry:
;	W0 = q
;	W1 = i
;	On Exit:
;	W0 = atan2(q, i)
;   Uses:
;	ACCA = rotated I value during calculation
;	ACCB = rotated Q value during calculation
;	1 word on stack to store temporary I value
;	w4 = current phase offset
;	w5 = K (series divider 1.0, 0.5, 0.25 etc)
;	w6 = Q value (16 bit version of ACCB)
;	w7 = Accumulated phase
;	w8 = Pointer to CORDIC table
;	w9 = Pointer to storage word on stack
;
;	A conventional CORDIC routine can achieve its result by
; 	just using shifts and adds. However this routine takes
;	advantage of the DSP mac and msc to achieve the same result
;	it also relies on the single cycle preload architecture to
;	get parameters for the next operation into place.
;   There are bounds checking for y = -1 and x = -1 other than
;	that the data is assumed to be correctly scaled and normalized
;	in the range -1.0 to 0.999 (0x8000 to 7FFF)
;	Apart from at the extrema the results agree with those returned
; 	by the standard atan2 library routine to within 0.05 degrees
;
;	Total cycle count is 160 cycles for any rational value
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	.global _atan2CORDIC
		
_atan2CORDIC:
	lnk		#0x02		; reserve 2 bytes for storage
	push	w8
	push	w9
	push	CORCON
	
	; check for values that cause errors 
	; due to asymptotic atan behaviour
	mov		#0x8000, w8
	cp		w0, w8			; Q = -1.0 ?
	bra		NZ, checkI
	mov		#NEG_PI_BY_2, w0
	bra		exitCORDICRoutine
checkI:
	cp		w1, w8			; I = -1.0
	bra		NZ, mainCORDICRoutine
	mov		#PI, w0
	bra		exitCORDICRoutine
mainCORDICRoutine:

	; set w9 to point to the reserved 2 byte space
	; this can then be used to preload w6 in the dsp MACs
	mov		w14, w9		
	; ACCUM_PHASE (w7) is the total phase angle calculated
	clr		ACCUM_PHASE	

	; adjust q and i to be in quadrant I
	cp0		w1
	bra		NN, setupIter
	mov		w1, [w9]		; w2 = temporary I
	cp0		w0
	bra		LE, quadIII
	mov		w0, w1
	neg		[w9], w0
	mov		#NEG_PI_BY_2, ACCUM_PHASE
	bra		setupIter
quadIII:
	neg		w0, w1
	mov		[w9], w0
	mov		#PI_BY_2, ACCUM_PHASE

setupIter:
	; set ACCA and ACCB to equal I and Q	
	lac		w0, #1, B
	lac		w1, #1, A
	
	mov		#CORDIC_DATA, w8	; w8 points to CORDIC data table
	mov		#0x7FFF, w5			; w5 = K = 1.0
	
	do		#9, endCORDICRoutine
	sac.r	a, [w9]				; put I onto local stack
	sac.r	b, w6				; w6 = Q
	cp0		w6					; if Q < 0 goto rotate positive
	bra		N, rotate_pos
rotate_neg:
	mac		w5*w6, a			; I = I + Q * K, w6 = temp I
	mov		[w9], w6
	msc		w5*w6, b			; Q = Q - oldI * K
	mov		[w8++], w4
	subbr	w4, ACCUM_PHASE, ACCUM_PHASE
	bra		endCORDICRoutine
rotate_pos:
	msc		w5*w6, a			; I = I - Q * K, w6 = temp I
	mov		[w9], w6
	mac		w5*w6, b			; Q = Q + oldI * K
	mov		[w8++], w4
	add 	w4, ACCUM_PHASE, ACCUM_PHASE
	
endCORDICRoutine:
	lsr		w5, w5				; K = K / 2

	neg		ACCUM_PHASE, w0		; reverse the sign
exitCORDICRoutine:	
	pop 	CORCON
	pop		w9
	pop 	w8
	ulnk
	return

	.end
	
	
