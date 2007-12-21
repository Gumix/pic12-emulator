	include	"p12f629.inc"
	org 0
_temp	EQU 0x22
init:
	clrf	_temp
	bsf		_temp, 1
main:
	rlf		_temp, f
	btfss	STATUS, C
	goto	main
	nop
	goto	init	
zero:
	nop
	END