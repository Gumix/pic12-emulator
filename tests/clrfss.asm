	include	"p12f629.inc"
	org 0
_temp	EQU 0x22
init:
	movlw	D'250'
	movwf	_temp
main:
	clrf	_temp
	btfss	STATUS, Z
	goto	main
	nop
	goto	init	
zero:
	nop
	END