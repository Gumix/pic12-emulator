	org 0
_temp	EQU 0x22
init:
	movlw	D'250'
	movwf	_temp
main:
	incfsz	_temp, f
	goto	main
	nop
	goto	init	
zero:
	nop
	END