	org 0
_temp	EQU 0x22
init:
	movlw	D'250'
	xorlw	D'255'
	movwf	_temp
main:
	decfsz	_temp, f
	goto	main
	nop
	goto	init	
zero:
	nop
	END