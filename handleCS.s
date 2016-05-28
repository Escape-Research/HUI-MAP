.include "p30f3013.inc"
;
; file: handleCS.s
;
.global __INT0Interrupt

__INT0Interrupt:
    
    ; Is this a RD or a WR ?
    BTSC PORTD, #8
    bra checkForWR
    
    ; Ok, we are in a RD
    ; enable the output
    clr TRISB
    ;mov #0b0000000011, w0
    ;mov w0, TRISB
    
    ; wait an additional ~ 36ns
    nop
    
    ; disable the output
    setm TRISB
    ;mov #0b1111111111, w0
    ;mov w0, TRISB
    
    ; prepare the next byte for output
    ; TODO
    
    bra exit_int0
    
checkForWR:
 
    ; TODO
    
exit_int0:
    
    bclr IFS0, #INT0IF	; clear the interrupt flag
    retfie		; and return from interrupt

.end

