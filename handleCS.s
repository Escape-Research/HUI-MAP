.include "p30f3013.inc"
;
; file: handleCS.s
;
.global __INT0Interrupt

__INT0Interrupt:
    
    ; *********************************
    ; * Begin timing Critical section *
    ; *********************************
    
    ; Is this a RD or a WR ?
    btsc PORTD, #8
    bra exit_int0
    
    ; Ok, we are in a RD
    ; enable the output
    mov #0b0000000011, w13
    mov w13, TRISB
    nop
    ;nop
    
    ; disable the output
    setm TRISB

    ; ********************************
    ; * End timing Critical section *
    ; ********************************

    push.s
    
    ; prepare the next byte for output
    
    ; are we on the "high" byte?
    mov _g_bOutput2ndByte
    bra nz, its_not_zero
    
its_zero:
    
    ; we need to process the "high" byte
    
    mov _g_nextOutput, w13
    
    and #0b11, w13
    rlnc w13, w12
    rlnc w12, w13
    rlnc w13, w12
    rlnc w12, w13
    rlnc w13, w12
    rlnc w12, w13
    ;lsr w13, #6, w12
    
    mov w13, LATB

    mov #1, w12
    mov w12, _g_bOutput2ndByte
    
    bra done_with_output
        
its_not_zero:
    
    ; we need to process the "low" byte
    
    mov _g_nextOutput, w13
    
    and #0b1111111100, w13
    mov w13, LATB
    
    mov #0, w12
    mov w12, _g_bOutput2ndByte

done_with_output:
    
    pop.s
    
exit_int0:
    
    bclr IFS0, #INT0IF	; clear the interrupt flag
    retfie		; and return from interrupt
    
.end

