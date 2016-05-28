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
    nop
    
    ; disable the output
    setm TRISB

    ; ********************************
    ; * End timing Critical section *
    ; ********************************
    
    ; prepare the next byte for output
    push.s
    
    ; are we on the "high" byte?
    mov _g_bOutput2ndByte
    ;mov _g_bOutput2ndByte, w13
    ;cp0 [w13]
    bra nz, its_not_zero
    
its_zero:
    
    ; we need to process the "high" byte
    
    mov _g_nextOutput, w12
    mov [w12], w13
    
    and #0b1100000000, w13
    lsr w13, #6, w12
    
    mov w12, LATB
    
    mov _g_bOutput2ndByte, w13
    mov #1, w12
    mov w12, [w13]
        
its_not_zero:
    
    ; we need to process the "low" byte
    
    mov _g_nextOutput, w12
    mov [w12], w13
    
    and #0b0011111111, w13
    rlnc w13, w12
    rlnc w12, w13
    mov w13, LATB
    
    mov _g_bOutput2ndByte, w13    
    mov #0, w12
    mov w12, [w13]

    pop.s
    
exit_int0:
    
    bclr IFS0, #INT0IF	; clear the interrupt flag
    ;pop.s
    retfie		; and return from interrupt

.end

