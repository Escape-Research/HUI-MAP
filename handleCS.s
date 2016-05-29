.include "p30f3013.inc"
;
; file: handleCS.s
;

    .global _asm_ProcessRDRequest

_asm_ProcessRDRequest:
    
    ; *********************************
    ; * Begin timing Critical section *
    ; *********************************
    
    ; Temporarily disable interrupts
    disi #100
    
s_wait_1:
    ; Is this a RD or a WR ?
    btsc PORTD, #8
    bra s_wait_1

    btsc PORTF, #6
    ;bra s_proceed1
    bra s_wait_1
    
s_proceed1:    
    
    ; Ok, we are in a RD
    ; enable the output
    mov #0b0000000011, w13
    mov w13, TRISB
    ;nop
    ;nop    
    
    ; disable the output
    setm TRISB

    ; ********************************
    ; * End timing Critical section *
    ; ********************************

    push.s
    
    ; we need to process the "high" byte
    
    mov _g_nextOutput, w13
    
    and #0b11, w13
    rlnc w13, w12
    rlnc w12, w13
    rlnc w13, w12
    rlnc w12, w13
    rlnc w13, w12
    rlnc w12, w13
    rlnc w13, w12
    rlnc w12, w13
    
    mov w13, LATB

s_wait_2:
    ; Is this a RD or a WR ?
    btsc PORTD, #8
    bra s_wait_2

    btsc PORTF, #6
    ;bra s_proceed1
    bra s_wait_2
    
s_proceed2:    

    ; Ok, we are in a RD
    ; enable the output
    mov #0b0000000011, w13
    mov w13, TRISB
    ;nop
    ;nop    
    
    ; disable the output
    setm TRISB    
    
    pop.s
    
s_exit_int0:
    
    ;bclr IFS0, #INT0IF	; clear the interrupt flag
    return		; and return from interrupt
        
    .end

