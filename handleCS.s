.include "p30f3013.inc"
;
; file: handleCS.s
;

    .global _asm_ProcessRDRequest

_asm_ProcessRDRequest:
    
    ; Temporarily disable interrupts
    disi #0x3FFF

    ; Store the current state of registers
    push.s
 
    ; Prepare the first output byte
    mov _g_nextOutput, w0    
    and #0b1111111100, w0
    
    ; Load the output latch
    mov w0, LATB
    
    ; *********************************
    ; * Begin timing Critical section *
    ; *********************************
       
wait_1:
    ; Is this a RD or a WR ?
    btsc PORTD, #8
    bra wait_1

    btsc PORTF, #6
    bra wait_1
    
    ; Ok, we are in a RD
    ; enable the output
    mov #0b0000000011, w0
    mov w0, TRISB
    
    ; Through verification with the logic analyzer and while using
    ; the internal FRC x 16 PLL with OSCTUN = 7 we don't need any
    ; extra wait time
    
    ; disable the output
    setm TRISB

    ; ********************************
    ; * End timing Critical section *
    ; ********************************

    ; we need to process the "high" (although it's the LSB) byte
    mov _g_nextOutput, w0    
    and #0b11, w0
    sl w0, #8, w0
   
    ; Load the output latch
    mov w0, LATB

wait_2:
    ; Is this a RD or a WR ?
    btsc PORTD, #8
    bra wait_2

    btsc PORTF, #6
    bra wait_2
    
    ; Ok, we are in the second RD (MSB)
    ; enable the output
    mov #0b0000000011, w0
    mov w0, TRISB

    ; See note above regarding the timing out the output
    
    ; disable the output
    setm TRISB    
       
    ; restore the prior state of registers
    pop.s

    ; re-enable interrupts
    clr DISICNT
    
    return
        
    .end

