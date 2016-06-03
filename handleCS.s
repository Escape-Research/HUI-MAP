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
    
    ; copy w0 to w1
    mov w0, w1
    
    ; load w3 with the enable mask for port B
    mov #0b0000000011, w3
    
    ; Prepare the first and second output bytes
    ; First byte
    and #0b1111111100, w0

    ; Prepare the second byte
    and #0b11, w1    
    ; we are shifting the two LSBits 8 places since we are using PORTB 2..9
    ; as the D0..7 data bus integration.
    sl w1, #8, w1

    ; FIRST BYTE
    ; ----------
    
    ; Load the output latch for the first (MSB) byte
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
    mov w3, TRISB

    nop
    nop
    
    ; Through verification with the logic analyzer and while using
    ; the internal FRC x 16 PLL with OSCTUN = 7 we don't need any
    ; extra wait time
    
    ; disable the output
    setm TRISB

    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    
    ; SECOND BYTE
    ; -----------
        
    ; Load the output latch for the second (LSB) byte
    mov w1, LATB

wait_2:
    ; Is this a RD or a WR ?
    btsc PORTD, #8
    bra wait_2

    btsc PORTF, #6
    bra wait_2
    
    ; Ok, we are in the second RD (MSB)
    ; enable the output
    mov w3, TRISB

    nop
    nop
    
    ; See note above regarding the timing out the output
    
    ; disable the output
    setm TRISB    
       
    ; ********************************
    ; * End timing Critical section *
    ; ********************************

    ; restore the prior state of registers
    pop.s

    ; re-enable interrupts
    clr DISICNT
    
    return
        
    .end

