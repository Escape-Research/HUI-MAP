
;
; file: handleCS.s
;
.global _handleCS

_handleCS:
    
    ; Make sure that we disable nested interruprs
 
    bset LATB, #2
    bclr LATB, #2
    
    bra _handleCS
    
    return
.end

