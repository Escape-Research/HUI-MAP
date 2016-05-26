
;
; file: handleCS.s
;
.global _handleCS

_handleCS:
    
    ; Make sure that we disable nested interruprs
    
    bset LATC, #15
    bclr LATC, #15
    
    bra _handleCS
    
    return
.end

