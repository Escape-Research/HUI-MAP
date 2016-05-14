;******************************************************************************
 ;*                                                                            *
 ;*  Project:     Papsi                                                        *
 ;*  Module:      rwflash.s                                                        *
 ;*  Author:      Christian Klugesherz                                               *
 ;*                                                                            *
 ;******************************************************************************
 
     .include "p30f3013.inc"
 
 ;******************************************************************************
 ; Global Declarations:
 ;******************************************************************************
        .global _asm_read16b_flash
        .global _asm_read16b_row_flash
        .global _asm_write16b_row_flash
 
 ;******************************************************************************
 ;Code Section in Program Memory
 ;******************************************************************************
        .text                         ; Start of Code section
 
 ;------------------------------------------------------------------------------
 ; Read 16bits in flash Memory Space
 ;; Inputs:
 ;;    w0,w1 = Program Memory Flash Address to Read: (24 bits) = w0(8bits)+w1(16bits)
 ;; Outputs: 
 ;;    w0 = Result copied to Data RAM (Low 16 bits) 
 ;; 
 ;; extern short asm_read16b_flash(char add_flash_ch, short add_flash_wl);
 ;------------------------------------------------------------------------------
 _asm_read16b_flash:
	push    TBLPAG
	mov     w0, TBLPAG
        tblrdl  [w1],w2
        mov     w2, w0
        pop     TBLPAG
        return 
            
 
 ;------------------------------------------------------------------------------
 ; Read 32 x 16bits in flash Memory Space
 ;; Inputs:
 ;;    w0,w1 = Begin of Program Memory Flash Address to Read: (24 bits) = w0(8bits)+w1(16bits)
 ;;       w2 = Begin of Data RAM address (Low 16 bits), result of flash Memory Space read
 ;; Outputs: 
 ;;    void
 ;; 
 ;; extern void    asm_read16b_row_flash(char add_flash_ch, short add_flash_wl, short &dataspace);
 ;------------------------------------------------------------------------------
 _asm_read16b_row_flash:
        push    TBLPAG
        mov     w0, TBLPAG
        mov     #32, w3 
        
 readNext:    
        tblrdl  [w1++],w4
        mov      w4,[w2++]
        dec     w3, w3
        bra     nz, readNext
        clr      w0
        pop     TBLPAG
        return 
 
 ;------------------------------------------------------------------------------
 ; Write 32 x 16bits in flash Memory Space
 ;; Inputs:
 ;;    w0,w1 = Start of Program Memory Flash Address to Write: (24 bits) = w0(8bits)+w1(16bits)
 ;;             ---> corresponds to Latch memory
 ;;       w2 = Start of Data RAM address (Low 16 bits), to copy to flash
 ;; Outputs: 
 ;;    void
 ;; 
 ;; extern void asm_write16b_row_flash(char add_flash_ch, short add_flash_wl, short &dataspace);
 ;; 
 ;------------------------------------------------------------------------------
 _asm_write16b_row_flash:
 
        push    TBLPAG
 
        mov     #32, w3             ; Assigns Number of 16bits word per Row
            
        mov     w0, TBLPAG            ; For latch loading and programming
        mov     W0, NVMADRU         ; For erase OR program written from TBLPAG
        mov     W1, NVMADR         
 
 Erase_flash:
        mov     #0x4041, W0         ; Enable Erase or Program
                                     ; PROGOP: 0x41-> Erase 1 row (32 instruction words) of program Flash
 
 EraseKey:
        DISI    #7                    ; Block all interrupts for next cycles
        mov     W0, NVMCON
        mov     #0x55, W0
        mov     W0, NVMKEY
        mov     #0xAA, W0
        mov     W0, NVMKEY
        bset     NVMCON, #WR            ; Start Erasing
        nop
        nop
 WaitErasing:
        btsc NVMCON, #WR            ; WR or WREN - Wait until operation is finished
        bra WaitErasing
 
 Write_latch:
        mov     #0x4001, W0            ; Enable Erase or Program
                                     ; PROGOP: 0x01-> Program 1 row (32 instruction words) of program Flash
 
 Write_latch_loop:
        tblwtl [W2++], [W1++]       ; Load low word to latch
        dec W3, W3                  ; Repeat until whole row is loaded
        bra NZ, Write_latch_loop
         
 
 WriteKey:
        DISI    #7                ; Block all interrupts for next cycles
        mov     W0, NVMCON
        mov     #0x55, W0
        mov     W0, NVMKEY
        mov     #0xAA, W0
        mov     W0, NVMKEY
        bset     NVMCON, #WR        ; Start Writing
        nop
        nop
 WaitWriting:
        btsc NVMCON, #WR        ; WR or WREN - Wait until operation is finished
        bra WaitWriting
 
 Write_Erase_end:
        pop     TBLPAG
        return
         
 
 ;--------End of All Code Sections ---------------------------------------------
 
 .end                              ; End of program code in this file
 
 





