/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #if defined(__dsPIC30F__)
        #include <p30Fxxxx.h>
    #endif
#endif

#include <stdint.h>          /* For uint32_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */
#include <stdlib.h>

#include <libpic30.h>

#include "user.h"            /* variables/params used by user.c               */

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

uint16_t getMap(char fader, uint16_t position)
{
    uint16_t lo_value = map_lo[fader][position];
    uint16_t hi_pos = position / 2;
    uint16_t hi_complex = map_hi[fader][hi_pos];
    uint16_t hi_value = (position % 2) ? 0xF0 & hi_complex
                                       : 0x0F & hi_complex;
    
    uint16_t value = (hi_value << 8) + lo_value;
    return value;
}

uint16_t map_binary_search(char fader, uint16_t low_bound, uint16_t hi_bound, uint16_t value)
{
    // Is there anything to?
    if (low_bound == hi_bound)
        return low_bound;
    
    // Get the middle element 
    uint16_t test_pos = low_bound + ((hi_bound - low_bound) / 2);
    uint16_t test_value = getMap(fader, test_pos);
    
    if (test_value < value)
        return map_binary_search(fader, test_pos, hi_bound, value);
    else if (test_value > value)
        return map_binary_search(fader, low_bound, test_pos, value);
    else
        return test_pos;
}

uint16_t map_approx_lookup(char fader, uint16_t lkValue)
{
    // We will do a binary tree search through the map arrays
    return map_binary_search(fader, 0, 1023, lkValue);    
}

void SaveTempMapToFlash(char fader)
{
    /*
    for(i=0; i<32; i++)
    {
        data_ram[i] = i*2047;
    }
        
    asm_write16b_row_flash(0x00,0x1F80,data_ram); 
    asm_read16b_row_flash(0x00,0x1F80,result);    
    tmp=asm_read16b_flash(0x00,0x1F86);
    */

    
    int i;
    _prog_addressT p_s_lo, p_lo;
    _init_prog_address(p_s_lo, map_lo);
    for (i = 0; i < 1024; i += 64)
    {     
        p_lo = p_s_lo + i;
        _erase_flash(p_lo);
        _write_flash16(p_lo, (int *)(temp_map_lo[fader] + i));
    }

    _prog_addressT p_s_hi, p_hi;
    _init_prog_address(p_s_hi, map_hi);    
    for (i = 0; i < 512; i += 64)
    {
        p_hi = p_s_hi + i;
        _erase_flash(p_hi);
        _write_flash16(p_hi, (int *)(temp_map_hi[fader] + i));
    }
}

uint16_t readADC(char channel)
{
    int r = rand();
    r = r & 0x3FF;
    return r;
}


/* <Initialize variables in user.h and insert code for user algorithms.> */

/* TODO Initialize User Ports/Peripherals/Project here */

void InitApp(void)
{
    /* Setup analog functionality and port direction */

    /* Initialize peripherals */
}

