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

// Retrieve the 12 bit calibration value from the map in flash associated 
// with the specific fader (0..7) for the specific 10 bit location (0..1023)
uint16_t getMap(char fader, uint16_t position)
{
    char lo_value = map_lo[fader][position];
    uint16_t hi_pos = position >> 1;
    char hi_complex = map_hi[fader][hi_pos];
    char hi_value = (position & 0x01) ? 0xF0 & hi_complex
                                      : 0x0F & hi_complex;
    
    uint16_t value = (hi_value << 8) + lo_value;
    return value;
}

// Retrieve the 12 bit calibration value from the current temporary map 
// in RAM associated with the specific 10 bit location (0..1023)
uint16_t gettempMap(uint16_t position)
{
    char lo_value = temp_map_lo[position];
    uint16_t hi_pos = position >> 1;
    char hi_complex = temp_map_hi[hi_pos];
    char hi_value = (position & 0x01) ? 0xF0 & hi_complex
                                      : 0x0F & hi_complex;
    
    uint16_t value = (hi_value << 8) + lo_value;
    return value;
}

uint16_t settempMap(uint16_t position, uint16_t value)
{
    char dut10bit_lo = value & 0xFF;
    char dut10bit_hi = value >> 8;
    temp_map_lo[position] = dut10bit_lo;

    uint16_t hi_pos = position >> 1;
    char existing_hi = temp_map_hi[hi_pos];
    temp_map_hi[hi_pos] = (position & 0x01) 
                          ? (dut10bit_hi << 4) + (existing_hi & 0xF)
                          : (existing_hi & 0xF0) + dut10bit_hi;
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

void init_tempmap()
{
    int i = 0;
    for (i = 0; i < 1024; i++)
    {
        if (i < 512)
            temp_map_hi[i] = 0;
        
        temp_map_lo[i] = 0;
    }
}

void interpolate_tempmap()
{
    int i = 0;
    int prev_value = gettempMap(0);
    int curr_value = prev_value;
    for (i = 1; i < 1023; i++)
    {
        curr_value = gettempMap(i);
        
        int next_val_pos = 1;
        if (curr_value == 0)
        {
            int next_val_pos = i + 1;
            int next_value = gettempMap(next_val_pos);
            while (next_value == 0)
            {
                next_val_pos++;
                if (next_val_pos < 1024)
                    next_value = gettempMap(next_val_pos);
                else
                    next_value = 0xFFF;
            }
            curr_value = (next_value - prev_value) / (next_val_pos - i);
            settempMap(i, curr_value);
        }
    }
    curr_value = gettempMap(1023);
    if (curr_value == 0)
        settempMap(1023, 0xFFF);
}

void SaveTempMapToFlash(char fader)
{
    int i;
    int offset_lo = fader << 10;
    _prog_addressT p_s_lo, p_lo;
    _init_prog_address(p_s_lo, map_lo);
    p_s_lo += offset_lo;
    for (i = 0; i < 1024; i += 64)
    {     
        p_lo = p_s_lo + i;
        _erase_flash(p_lo);
        
        int buffer[32];
        int j = 0;
        for (j = 0; j < 32; j++)
        {
            int value_pos = i + (j << 1);
            buffer[j] = temp_map_lo[value_pos] + (temp_map_lo[value_pos + 1] << 8);
        }
        
        _write_flash16(p_lo, buffer);
    }

    int offset_hi = fader << 9;
    _prog_addressT p_s_hi, p_hi;
    _init_prog_address(p_s_hi, map_hi);    
    p_s_hi += offset_hi;
    for (i = 0; i < 512; i += 64)
    {
        p_hi = p_s_hi + i;
        _erase_flash(p_hi);

        int buffer[32];
        int j = 0;
        for (j = 0; j < 32; j++)
        {
            int value_pos = i + (j << 1);
            buffer[j] = temp_map_hi[value_pos] + (temp_map_hi[value_pos + 1] << 8);
        }

        _write_flash16(p_hi, buffer);
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

