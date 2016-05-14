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

/* <Initialize variables in user.h and insert code for user algorithms.> */

/* TODO Initialize User Ports/Peripherals/Project here */

void InitApp(void)
{
    /* Setup analog functionality and port direction */

    /* Initialize peripherals */
}

