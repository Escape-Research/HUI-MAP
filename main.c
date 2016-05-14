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

#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp              */

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/* i.e. uint16_t <variable_name>; */


__psv__ char __attribute__((space(psv))) map_lo[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_hi[8][512] = { 0 }; 

char temp_map_lo[1024];
char temp_map_hi[512];

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

uint16_t map_approx_lookup(char fader, uint16_t lkValue)
{
    // We will do a binary tree search through the map arrays
    
}

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{
    char bCalMode = 0;
    
    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Initialize IO ports and peripherals */
    InitApp();

    /* TODO <INSERT USER APPLICATION CODE HERE> */

    char fadernum = 0;

    while(1)
    {
        if (!bCalMode)
        {
            // Do translation
            
            // Figure out which fader are we operating on!
            fadernum = 0;
            
            // Capture the current fader position (Analog input 0)
            uint16_t fpos = readADC(0);
                        
            // Locate where this value is on the map!
            uint16_t corrected_value = map_approx_lookup(fadernum, fpos);
            
            // Output that (behave like an ADC1001  !!!!!)
            
            // Should we enter cal mode?
        }
        else
        {
            // Figure out which fader are we operating on!
            fadernum = 0;
            
            // Capture the current reference position (Analog input 1)
            uint16_t ref = readADC(1);
            
            // Capture the current fader position (Analog input 0)
            uint16_t dut = readADC(0);
            
            // Scale the 12bit to a 10bit value
            long scaled_value = ref * 10;
            scaled_value /= 12;
            
            // Update the temp_map      
            uint16_t dut10bit_lo = dut & 0xFF;
            uint16_t dut10bit_hi = dut >> 8;
            temp_map_lo[scaled_value] = dut10bit_lo;
            
            uint16_t hi_pos = scaled_value / 2;
            char existing_hi = temp_map_hi[hi_pos];
            temp_map_hi[hi_pos] = (scaled_value % 2) 
                                  ? (dut10bit_hi << 4) + (existing_hi & 0xF)
                                  : (existing_hi & 0xF0) + dut10bit_hi;
            
            // Should we exit cal mode?
        }
    }
}
