/******************************************************************************/
/* General information / pinout                                               */
/******************************************************************************/

// dsPIC30F3013
/*
                               ----_----
                   ~MCLR     1 |       | 28  AVDD
    [Analog In]      AN0     2 |       | 27  AVSS
    [Cal Fader]      AN1     3 |       | 26  RB6
                 CN4/RB2     4 |       | 25  RB7
                 CN5/RB3     5 |       | 24  RB8
                 CN6/RB4     6 |       | 23  RB9
                 CN7/RB5     7 |       | 22  CN17/RF4
                     VSS     8 |       | 21  CN18/RF5
                             9 |       | 20  VDD
                    RC15    10 |       | 19  VSS
                CN1/RC13    11 |       | 18  PGC/SDA/RF2
                CN0/RC14    12 |       | 17  PGD/SCL/RF3
                     VDD    13 |       | 16  RF6
                     RD9    14 |       | 15  RD8
                               ---------
 
 */


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

#include <libpic30.h>

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp              */

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/*
__psv__ char __attribute__((space(psv))) map_lo_0_page0[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_0_page64[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_0_page128[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_0_page192[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_0_page256[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_0_page320[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_0_page384[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_0_page448[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_0_page512[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_0_page576[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_0_page640[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_0_page704[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_0_page768[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_0_page832[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_0_page896[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_0_page960[64]; //[8][1024] = { 0 }; 

__psv__ char __attribute__((space(psv))) map_lo_1_page0[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_1_page64[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_1_page128[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_1_page192[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_1_page256[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_1_page320[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_1_page384[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_1_page448[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_1_page512[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_1_page576[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_1_page640[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_1_page704[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_1_page768[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_1_page832[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_1_page896[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_1_page960[64]; //[8][1024] = { 0 }; 

__psv__ char __attribute__((space(psv))) map_lo_2_page0[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_2_page64[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_2_page128[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_2_page192[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_2_page256[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_2_page320[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_2_page384[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_2_page448[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_2_page512[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_2_page576[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_2_page640[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_2_page704[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_2_page768[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_2_page832[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_2_page896[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_2_page960[64]; //[8][1024] = { 0 }; 

__psv__ char __attribute__((space(psv))) map_lo_3_page0[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_3_page64[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_3_page128[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_3_page192[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_3_page256[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_3_page320[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_3_page384[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_3_page448[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_3_page512[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_3_page576[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_3_page640[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_3_page704[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_3_page768[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_3_page832[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_3_page896[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_3_page960[64]; //[8][1024] = { 0 }; 

__psv__ char __attribute__((space(psv))) map_lo_4_page0[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_4_page64[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_4_page128[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_4_page192[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_4_page256[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_4_page320[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_4_page384[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_4_page448[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_4_page512[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_4_page576[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_4_page640[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_4_page704[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_4_page768[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_4_page832[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_4_page896[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_4_page960[64]; //[8][1024] = { 0 }; 

__psv__ char __attribute__((space(psv))) map_lo_5_page0[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_5_page64[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_5_page128[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_5_page192[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_5_page256[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_5_page320[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_5_page384[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_5_page448[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_5_page512[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_5_page576[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_5_page640[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_5_page704[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_5_page768[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_5_page832[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_5_page896[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_5_page960[64]; //[8][1024] = { 0 }; 

__psv__ char __attribute__((space(psv))) map_lo_6_page0[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_6_page64[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_6_page128[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_6_page192[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_6_page256[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_6_page320[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_6_page384[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_6_page448[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_6_page512[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_6_page576[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_6_page640[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_6_page704[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_6_page768[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_6_page832[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_6_page896[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_6_page960[64]; //[8][1024] = { 0 }; 

__psv__ char __attribute__((space(psv))) map_lo_7_page0[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_7_page64[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_7_page128[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_7_page192[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_7_page256[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_7_page320[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_7_page384[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_7_page448[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_7_page512[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_7_page576[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_7_page640[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_7_page704[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_7_page768[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_7_page832[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_7_page896[64]; //[8][1024] = { 0 }; 
__psv__ char __attribute__((space(psv))) map_lo_7_page960[64]; //[8][1024] = { 0 }; 
*/

__psv__ char __attribute__((space(psv), aligned(_FLASH_PAGE * 2))) map_lo[8][1024] = { 0x12 };
__psv__ char __attribute__((space(psv), aligned(_FLASH_PAGE * 2))) map_hi[8][512] = { 0x34 }; 

char temp_map_lo[1024];
char temp_map_hi[512];

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{
    char bCalMode = 1;      // are we in calibration mode?
    char fadernum = 0;      // the active fader (based on the HUI selection lines)
    
    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Initialize IO ports and peripherals */
    InitApp();

    /* TODO <INSERT USER APPLICATION CODE HERE> */


    while(1)
    {
        if (!bCalMode)
        {
            // Do translation
            
            // Figure out which fader are we operating on!
            fadernum = 0;
            
            // Capture the current fader position (Analog input 0)
            uint16_t fpos = 0; //readADC(0);
                        
            // Locate where this value is on the map!
            uint16_t corrected_value = map_approx_lookup(fadernum, fpos);
            
            // Output that (behave like an ADC1001  !!!!!)
            
            // Should we enter cal mode?
        }
        else
        {
            // Figure out which fader are we operating on!
            fadernum = 0;
            
            int i = 0;
            for (i = 0; i < 4096; i++)
            {
                // Capture the current reference position (Analog input 1)
                uint16_t ref = i; //readADC(1);

                // Capture the current fader position (Analog input 0)
                uint16_t dut = readADC(0);

                // Scale the 12bit to a 10bit value
                //double scaled_value_d = ref * 0.8333;
                uint16_t scaled_value = ref >> 2;

                // Update the temp_map      
                char dut10bit_lo = dut & 0xFF;
                char dut10bit_hi = dut >> 8;
                temp_map_lo[scaled_value] = dut10bit_lo;

                uint16_t hi_pos = scaled_value >> 1;
                char existing_hi = temp_map_hi[hi_pos];
                temp_map_hi[hi_pos] = (scaled_value & 0x01) 
                                      ? (dut10bit_hi << 4) + (existing_hi & 0xF)
                                      : (existing_hi & 0xF0) + dut10bit_hi;
            }
            
            // Should we exit cal mode?
            
            // if YES, then save the temp_map to flash
            SaveTempMapToFlash(fadernum);
            
        }
    }
}
