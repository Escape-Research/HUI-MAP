/******************************************************************************/
/* General information / pinout                                               */
/******************************************************************************/

// dsPIC30F3013
/*
                                 ----_----
   [Res to VDD, Debug] ~MCLR   1 |       | 28  AVDD
   [Analog In]   AN0/CN2/RB0   2 |       | 27  AVSS
   [Cal Fader]   AN1/CN3/RB1   3 |       | 26  RB6          [D4 out]
   [D0 out]      AN2/CN4/RB2   4 |       | 25  RB7          [D5 out]
   [D1 out]          CN5/RB3   5 |       | 24  RB8          [D6 out]
   [D2 out]          CN6/RB4   6 |       | 23  RB9          [D7 out]
   [D3 out]          CN7/RB5   7 |       | 22  CN17/RF4     [SEL B]
                         VSS   8 |       | 21  CN18/RF5     [SEL C]
   [Not used]        OSC/CLK   9 |       | 20  VDD
   [LED]                RC15  10 |       | 19  VSS
   [Button]         CN1/RC13  11 |       | 18  PGC/SDA/RF2  [Used for Debugging]
   [SEL A]          CN0/RC14  12 |       | 17  PGD/SCL/RF3  [Used for Debugging]
                         VDD  13 |       | 16  RF6/INT0     [CS]
   [WR]             RD9/INT2  14 |       | 15  RD8/INT1     [RD]
                                 ---------

 Connections:
 ===========
 * Pin 1 connects to VDD through a 4K7 - 10K resistor and also goes to the debug
   header
 *
 
 Debug header:
 ============
 * Pin 1 goes to dsPIC pin 1 (~MCLR)
 * Pin 2 goes to dsPIC pin 20 (VDD)
 * Pin 3 goes to dsPIC pin 19 (VSS)
 * Pin 4 goes to dsPIC pin 17 (PGD)
 * Pin 5 goes to dsPIC pin 18 (PGC)
 * Pin 6 remains unconnected

 NOTES FOR ADC1001:
 =================
 * When BOTH CS (pin 1) and WR (pin 3) go LOW the device starts a new conversion
 * When BOTH CS (pin 1) and RD (pin 2) go LOW the output latches are enabled.
 * Each successive read (8 bits + 2 bits) is a complete cycle of CS / RD going
   LOW.
 * When CS is high the output latches should be TRI-STATE
 * 
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

// Permanent calibration maps located in FLASH
__psv__ char __attribute__((space(psv), aligned(_FLASH_PAGE * 2))) map_lo[8][1024] = { 0x00 };
__psv__ char __attribute__((space(psv), aligned(_FLASH_PAGE * 2))) map_hi[8][512] = { 0x00 }; 

// Temporary calibration map located in RAM
char temp_map_lo[1024] = { '\0' };
char temp_map_hi[512] = { '\0' };

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{
    char bCalMode = 0;      // are we in calibration mode?
    char fadernum = 0;      // the active fader (based on the HUI selection lines)
    
    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Initialize IO ports and peripherals */
    InitApp();

    /* TODO <INSERT USER APPLICATION CODE HERE> */

    // clear-up the temp map
    init_tempmap();

    while(1)
    {
        if (!bCalMode)
        {
            // Do translation
            
            // Figure out which fader are we operating on!
            fadernum = 0;
            
            // Capture the current fader position (Analog input 0)
            uint16_t fpos = readADC(0);
            
            if (fpos != 0)
                return -1;
                        
            // Locate where this value is on the map!
            uint16_t corrected_value = map_approx_lookup(fadernum, fpos);
            
            // Output that (queue) (behave like an ADC1001  !!!!!)
            
            // Should we enter cal mode?
            if (0 /* should we enter calibration mode? */)
            {
                // Clear up the temp map
                init_tempmap();   
                bCalMode = 1;
            }
        }
        else
        {
            // Figure out which fader are we operating on!
            fadernum = 3;
            
            // Capture the current reference position (Analog input 1)
            uint16_t ref = readADC(1);

            // Capture the current fader position (Analog input 0)
            uint16_t dut = readADC(0);

            // Scale the 12bit to a 10bit value
            //double scaled_value_d = ref * 0.8333;
            uint16_t scaled_value = ref >> 2;

            // Update the temp_map      
            settempMap(scaled_value, dut);
            
            // Should we exit cal mode?
            if (0 /* should we exit? */)
            {
                // if YES, then save the temp_map to flash
                interpolate_tempmap();
                SaveTempMapToFlash(fadernum);
                
                bCalMode = 0;
            }
            
            uint16_t values[10];
            values[0] = getMap(3, 0);
            values[1] = getMap(3, 1);
            values[2] = getMap(3, 2);
            values[3] = getMap(3, 3);
            values[4] = getMap(3, 4);
            values[5] = getMap(3, 5);
            values[6] = getMap(3, 6);
            values[7] = getMap(3, 7);
            values[8] = getMap(3, 8);
            values[9] = getMap(3, 9);
        }
    }
}
