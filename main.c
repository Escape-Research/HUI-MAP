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
   [Button]    T2CK/CN1/RC13  11 |       | 18  PGC/SDA/RF2  [Used for Debugging]
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
 
 Design considerations:
======================
 * Use a combination of 1uF, 0.1uF and 0.01uF on Vdd and AVdd.
 * Use a 0.01 or 0.1uF cap to ground on the analog inputs to help remove some of
   the higher frequency noise.
 
 NOTES FOR ADC1001:
 =================
 * When BOTH CS (pin 1) and WR (pin 3) go LOW the device starts a new conversion
 * When BOTH CS (pin 1) and RD (pin 2) go LOW the output latches are enabled.
 * Each successive read (8 bits + 2 bits) is a complete cycle of CS / RD going
   LOW.
 * When CS is high the output latches should be TRI-STATE
 * 
 * 
 * Timing - Tick = 36 ns (un-tuned)
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
#include <stddef.h>

#include "system.h"        /* System funct/params, like osc/peripheral config */

#include <libpic30.h>
#include <adc12.h>

#include "user.h"          /* User funct/params, such as InitApp              */

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

// The map calibration values (stored in the flash memory)
__prog__ int __attribute__((space(prog),aligned(_FLASH_PAGE * 2)))  map_cal_flash[32];

//char __attribute__((space(eedata), aligned(_EE_ROW)))  map_cal_flash_lo[32];
//char __attribute__((space(eedata), aligned(_EE_ROW)))  map_cal_flash_hi[32];
//__eds__ int _EEDATA(2) map_cal_flash[32];
//int __attribute__((space(eedata), address(0x7FFC00), aligned(_EE_ROW))) 
//               map_cal_flash[] = {  _12BIT_HALF, _12BIT_1Q, _12BIT_3Q, _12BIT_FS,
//                                    _12BIT_HALF, _12BIT_1Q, _12BIT_3Q, _12BIT_FS,
//                                    _12BIT_HALF, _12BIT_1Q, _12BIT_3Q, _12BIT_FS,
//                                    _12BIT_HALF, _12BIT_1Q, _12BIT_3Q, _12BIT_FS};
                                    //_12BIT_HALF, _12BIT_1Q, _12BIT_3Q, _12BIT_FS,
                                    //_12BIT_HALF, _12BIT_1Q, _12BIT_3Q, _12BIT_FS,
                                    //_12BIT_HALF, _12BIT_1Q, _12BIT_3Q, _12BIT_FS,
                                    //_12BIT_HALF, _12BIT_1Q, _12BIT_3Q, _12BIT_FS };

// Buffer in data memory used during runtime
int map_cal[8][4];

// Buffer to hold the queued output values for each fader read inquiry
uint16_t out_buffer[8];

// Buffer to hold the current value for each one of the 8 faders
uint16_t fader_pos[8];

// Last known state of the push button
unsigned g_bButtonState = 0;

// The next 10bit value to output
uint16_t g_nextOutput = 0;

// Have we already send out the 1st byte?
char g_bOutput2ndByte = 0;

// Current Fader selected
SELBITS g_SELbits; // = { 0, 0, 0 };

// Are we in cal mode?
char g_bCalMode = 0;

// Region to calibrate
char g_CalRegion = 0;  

// Are we ready to start? (CS and WR signals low)
char g_bReadyToStart = 0;

// Flag that the button was pressed for a long duration
char g_bLongDuration = 0;

// Flag to signal that we should enter calibration mode
char g_bShouldEnterCal = 0;

// Flag to signal that we should exit calibration mode
char g_bShouldExitCal = 0;

// Blink counter
unsigned g_Blinks = 0;

// Flag to keep track of the LED ON state
char g_bLEDON = 0;

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{   
    // utility counters
    int i, j;
    
    /* Configure the oscillator for the device */
    ConfigureOscillator();
    
    /* Initialize IO ports, peripherals and interrupts */
    InitApp();

    // Copy the calibration values from flash to the data memory
    //for (i = 0; i < 8; i++)
    //    for (j = 0; j < 4; j++)
    //        map_cal[i][j] = map_cal_flash[(i * 4) + j];
    
    // If we still haven't saved a calibration blink once!
    if (map_cal[0][1] == 0)
    {
        // Initialize the map
        init_tempmap();
        
        g_Blinks = 1;
        TMR1 = 0;
        T1CONbits.TON = 1;
    }
    
    // clear-up the temp map
    //init_tempmap();

    // The main (infinite) loop
    while(1)
    {
        // Reset the watchdog!
        ClrWdt();
        
        // Update the LED status
        LATCbits.LATC15 = g_bLEDON;
        
        if (!g_bCalMode)
        {
            // Wait for request to start
            if (g_bReadyToStart)
            {
                // Reset the flag
                g_bReadyToStart = 0;
                
                // Do translation

                // Wait until the voltage is stable before we begin the A2D
                __delay_us(150);
                
                // Cache the fader number
                int currFader = getFaderNum();

                // Capture the current fader position (Analog input 0)
                uint16_t fpos = readADC();

                // Disable CN interrupts (just for debugging convenience)
                //_CNIE = 0;      
                // Locate where this value is on the map!
                g_nextOutput = map_location(currFader, fpos);
                // Re-enable CN interrupts (just for debugging convenience)
                //_CNIE = 1;      

                // Process the next two RD requests...
                asm_ProcessRDRequest(g_nextOutput);
            }
                           
            // Should we enter cal mode?
            if (g_bShouldEnterCal)
            {
                // Clear the flag
                g_bShouldEnterCal = 0;
                
                // Clear up the fader_pos array
                for (i = 0; i < 8; i++)
                    fader_pos[i] = 4096;    // 4096 is unsaved
                
                g_bCalMode = 1;
            }
        }
        else
        {
            // Wait for request to start
            if (g_bReadyToStart)
            {
                // Reset the flag
                g_bReadyToStart = 0;

                // Wait until the voltage is stable before we begin the A2D
                __delay_us(150);

                // Figure out which fader we have been given..
                char currFader = getFaderNum();

                // Read the location of this fader
                uint16_t dut = readADC();

                // record the location of this fader
                fader_pos[currFader] = dut; //scaled_value;
                
                // Have we recorded all 8 fader locations yet?
                bool bReady = true;
                for (i = 0; i < 8; i++)
                    if (fader_pos[i] == 4096)
                        bReady = false;
                
                if (bReady)
                {
                    LATCbits.LATC15 = !g_bLEDON;
                    
                    switch(g_CalRegion)
                    {
                        // Half-point
                        case 0: g_nextOutput = 50; break;
                        case 1: g_nextOutput = 25; break;
                        case 2: g_nextOutput = 75; break;
                        case 3: g_nextOutput = 100; break;
                    }
                    
                    // Process the next two RD requests...
                    asm_ProcessRDRequest(g_nextOutput);

                    LATCbits.LATC15 = g_bLEDON;

                    // Should we exit cal mode and adjust the maps?
                    if (g_bShouldExitCal)
                    {
                        // Disable CN interrupts and Watchdog
                        _CNIE = 0;      
                        __builtin_disable_interrupts();

                        // Clear the flag
                        g_bShouldExitCal = 0;

                        // Initialize the map_cal (if needed)
                        if (g_CalRegion == 0)
                            init_tempmap();
                        
                        // Store the calibrated positions
                        for (i = 0; i < 8; i++)
                            map_cal[i][g_CalRegion] = fader_pos[i];

                        // Save calibration to flash
                        //SaveTempMapToFlash();
                        
                        // Re-enable CN interrupts
                        _CNIE = 1;      
                        __builtin_enable_interrupts();

                        // Blink (based on the current Fader number)
                        // to let us know that flash is saved!
                        g_Blinks = g_CalRegion + 1;
                        TMR1 = 0;
                        T1CONbits.TON = 1;

                        g_bCalMode = 0;                        
                    }
                }                
            }            
        }
    }
}
