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

// Permanent calibration maps located in FLASH
__prog__ char __attribute__((space(prog), aligned(_FLASH_PAGE * 2))) map_lo[8][1024];
__prog__ char __attribute__((space(prog), aligned(_FLASH_PAGE * 2))) map_hi[8][512]; 

// Calibration indicator flags
__prog__ int __attribute__((space(prog), aligned(_FLASH_PAGE * 2))) map_saved[32] = 
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// Temporary calibration map located in RAM
char temp_map_lo[1024];     // = { '\0' };
char temp_map_hi[512];      // = { '\0' };
int map_saved_buffer[32];   // = { 0 };

// Buffer to hold the queued output values for each fader read inquiry
uint16_t out_buffer[8];

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

// Fader to calibrate
char g_CalFader = 7;  // It will reset to 0 when enter CalMode

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
    /* Configure the oscillator for the device */
    ConfigureOscillator();
    
    INTCON1bits.NSTDIS = 1;   // disable nested interrupts
    
    /* Initialize IO ports, peripherals and interrupts */
    InitApp();
    
    // Load flags from flash
    int i = 0;
    for (i = 0; i < 32; i++)
        map_saved_buffer[i] = map_saved[i];
    
    // If we still haven't saved a calibration blink once!
    if (map_saved_buffer[0] == 0)
    {
        g_Blinks = 1;
        TMR1 = 0;
        T1CONbits.TON = 1;
    }
    
    // clear-up the temp map
    init_tempmap();

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
            //while (!g_bCalMode)            
            {
                // Reset the flag
                g_bReadyToStart = 0;
                
                // Do translation

                // Wait until the voltage is stable before we begin the A2D
                __delay_us(125);
                
                // Cache the fader number
                int currFader = getFaderNum();

                // Capture the current fader position (Analog input 0)
                uint16_t fpos = readADC(0, NULL);

                // Do we have a calibration?
                if (map_saved[currFader])
                {
                    // Locate where this value is on the map!
                    uint16_t corrected_value = map_approx_lookup(currFader, fpos);

                    // Output that (queue) (behave like an ADC1001  !!!!!)
                    g_nextOutput = corrected_value;
                    //g_nextOutput = 0x3FF;
                }
                else
                {
                    // No calibration done yet, just truncate the 2 LSBs
                    // (but round out the value to the closest 10bit)
                    int remainder = fpos & 0b11;
                    if (remainder < 2)
                        remainder = 0;
                    else
                        remainder = 4;
                    g_nextOutput = (fpos + remainder) >> 2;
                    if (g_nextOutput > 0x3FF)
                        g_nextOutput = 0x3FF;
                }

                // Store the result in the queue
                //out_buffer[currFader] = g_nextOutput;
                
                // Locate the appropriate queue index to push
                //int nextIndex = currFader; //(currFader + 9) % 8;
                //g_nextOutput = out_buffer[nextIndex];
                
                // The first output doesn't have the last two LSBs
                LATB = g_nextOutput & 0x3FC;
                
                /*LATBbits.LATB2 = g_nextOutput & 0b100;
                LATBbits.LATB3 = g_nextOutput & 0b1000;
                LATBbits.LATB4 = g_nextOutput & 0b10000;
                LATBbits.LATB5 = g_nextOutput & 0b100000;
                LATBbits.LATB6 = g_nextOutput & 0b1000000;
                LATBbits.LATB7 = g_nextOutput & 0b10000000;
                LATBbits.LATB8 = g_nextOutput & 0b100000000;
                LATBbits.LATB9 = g_nextOutput & 0b1000000000;*/
                
                // Process the following two RD requests (assembly)
                asm_ProcessRDRequest();
            }
                           
            // Should we enter cal mode?
            if (g_bShouldEnterCal)
            {
                // Clear the flag
                g_bShouldEnterCal = 0;
                
                // Clear up the temp map
                init_tempmap();   
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

                // Figure out which fader we have been given..
                char currFader = getFaderNum();

                // Is this the one we are calibrating?
                if (currFader == g_CalFader)
                {
                    // Capture the current fader and reference position
                    uint16_t ref = 0;
                    uint16_t dut = readADC(1, &ref);
                    
                    // Scale the 12bit ref to a 10bit value
                    // (but round out the value to the closest 10bit)
                    int remainder = ref & 0b11;
                    if (remainder < 2)
                        remainder = 0;
                    else
                        remainder = 4;
                    uint16_t scaled_value = (ref + remainder) >> 2;
                    if (scaled_value > 0x3FF)
                        scaled_value = 0x3FF;

                    // Update the temp_map      
                    settempMap(scaled_value, dut);
                }
            }
            
            // Should we exit cal mode?
            if (g_bShouldExitCal)
            {
                // Clear the flag
                g_bShouldExitCal = 0;
                
                // Disable interrupts
                __builtin_disable_interrupts();
                
                //INTCON1bits.NSTDIS = 1;   // disable nested interrupts
                //_DISI = 1;
                
                // if YES, then save the temp_map to flash
                interpolate_tempmap();
                
                // mark the flag
                map_saved_buffer[g_CalFader] = 1;
                
                // Save to flash
                SaveTempMapToFlash(g_CalFader);

                // Re-enable interrupts
                __builtin_enable_interrupts();
                
                //INTCON1bits.NSTDIS = 0;   // enable nested interrupts
                //_DISI = 0;
                
                // Blink (based on the current Fader number)
                // to let us know that flash is saved!
                g_Blinks = g_CalFader + 1;
                TMR1 = 0;
                T1CONbits.TON = 1;
                
                g_bCalMode = 0;
            }
        }
    }
}
