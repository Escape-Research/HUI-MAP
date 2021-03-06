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

#include "system.h"          /* variables/params used by system.c             */

/******************************************************************************/
/* System Level Functions                                                     */
/*                                                                            */
/* Custom oscillator configuration funtions, reset source evaluation          */
/* functions, and other non-peripheral microcontroller initialization         */
/* functions get placed in system.c                                           */
/*                                                                            */
/******************************************************************************/

/* Refer to the device Family Reference Manual Oscillator section for
information about available oscillator configurations.  Typically
this would involve configuring the oscillator tuning register or clock
switching useing the compiler's __builtin_write_OSCCON functions.
Refer to the C Compiler for PIC24F MCUs and dsPIC DSCs User Guide in the
compiler installation directory /doc folder for documentation on the
__builtin functions.  Refer to the XC16 C Compiler User's Guide appendix G
 for a list of the XC16 compiler __builtin functions */

void ConfigureOscillator(void)
{
    // Make all ports inputs temporarily!
    TRISB = 0x3FF;
    TRISC = 0x3FF;
    TRISD = 0x3FF;
    TRISF = 0x3FF;

    /* Enable Watch Dog Timer */
    RCONbits.SWDTEN = 1;

    // As fast as she'll go (on internal clock)
    OSCTUN = 0x7;

    /* When clock switch occurs switch to Pri Osc controlled by FPR<4:0> */
    __builtin_write_OSCCONH(0x07);  /* Set OSCCONH for clock switch */
    __builtin_write_OSCCONL(0x01);  /* Start clock switching */
    while(OSCCONbits.COSC != 0b111);

    /* Wait for Clock switch to occur */
    /* Wait for PLL to lock, if PLL is used */
    while(OSCCONbits.LOCK != 1); 

    // Reset the watchdog!
    ClrWdt();
}

