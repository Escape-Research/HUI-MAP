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

#include "system.h"

#include <libpic30.h>

#include "user.h"

/******************************************************************************/
/* Interrupt Vector Options                                                   */
/******************************************************************************/
/*                                                                            */
/* Refer to the C30 (MPLAB C Compiler for PIC24F MCUs and dsPIC33F DSCs)      */
/* MPLAB Assembler, Linker and Utilities for PIC24F MCUs and dsPIC DSCs       */
/* User's Guide for an up to date list of the available interrupt options.    */
/* Alternately these names can be pulled from the device linker scripts.      */
/*                                                                            */
/* Primary Interrupt Vector Names:                                            */
/*                                                                            */
/* _INT0Interrupt  _INT2Interrupt                                             */
/* _IC1Interrupt   _U2RXInterrupt                                             */
/* _OC1Interrupt   _U2TXInterrupt                                             */
/* _T1Interrupt    _SPI2Interrupt                                             */
/* _IC2Interrupt   _C1Interrupt                                               */
/* _OC2Interrupt   _IC3Interrupt                                              */
/* _T2Interrupt    _IC4Interrupt                                              */
/* _T3Interrupt    _IC5Interrupt                                              */
/* _SPI1Interrupt  _IC6Interrupt                                              */
/* _U1RXInterrupt  _OC5Interrupt                                              */
/* _U1TXInterrupt  _OC6Interrupt                                              */
/* _ADCInterrupt   _OC7Interrupt                                              */
/* _NVMInterrupt   _OC8Interrupt                                              */
/* _SI2CInterrupt  _INT3Interrupt                                             */
/* _MI2CInterrupt  _INT4Interrupt                                             */
/* _CNInterrupt    _C2Interrupt                                               */
/* _INT1Interrupt  _PWMInterrupt                                              */
/* _IC7Interrupt   _QEIInterrupt                                              */
/* _IC8Interrupt   _DCIInterrupt                                              */
/* _OC3Interrupt   _LVDInterrupt                                              */
/* _OC4Interrupt   _FLTAInterrupt                                             */
/* _T4Interrupt    _FLTBInterrupt                                             */
/* _T5Interrupt                                                               */
/*                                                                            */
/* For alternate interrupt vector naming, simply add 'Alt' between the prim.  */
/* interrupt vector name '_' and the first character of the primary interrupt */
/* vector name.                                                               */
/*                                                                            */
/* For example, the vector name _ADC2Interrupt becomes _AltADC2Interrupt in   */
/* the alternate vector table.                                                */
/*                                                                            */
/* Example Syntax:                                                            */
/*                                                                            */
/* void __attribute__((interrupt,auto_psv)) <Vector Name>(void)               */
/* {                                                                          */
/*     <Clear Interrupt Flag>                                                 */
/* }                                                                          */
/*                                                                            */
/* For more comprehensive interrupt examples refer to the C30 (MPLAB C        */
/* Compiler for PIC24 MCUs and dsPIC DSCs) User Guide in the                  */
/* <compiler installation directory>/doc directory for the latest compiler    */
/* release. For XC16, refer to the MPLAB XC16 ASSEMBLER, LINKER AND UTILITIES */
/* User's Guide within the <XC16 compiler instal directory>/doc folder.  The  */
/* chapter to refer is entitled "INTERRUPT VECTOR TABLES"                     */
/*                                                                            */
/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/

/* TODO Add interrupt routine code here.                                      */
void __attribute__((interrupt(auto_psv))) _CNInterrupt(void)
{
    // Clear the CN interrupt flag
    _CNIF = 0;
    
    // Figure out what has changed!
    
    PORTCBITS portc = PORTCbits;
    PORTFBITS portf = PORTFbits;
    
    // SEL A - CN0 (PORTC:14)
    g_SELbits.SELA = portc.RC14;
 
    // SEL B - CN17 (PORTF:4)
    g_SELbits.SELB = portf.RF4;
 
    // SEL C - CN18 (PORTF:5)
    g_SELbits.SELC = portf.RF5;
    
    // push button - CN1 (PORTC:13)
    if (portc.RC13 != g_bButtonState)
    {
        if (!portc.RC13)
        {
            // The user just pressed the button
            
            // Reset the "long" duration flag
            g_bLongDuration = 0;

            // Reset and Start the Timer2
            TMR2 = 0;
            T2CONbits.TON = 1;            
        }
        else
        {
            // The user just released the button
            
            // Stop and Reset Timer2
            T2CONbits.TON = 0;
            TMR2 = 0;
            
            // Was it a long duration?
            HandleButton(g_bLongDuration);            
        }
        g_bButtonState = portc.RC13;
    }
}

void __attribute__((interrupt(auto_psv))) _INT0Interrupt(void)
{
    // Handle CS (positive going edge)
    
    // Make sure that we out the port B output back to TRI-STATE
    DisableDataOutput();
    
    // Clear the INT0 interrupt flag
    _INT0IF = 0;
}

void __attribute__((interrupt(auto_psv))) _INT1Interrupt(void)
{
    // Handle RD (negative going edge)
    
    // Check if CS (RF6) is LOW
    if (LATFbits.LATF6 == 0)
    {
        // We need to output the next BYTE on port B2:9
                
        // Is this the first or the second byte?
        if (!g_bOutput2ndByte)
        {
            OutputByte(g_nextOutput & 0xFF);
            g_bOutput2ndByte = 1;
        }
        else
        {
            OutputByte(g_nextOutput & 0x300);
            g_bOutput2ndByte = 0;
                        
            // Keep outputing the same as long as we don't
            // have a new AD conversion request
            //g_nextOutput = 0;
        }

        // Wait 
        __delay_us(100);
        
        // Make sure that enable the Output
        EnableDataOutput();
    }
    
    // Clear the INT1 interrupt flag
    _INT1IF = 0;
}

void __attribute__((interrupt(auto_psv))) _INT2Interrupt(void)
{
    // Handle WR (negative going edge)
  
    // Check if CS (RF6) is LOW
    if (LATFbits.LATF6 == 0)
    {
        // We need to initiate a new AD conversion
        g_bReadyToStart = 1;                

        // Blip the LED
        int prevState = PORTCbits.RC15;        
        LATCbits.LATC15 = 1;
        __delay_us(1);
        LATCbits.LATC15 = 0;        
        LATCbits.LATC15 = prevState; 
    }
    
    // Clear the INT2 interrupt flag
    _INT2IF = 0;
}

void __attribute__((interrupt(auto_psv))) _T1Interrupt(void)
{
    // Stop Timer1
    T1CONbits.TON = 0;
    TMR1 = 0;

    // Are we to turn on or off?
    if (!g_bLEDON)
    {
        // Turn on the LED
        LATCbits.LATC15 = 1;
        
        // Change the ON flag
        g_bLEDON = 1;
    }
    else
    {
        // Turn off the LED
        LATCbits.LATC15 = 0;
        
        // Update the ON flag
        g_bLEDON = 0;
        
        // Decrement the blink counter
        if (g_Blinks)
            g_Blinks--;
    }

    if (g_Blinks)
    {
        // Restart the Timer
        TMR1 = 0;
        T1CONbits.TON = 1;
    }
        
    // Clear the T1 interrupt flag
    _T1IF = 0;   
}

void __attribute__((interrupt(auto_psv))) _T2Interrupt(void)
{
    // Stop Timer2
    T2CONbits.TON = 0;
    TMR2 = 0;
    
    // Note the fact that we have reached the "long" push duration
    g_bLongDuration = 1;
    
    // Turn on the LED as an indication that we will enter Calibration mode
    LATCbits.LATC15 = 1;
    
    // Clear the T2 interrupt flag
    _T2IF = 0;   
}
