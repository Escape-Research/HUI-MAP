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

#include "system.h"

#include <libpic30.h>
#include <adc12.h>

#include "user.h"            /* variables/params used by user.c               */

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

// Initialize the temporary holding map (in data memory) with the defaults
void init_tempmap()
{
    // Initialize calibration points to default
    int i = 0;
    for (i = 0; i < 8; i++)
    {
        map_cal[i][0] = _12BIT_HALF;
        map_cal[i][1] = _12BIT_1Q; 
        map_cal[i][2] = _12BIT_3Q;
        map_cal[i][3] = _12BIT_FS;                
    }
}

// Return the "calibrated" 10bit value for a fader
uint16_t map_location(int fader, uint16_t fpos)
{
    double factor;
    uint16_t result = 0;
    
    // Figure out which region we're in
    if (fpos == 0)
    {
        return 0;
    }
    if (fpos < map_cal[fader][1])
    {
        factor = fpos * (double)(_12BIT_1Q);
        factor /= (double)(map_cal[fader][1]);
        result = factor;
    }
    else if (fpos == map_cal[fader][1])
    {
        return _10BIT_1Q;
    }
    else if (fpos < map_cal[fader][0])
    {
        factor = ( fpos - map_cal[fader][1] ) * (double)(_12BIT_1Q);
        factor /= (double)(map_cal[fader][0] - map_cal[fader][1]);
        result = factor + _12BIT_1Q;
    }
    else if (fpos == map_cal[fader][0])
    {
        return _10BIT_HALF;
    }
    else if (fpos < map_cal[fader][2])
    {
        factor = ( fpos - map_cal[fader][0] ) * (double)(_12BIT_1Q);
        factor /= (double)(map_cal[fader][2] - map_cal[fader][0]);        
        result = factor + _12BIT_HALF;
    }
    else if (fpos == map_cal[fader][2])
    {
        return _10BIT_3Q;
    }
    else if (fpos < map_cal[fader][3])
    {
        factor = ( fpos - map_cal[fader][2] ) * (double)(_12BIT_1Q);
        factor /= (double)(map_cal[fader][3] - map_cal[fader][2]);        
        result = factor + _12BIT_3Q;
    }
    else
        return _10BIT_FS;
    
    result = scale_from_12_to_10bits(result);
    return result;
}

// Load calibration values from EEPROM
void LoadCalFromEE()
{
    _prog_addressT EE_addr1, EE_addr2;

    /* initialize a variable to represent the Data EEPROM address */
    _init_prog_address(EE_addr1, map_cal_eeprom1);
    
    /*Copy array "map_cal_eeprom" from DataEEPROM to "map_cal" in RAM*/
    _memcpy_p2d16((int *)map_cal, EE_addr1, _EE_ROW);

    /* initialize a variable to represent the Data EEPROM address */
    _init_prog_address(EE_addr2, map_cal_eeprom2);
    
    /*Copy array "map_cal_eeprom" from DataEEPROM to "map_cal" in RAM*/
    _memcpy_p2d16((int *)&map_cal[4], EE_addr2, _EE_ROW);
}

// Save calibration values to EEPROM
void SaveCalToEE()
{
    _prog_addressT EE_addr1, EE_addr2;

    /* initialize a variable to represent the Data EEPROM address */
    _init_prog_address(EE_addr1, map_cal_eeprom1);

    /*Erase a row in Data EEPROM at array "map_cal_eeprom1" */
    _erase_eedata(EE_addr1, _EE_ROW);
    _wait_eedata();

    /*Write a row to Data EEPROM from array "map_cal" */
    _write_eedata_row(EE_addr1, (int *)map_cal);
    _wait_eedata();    

    /* initialize a variable to represent the Data EEPROM address */
    _init_prog_address(EE_addr2, map_cal_eeprom2);

    /*Erase a row in Data EEPROM at array "map_cal_eeprom2" */
    _erase_eedata(EE_addr2, _EE_ROW);
    _wait_eedata();

    /*Write a row to Data EEPROM from array "map_cal" */
    _write_eedata_row(EE_addr2, (int *)&map_cal[4]);
    _wait_eedata();    
}

// Quick helpder to return the current state of the SEL A, B and C lines
// that indicate which multiplexed fader analog level is present in the 
// AD0 channel.
int getFaderNum()
{
    int fader = (PORTFbits.RF5 << 2) +
                (PORTFbits.RF4 << 1) +
                (PORTCbits.RC14);
    return fader;
}

// Round up to the nearest 10bit value and truncate the last two LSBs
uint16_t scale_from_12_to_10bits(uint16_t value)
{
    // Round out to the closest 10 bit value
    uint16_t remainder = value & 0b11;
    
    if (remainder < 2)
        remainder = 0;
    else
        remainder = 4;
    
    uint16_t result = (value + remainder) >> 2;
    
    // Make sure that we don't go over 10bits
    if (result > 0x3FF)
        result = 0x3FF;
    
    return result;
}

// Initialize the ADC configuration and "turn on" the module
// Make sure that the module is stable upon returning from this function.
void configADC()
{
    // Make sure the ADC if off during initial configuration
    ADCON1bits.ADON = 0;

    // Enable AN0 and AN1 as analog inputs
    ADPCFG = ENABLE_AN0_ANA & ENABLE_AN1_ANA;

    ADCON1bits.ADSIDL = 0;   // Continue operation while in idle mode
    ADCON1bits.FORM = 0x00;  // Output format is Integer
    ADCON1bits.SSRC = 0b111; // Internal counter ends sampling and starts conversion
    ADCON1bits.ASAM = 0;     // Initial the A/D sample enable is turned-off
    ADCON1bits.SAMP = 0;     // A/D sample/hold amp is (should) not sampling
    
    ADCHSbits.CH0SA = 0x00;  // Channel 0 for MUX A positive is AN0
    ADCHSbits.CH0NA = 0;     // Channel 0 for MUX A negative is VREF- (AVss)
    ADCHSbits.CH0SB = 0x01;  // Channel 0 for MUX B positive is AN1
    ADCHSbits.CH0NB = 0;     // Channel 0 for MUX B negative is VREF- (AVss)
    
    ADCSSL = 0x0000;         // No scanning
    
    ADCON3bits.SAMC = 0x04;  // (0x01) 1 TAD (auto sample time)
    ADCON3bits.ADRC = 0;     // Clock derived from system clock
    ADCON3bits.ADCS = 22;    // (19) Configure for 333nS TAD time
    
    ADCON2bits.CSCNA = 0;    // Do not scan inputs
    ADCON2bits.BUFM = 0;     // Buffer configured as one 16-word buffer ADCBUF(15..0)
    ADCON2bits.SMPI = 0xF;    // Collect 16 samples at a time!
    ADCON2bits.ALTS = 0;     // Don't alternate between MUX A and MUX B
}

// This is the primary A/D conversion handling routine.
// The are two modes supported single channel (only MUX A - AD0)
// and alternate channels (MUX A / MUX B - AD0, AD1).
// In both cases each channel will be sampled and converted multiple consecutive
// times and the converted values will be averaged.
// channel == 0 denotes single channel and channel == 1 denotes dual.
// For dual channel, the parameter pAltResult is expected to be provided
// as a pointer to store the averaged 2nd channel (AD1).
uint16_t readADC()
{
    uint16_t resultA;
    int batch, count;
    uint16_t res1 = 0;
    uint16_t *ADC16Ptr;

    // Turn on the ADC
    ADCON1bits.ADON = 1;
    
    // Make sure that we'll wait for at least 20uS for ADC to stabilize
    __delay_us(20);

    // we will do 2 batches of 16 samples
    for (batch = 0; batch < 2; batch++)
    {
        // We are only to sample AD0
        ADCON2bits.ALTS = 0;

        // Collect 16 samples at a time!
        ADCON2bits.SMPI = 0xF;

        // Initialize accumulation vars
        resultA = 0;

        // Initialize buffer pointer
        ADC16Ptr = (uint16_t *)&ADCBUF0;

        // Reset the AD interrupt flag
        IFS0bits.ADIF = 0;

        // Initiate auto-sampling
        ADCON1bits.ASAM = 1;

        // Wait for the sampling to complete
        while (!IFS0bits.ADIF)
            ;

        // Make sure that we prevent further auto-sampling
        ADCON1bits.ASAM = 0;

        // We took 16 samples of AD0 (using MUX A)
        for (count = 0; count < 16; count++)
            resultA = resultA + *ADC16Ptr++;

        // Divide by 16 to calculate the average value
        resultA = resultA >> 4;

        res1 += resultA;
    }
    
    res1 = res1 >> 1;

    // Turn off the ADC
    ADCON1bits.ADON = 0;    

    return res1;
}

// Update state transition indicator flags based on push button triggers
void HandleButton(char bLongDuration)
{
    // If we are currently in the middle of indicating the switching of the
    // active calibration mode, ignore the button push
    if (g_Blinks)
        return;

    if (bLongDuration)
        if (!g_bCalMode)
        {
            g_bShouldEnterCal = 1;
            return;
        }
    
    if (g_bCalMode)
    {
        // Turn off the LED
        g_bLEDON = 0;
                
        g_bShouldExitCal = 1;
        return;
    }
    
    // Select next mode
    if (g_CalRegion < 3)
        g_CalRegion++;
    else
        g_CalRegion = 0;
    
    // Blink the LED
    g_Blinks = g_CalRegion + 1;
    TMR1 = 0;
    T1CONbits.TON = 1;
}

/* Initialize User Ports/Peripherals */
void InitApp(void)
{
    /* Setup analog functionality and port direction */
    
    TRISBbits.TRISB0 = 1;  // RB0 as input (we actually use it as AN0)
    TRISBbits.TRISB1 = 1;  // RB1 as input (we actually use it as AN1)
    
    // Initialize port B as all inputs
    TRISB = 0x3FF;
    
    TRISCbits.TRISC15 = 0; // Output LED
    TRISCbits.TRISC13 = 1; // Input (T2CK gate) Push button
    TRISCbits.TRISC14 = 1; // Input (CN0) SEL A
    
    TRISFbits.TRISF2 = 1;  // Input PGC (used for debugging)
    TRISFbits.TRISF3 = 1;  // Input PGD (used for debugging)
    TRISFbits.TRISF4 = 1;  // Input (CN17) SEL B
    TRISFbits.TRISF5 = 1;  // Input (CN18) SEL C
    TRISFbits.TRISF6 = 1;  // Input (INT0) CS
    
    TRISDbits.TRISD8 = 1;  // Input (INT1) RD
    TRISDbits.TRISD9 = 1;  // Input (INT2) WR
    
    ADPCFGbits.PCFG0 = 0;  // AN0 enabled as Analog input
    ADPCFGbits.PCFG1 = 0;  // AN1 enabled as Analog input
    ADPCFGbits.PCFG2 = 1;  // AN2 enabled as Digital I/O
    ADPCFGbits.PCFG3 = 1;  // AN3 enabled as Digital I/O
    ADPCFGbits.PCFG4 = 1;  // AN4 enabled as Digital I/O
    ADPCFGbits.PCFG8 = 1;  // AN8 enabled as Digital I/O
    ADPCFGbits.PCFG9 = 1;  // AN9 enabled as Digital I/O

    // Weak pull-up resistors for CN pins
    CNPU1bits.CN0PUE = 0;
    CNPU1bits.CN1PUE = 1;  // Push button
    CNPU1bits.CN2PUE = 0;
    CNPU1bits.CN3PUE = 0;
    CNPU1bits.CN4PUE = 0;
    CNPU1bits.CN5PUE = 0;
    CNPU1bits.CN6PUE = 0;
    CNPU1bits.CN7PUE = 0;
    CNPU2bits.CN17PUE = 0;
    CNPU2bits.CN18PUE = 0;
        
    // Change notification interrupts
    CNEN1bits.CN0IE = 1;  // CN interrupt for SEL A (PORTC:14)
    CNEN1bits.CN1IE = 1;  // CN interrupt for Push Button (PORTC:13)
    CNEN1bits.CN2IE = 0;  // Not used. (AN0 input instead)
    CNEN1bits.CN3IE = 0;  // Not used. (AN1 input instead)
    CNEN1bits.CN4IE = 0;  // Not used. (RB2 / D0 output instead)
    CNEN1bits.CN5IE = 0;  // Not used. (RB3 / D1 output instead)
    CNEN1bits.CN6IE = 0;  // Not used. (RB4 / D2 output instead)
    CNEN1bits.CN7IE = 0;  // Not used. (RB5 / D3 output instead)

    CNEN2bits.CN17IE = 1; // CN interrupt for SEL B (PORTF:4)
    CNEN2bits.CN18IE = 1; // CN interrupt for SEL C (PORTF:5)
    
    /* Initialize peripherals */
    
    // Configure Timer1 and Timer2 for normal operation
    T1CON = 0;
    T2CON = 0;
    T1CONbits.TGATE = 0;
    T1CONbits.TCS = 0;
    T1CONbits.TCKPS = 3;  // 1:256 prescale
    T1CONbits.TON = 0;
    T2CONbits.TGATE = 0;
    T2CONbits.T32 = 1;
    T2CONbits.TCKPS = 3;  // 1:256 prescale
    T2CONbits.TCS = 0;
    T2CONbits.TON = 0;

    // NOTE: In 32bit configuration the Timer2 and Timer3 act as one.
    // The T2CON is used for configuration and the T3 interrupt is acting
    // as the combined 32bit timer2/3 interrupt.
    
    // Clear the timers
    TMR1 = 0;
    TMR2 = 0;
    TMR3 = 0;
    // Load the period register
    PR1 = 0xFFFF;
    PR2 = 0xFFFF;
    PR3 = 0x0008;
    
    // Setup Timer1 and Timer2/Timer3 interrupts    
    _T1IF = 0;
    _T1IE = 1;
    _T2IF = 0;
    _T2IE = 0;
    _T3IF = 0;
    _T3IE = 1;
    
    //Setup the CN interrupt   
    
    // Clear the CN interrupt flag
    _CNIF = 0;
    // Enable CN interrupts
    _CNIE = 1;      

    // Configure the ADC
    configADC();
    
    // Initialize the last known button state
    g_bButtonState = PORTCbits.RC13;
    
    // disable nested interrupts
    INTCON1bits.NSTDIS = 1;   
}

