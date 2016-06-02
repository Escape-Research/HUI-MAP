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
    
    _prog_addressT p_s_flag;
    _init_prog_address(p_s_flag, map_saved);
    _erase_flash(p_s_flag);
    _write_flash16(p_s_flag, map_saved_buffer);
    
}

int getFaderNum()
{
    int fader = (g_SELbits.SELC << 2) +
                 (g_SELbits.SELB << 1) +
                 (g_SELbits.SELA);
    return fader;
}

int getFaderNum2()
{
    int fader = (PORTFbits.RF5 << 2) +
                 (PORTFbits.RF4 << 1) +
                 (PORTCbits.RC14);
    return fader;
}

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
    
    ADCON3bits.SAMC = 0x01;  // 1 TAD (auto sample time)
    ADCON3bits.ADRC = 0;     // Clock derived from system clock
    ADCON3bits.ADCS = 19;    // Configure for 333nS TAD time
    
    ADCON2bits.CSCNA = 0;    // Do not scan inputs
    ADCON2bits.BUFM = 0;     // Buffer configured as one 16-word buffer ADCBUF(15..0)
    ADCON2bits.SMPI = 8;     // Collect 8 or 16 samples at a time!
    ADCON2bits.ALTS = 0;     // Don't alternate between MUX A and MUX B
    
    // Turn on the ADC
    ADCON1bits.ADON = 1;
    
    // Make sure that we'll wait for at least 20uS for ADC to stabilize
    __delay_us(20);
}

uint16_t readADC(int channel, int *pAltResult)
{
    uint16_t resultA, resultB;
    int count;
    uint16_t *ADC16Ptr;

    // We are collecting and averaging 8 samples
    
    if (channel == 0)
    {
        // We are only to sample AD0
        ADCON2bits.ALTS = 0;
        
        // Collect 8 samples at a time!
        ADCON2bits.SMPI = 8;
    }
    else
    {
        // We will sample AD0 and AD1 (alternate MUX)
        ADCON2bits.ALTS = 1;
        
        // Collect 16 samples at a time!
        ADCON2bits.SMPI = 0xF;
    }
        
    LATCbits.LATC15 = !g_bLEDON;

    // Initialize accumulation vars
    resultA = 0;
    resultB = 0;
    
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
    
    // Process result buffer
    if (channel == 0)
    {
        // We took 8 samples of AD0 (using MUX A)
        for (count = 0; count < 8; count++)
            resultA = resultA + *ADC16Ptr++;
        
        // Divide by 8 to calculate the average value
        resultA = resultA >> 3;
    }
    else
    {
        // We took 16 samples total alternating between MUX A and MUX B
        // (MUX A - AD0 and MUX B - AD1)
        for (count = 0; count < 16; count += 2)
        {
            resultA = resultA + *ADC16Ptr++;
            resultB = resultB + *ADC16Ptr++;
        }
        
        // Divide by 8 to calculate the averages
        resultA = resultA >> 3;
        resultB = resultB >> 3;
        
        if (pAltResult != NULL)
            *pAltResult = resultB;
    }
    
    LATCbits.LATC15 = g_bLEDON;
    
    return resultA;
}

void HandleButton(char bLongDuration)
{
    if (bLongDuration)
        if (!g_bCalMode)
        {
            g_bShouldEnterCal = 1;
            return;
        }
    
    if (g_bCalMode)
    {
        // Turn off the LED
        LATCbits.LATC15 = 0;
                
        g_bShouldExitCal = 1;
        return;
    }
    
    // Select next fader
    if (g_CalFader < 7)
        g_CalFader++;
    else
        g_CalFader = 0;
    
    // Blink the LED
    g_Blinks = g_CalFader + 1;
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
}

