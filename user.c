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

uint16_t readADC(int channel)
{
    unsigned int ch, result, i;
    
    ADCON1bits.ADON = 0;         /* turn off ADC */
    if (channel == 0)
        ch = ADC_CH0_POS_SAMPLEA_AN0;  
             //& ADC_CH0_NEG_SAMPLEA_NVREF;
    else if (channel == 1)
        ch = ADC_CH0_POS_SAMPLEA_AN1; 
             //& ADC_CH0_NEG_SAMPLEA_NVREF;
      
    // Select the input channel to convert
    SetChanADC12(ch);
    
    // Setup the ADC
    OpenADC12(g_ADC_Adcon1_reg, 
              g_ADC_Adcon2_reg,
              g_ADC_Adcon3_reg, 
              g_ADC_PinConfig, 
              g_ADC_Scanselect);

    __builtin_disable_interrupts();
    
    LATCbits.LATC15 = !g_bLEDON;

    // Init the S&H phase
    ADCON1bits.SAMP = 1;
    while(ADCON1bits.SAMP)
        ;
    
    // Init the convesion
    ConvertADC12();
    
    // Wait till processing is done
    while(ADCON1bits.SAMP)
        ;
    __delay_us(10);
    //while(!BusyADC12());
    while(BusyADC12())
        ;

    // Get the result
    result = ReadADC12(0);
    
    LATCbits.LATC15 = g_bLEDON;
    
    __builtin_enable_interrupts();
        
    return result;
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

    // Clear the timers
    TMR1 = 0;
    TMR2 = 0;
    // Load the period register
    PR1 = 0xFFFF;
    PR2 = 0xFFFF;
    PR3 = 0x0008;
    
    // Setup Timer1 and Timer2 interrupts    
    _T1IF = 0;
    _T1IE = 1;
    _T2IF = 0;
    _T2IE = 0;
    _T3IF = 0;
    _T3IE = 0;
    
    //Setup CN and INT0 - INT2 interrupts   
    
    // Clear the CN interrupt flag
    _CNIF = 0;
    //_CNIP = 7;
    // Enable CN interrupts
    _CNIE = 1;      

    // We are not using the ADC interrupts
    ConfigIntADC12(ADC_INT_DISABLE);
    
    // Initialize the last known button state
    g_bButtonState = PORTCbits.RC13;
}

