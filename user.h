/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

#define _12BIT_1Q   0x3FF
#define _12BIT_HALF 0x7FF
#define _12BIT_3Q   0xBFF
#define _12BIT_FS   0xFFF

#define _10BIT_1Q   0x0FF
#define _10BIT_HALF 0x1FF
#define _10BIT_3Q   0x2FF
#define _10BIT_FS   0x3FF

// The map calibration values (stored in the flash memory)
extern unsigned int _EEDATA(32) map_cal_eeprom1[16];
extern unsigned int _EEDATA(32) map_cal_eeprom2[16];

// Buffer in data memory used during runtime
extern unsigned int map_cal[8][4];

extern uint16_t g_nextOutput; // The next 10bit value to output
extern char g_bOutput2ndByte; // Have we already send out the 1st byte?

// Last known state of the push button
extern unsigned g_bButtonState;

// Current Fader selected
typedef struct tagSELBITS {
    unsigned SELA:1;
    unsigned SELB:1;
    unsigned SELC:1;
} SELBITS;
extern SELBITS g_SELbits;

// Are we in cal mode?
extern char g_bCalMode;

// Region to calibrate
extern char g_CalRegion;

// Are we ready to start? (CS and WR signals low)
extern char g_bReadyToStart;

// Flag that the button was pressed for a long duration
extern char g_bLongDuration;

// Flag to signal that we should enter calibration mode
extern char g_bShouldEnterCal;

// Flag to signal that we should exit calibration mode
extern char g_bShouldExitCal;

// Blink counter
extern unsigned g_Blinks;

// Flag to keep track of the LED ON state
extern char g_bLEDON;

/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/

void init_tempmap();
//void SaveTempMapToFlash();

void LoadCalFromEE();
void SaveCalToEE();

uint16_t map_location(int currFader, uint16_t fpos);

uint16_t scale_from_12_to_10bits(uint16_t value);

void HandleButton(char bLongDuration);

int getFaderNum();

void configADC();
uint16_t readADC();

void asm_ProcessRDRequest(int outputValue);

void InitApp(void); /* I/O and Peripheral Initialization */

/******************************************************************************/
/* User Assembly Function Prototypes                                                   */
/******************************************************************************/

extern void handleCS();

/******************************************************************************/
