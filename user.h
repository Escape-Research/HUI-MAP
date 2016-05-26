/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

extern __psv__ char __attribute__((space(psv))) map_lo[8][1024]; 
extern __psv__ char __attribute__((space(psv))) map_hi[8][512]; 
extern __psv__ int  __attribute__((space(psv))) map_saved[32];

extern char temp_map_lo[1024];
extern char temp_map_hi[512];
extern int map_saved_buffer[32];

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

// Fader to calibrate
extern char g_CalFader;

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

uint16_t getMap(char fader, uint16_t position);
uint16_t gettempMap(uint16_t position);
uint16_t settempMap(uint16_t position, uint16_t value);
uint16_t map_binary_search(char fader, uint16_t low_bound, uint16_t hi_bound, uint16_t value);
uint16_t map_approx_lookup(char fader, uint16_t lkValue);
void init_tempmap();
void interpolate_tempmap();
void SaveTempMapToFlash(char fader);

void DisableDataOutput();
void EnableDataOutput();
void OutputByte(unsigned byteToSend);

void HandleButton(char bLongDuration);

char getFaderNum();

uint16_t readADC(int channel);

void InitApp(void); /* I/O and Peripheral Initialization */

/******************************************************************************/
/* User Assembly Function Prototypes                                                   */
/******************************************************************************/

extern void handleCS();