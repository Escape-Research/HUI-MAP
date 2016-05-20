/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

extern __psv__ char __attribute__((space(psv))) map_lo[8][1024]; 
extern __psv__ char __attribute__((space(psv))) map_hi[8][512]; 

extern char temp_map_lo[1024];
extern char temp_map_hi[512];

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

// Are we ready to start? (CS and WR signals low)
extern char g_breadyToStart;


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

char getFaderNum();

uint16_t readADC(int channel);

void InitApp(void); /* I/O and Peripheral Initialization */
