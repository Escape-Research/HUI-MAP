/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

extern __psv__ char __attribute__((space(psv))) map_lo[8][1024]; 
extern __psv__ char __attribute__((space(psv))) map_hi[8][512]; 

extern char temp_map_lo[1024];
extern char temp_map_hi[512];

/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/

uint16_t getMap(char fader, uint16_t position);
uint16_t map_binary_search(char fader, uint16_t low_bound, uint16_t hi_bound, uint16_t value);
uint16_t map_approx_lookup(char fader, uint16_t lkValue);

void InitApp(void); /* I/O and Peripheral Initialization */
