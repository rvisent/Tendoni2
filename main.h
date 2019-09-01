// main.h
// TENDONI V2
// rev1.2 - RV110602

#ifndef _MAIN_H_
#define _MAIN_H_

//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------

// define mode/location
//#define SOGGIORNO
#define MANSARDA
//#define TESTMODE

#define SYSCLK (24500000)	// SYSCLK frequency in Hz (internal oscillator)

#define DI_WIND P0_0		// wind sensor, mapped to counter T0
#define DI_DOWN P0_1		// =0 when any down button is pressed
#define RL_AUTO P1_0		// RL_AUTO=1 turns on the relays to exclude manual commands
#define TRIAC_OFF P1_1		// TRIAC_OFF=0 drives the TRIAC
#define LEDG P1_2			// LEDG=1 means GREEN LED ON
#define LEDR P1_3			// LEDR=0 means RED LED ON
#define RL_DOWN P1_4		// RL_DOWN=1 commands DOWN, otherwise UP

// flash position optimized to avoid large unused areas before (looking .map)
//#define FLASH_STORE (0x1000)	// user data in flash at 0x1A00-0x1BFF

// operational constants
#define WIND_GUST_TIME 60	// seconds for wind gust evaluation
#define WIND_GUST_EVENTS 5	// number of cycles over threshold in WIND_GUST_TIME to get alarm
#define WATER_ALM_TIME 4	// seconds of water pre-alarm to get alarm
#define SOFT_WD_COUNTS 4	// number of 25 ms IRQ cycles before WD resets us

// locations
#ifdef SOGGIORNO
#define FOUR_HOURS	14400	// seconds without alarm before automatic down is allowed
#define TENTS_UP_TIME 35	// time (s) to lift tents
#define TENTS_DOWN_TIME 15	// time (s) to lower tents
#endif

#ifdef MANSARDA
#define FOUR_HOURS	14400	// seconds without alarm before automatic down is allowed
#define TENTS_UP_TIME 40	// time (s) to lift tents
#define TENTS_DOWN_TIME 30	// time (s) to lower tents
#endif

// test constants
#ifdef TESTMODE
#define FOUR_HOURS	120 	// seconds without alarm before automatic down is allowed
#define TENTS_UP_TIME 10 	// time (s) to lift tents
#define TENTS_DOWN_TIME 6 	// time (s) to lower tents
#endif

//-----------------------------------------------------------------------------
// Global FUNCTIONS
//-----------------------------------------------------------------------------
void init(void);


//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------

/*
struct FLASHDATA
{
	short magic_code;		// FLASH ID=0x5432
	struct STOREDATA store;
	char filler[510-sizeof(struct STOREDATA)];	// fill a flash block to force linker to skip it
};
*/
//extern __code __at(FLASH_STORE) struct FLASHDATA flash;

// clock data
extern volatile unsigned short clock_mins;
extern volatile unsigned char seconds_cnt;
extern volatile unsigned short tm0_cnt;
extern volatile __bit bDown;		// goes to zero after an alarm
extern volatile __bit bAutoDown;	// goes to zero after pressing of buttons
extern volatile unsigned char WDcnt;// watchdog counter
extern volatile unsigned short auto_down_timer;	// used for fast flash


#endif // _MAIN_H_
