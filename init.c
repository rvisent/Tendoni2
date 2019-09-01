//-----------------------------------------------------------------------------
// init.c
// TENDONI V2
// rev1 - RV110319
// initialization code and timer IRQ routine
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "C8051F350.H"				// SFR declarations
#include "main.h"
#include "F35x_ADC0.h"


//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------
void SYSCLK_Init(void);
void PORT_Init(void);
void Timer2_Init(int counts);


//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------
volatile unsigned char seconds_cnt=0;
volatile unsigned short tm0_cnt=0;
volatile unsigned char WDcnt = 10;


// we need to stop watchdog during sdcc init code, because clock is slow and
//   otherwise we get a watchdog reset before main() is called...
unsigned char _sdcc_external_startup()
{
	// disable watchdog timer
	PCA0MD &= ~0x40;					// WDTE = 0 (clear watchdog timer enable)

	// continue startup normally
	return 0;
}


void init(void)
{
	// enable VDD monitor and missing clock detector as reset sources
    int i=0;
    VDM0CN = 0x80;
    for (i=0; i<350; i++);  // Wait 100us for initialization
    RSTSRC = 0x06;

	PORT_Init();						// Initialize crossbar and GPIO
	SYSCLK_Init();						// Initialize system clock

	Timer2_Init(SYSCLK / 12 / 40);		// Init Timer2 to generate
										//   interrupts at a 40Hz rate.

	ADC0_Init();						// Initialize 24 bit A/D

	// enable and lock WD timer at 32 ms (max with our clock)
    PCA0MD    &= ~0x40;
    PCA0MD    = 0x00;
    PCA0CPL2  = 0xFE;
    PCA0MD    |= 0x20;

	// reset watchdog
	PCA0CPH2 = 0;

	EA = 1;								// enable global interrupts

/*
	FLASH_Init();						// initialize FLASH memory I/O

	// load parameters from FLASH
	// if valid, initialize global vars
	// otherwise, use default initial values for variables
	if (flash.magic_code == 0x5432)
	{
		FLASH_BufRead(FLASH_STORE+2, (unsigned char *)&ramparam, sizeof(ramparam));
	}
*/
}


//-----------------------------------------------------------------------------
// PORT_Init
//-----------------------------------------------------------------------------
//
// Configure the Crossbar and GPIO ports.
//
void PORT_Init (void)
{
	// set P0 as digital inputs with weak pullup (all reset defaults)
	//P0MDIN = 0xFF;
	//P0SKIP = 0x00;
	//P0MDOUT = 0x00;
	//P0 = 0xFF;

	// enable digital outputs P1.0-P1.4 and DAC IDA0 (on P1.6)
	P1MDIN = 0xBF;		// not sure this is necessary for IDA0
	P1SKIP = 0x40;		// required for IDA0, not mapped on XBAR
	P1MDOUT = 0x1F;		// P1.0-P1.4
	P1 = 0x0A;			// all off (TRIAC_OFF and LEDR have reverse logic)

	// crossbar Initialization
	XBR0    = 0x00;
	XBR1    = 0x50;		// enable T0 on P0.0, crossbar and weak pull-ups

	// enable TIMER0 as 16 bit counter with clock from P0.0
	// timer
	TMOD = 0x05;		// counter mode 1, GATE0=0
	TCON = 010;			// enable counter/timer TR0=1
}


//-----------------------------------------------------------------------------
// SYSCLK_Init
//-----------------------------------------------------------------------------
//
// This routine initializes the system clock to use the internal
// oscillator as its clock source. Divide by 1, multiply off to get 24.5 MHz
// Also enables missing clock detector reset.
//
void SYSCLK_Init (void)
{
    OSCICN = 0x83;
	RSTSRC = 0x04;				// enable missing clock detector
}


//-----------------------------------------------------------------------------
// Timer2_Init
//-----------------------------------------------------------------------------
//
// Configure Timer2 to 16-bit auto-reload and generate an interrupt at
// interval specified by <counts> using SYSCLK/48 as its time base.
//
void Timer2_Init (int counts)
{
   TMR2CN  = 0x00;                        // Stop Timer2; Clear TF2;
                                          // use SYSCLK/12 as timebase
   CKCON  &= ~0x60;                       // Timer2 clocked based on T2XCLK;

   TMR2RL  = -counts;                     // Init reload values
   TMR2    = 0xffff;                      // set to reload immediately
   ET2     = 1;                           // enable Timer2 interrupts
   TR2     = 1;                           // start Timer2
}


//-----------------------------------------------------------------------------
// Interrupt Service Routines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Timer2_ISR
//-----------------------------------------------------------------------------
// This routine measures time
//
void Timer2_ISR(void) __interrupt 5 __using 1
{
	static unsigned char cnt = 0;
	static unsigned short tm0_cnt_old = 0;
	static __bit bLEDG = 0;

	TF2H = 0;		// clear Timer2 interrupt flag

	// visual indication of status
	//  up, not auto: LEDG off
	//  down, not auto: LEDG flashes 1s on / 1s off
	//  down, auto: LEDG flashes 0.1s on / 1.9s off
	//  up, auto-down, alarm off (waiting 4 hours): 0.1s on / 0.2s off / 0.1s on / 1.6s off
	//  up, auto-down, but alarm still on: continuous fast flash
	// plus, momentary pulse to signal operation of wind sensor

	// increment 40 Hz counter
	cnt++;

	// reset watchdog (watchdog timer = 32 ms, we run at 25 ms), unless we have problems
	//   in main() routine
	if (WDcnt)
	{
		// any write is ok
		PCA0CPH2 = 0;
		// give some time to reload to main routine
		WDcnt--;
	}

	// timed actions: tick is 0.1s, or cnt/4
	// see LEDG.xls for details
	// for LEDG, work on static internal bit, so we can mess the value shown later
	if (!(cnt & 3))
	{
		switch (cnt>>2)
		{
		case 1:
		case 5:
		case 7:
		case 9:
			bLEDG = !bAutoDown && bDown;
			break;

		case 3:
			bLEDG = (!bAutoDown && bDown) || (bAutoDown && !bDown && auto_down_timer<FOUR_HOURS);
			break;

		case 2:
		case 4:
		case 6:
		case 8:
			bLEDG = (!bAutoDown && bDown) || (bAutoDown && !bDown && auto_down_timer==FOUR_HOURS);
			break;

		//case 10:	// handled by its own
		case 12:
		case 14:
		case 16:
		case 18:
			bLEDG = bAutoDown && !bDown && auto_down_timer==FOUR_HOURS;
			break;

		case 11:
		case 13:
		case 15:
		case 17:
		case 19:
			bLEDG = 0;
			break;

		case 20:	// cnt=80==0
			// 2s, reset counter
			cnt = 0;
			// turn on LEDG down or auto
			bLEDG = bDown || bAutoDown;
			// fall also into 1s actions 
			// break intentionally missing!

		case 10:
			// 1s actions
			// update external TIMER0 counter every 1s
			tm0_cnt = tm0_cnt_old;

			// increment seconds counter
			seconds_cnt++;

			// for LED, handle only the case cnt==40 (also 80 is arriving here)
			if (cnt == 40)
				bLEDG = bAutoDown && !bDown && auto_down_timer==FOUR_HOURS;
			break;
		}
	}

	// set LEDG
	LEDG = bLEDG;

	// acquire TIMER0 count (not sure if we need to disable/reenable timer when using
	//   16 bits readout: can't understand from documentation)
	// disable
	TCON = 0;
	// if timer changed, signal with irregular pulses of green LED
	if (TMR0 != tm0_cnt_old)
	{
		// reverse green LED (visible externally) if tent is down
		// will be restored from bLEDG on next cycle
		if (bDown)
			LEDG = !LEDG;
		tm0_cnt_old = TMR0;
	}
	// reenable timer
	TCON = 0x10;
}
