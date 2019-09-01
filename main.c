//-----------------------------------------------------------------------------
// main.c
// TENDONI V2
// rev1 - RV110522
// rev1.1 - RV110531
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "C8051F350.H"			// SFR declarations
#include <stdio.h>
#include "main.h"
#include "F35x_ADC0.h"

//-----------------------------------------------------------------------------
// IRQ declarations must stay in module containing main()
//-----------------------------------------------------------------------------
void Timer2_ISR(void) __interrupt 5 __using 1;
void ADC0_ISR (void) __interrupt 10 __using 2;

//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------
volatile __bit bDown = 1;		// goes to zero after an alarm
volatile __bit bAutoDown = 1;	// goes to zero after pressing of buttons
unsigned short ad[4];			// A/D readings
unsigned short prev_seconds=0xFFFF;
unsigned short prev_counter=0;
unsigned short water_threshold=0, wd_th_prev1=0, wd_th_prev2=0, water_min=65535;
unsigned char water_cnt=0, wind_timer[WIND_GUST_EVENTS-1] = { 0, 0, 0, 0 };
volatile unsigned short auto_down_timer = 0;

// flash persistent data with defaults
// defaults force allocation, so the linker respects the area (512 byte flash page)
//__code __at(FLASH_STORE) struct FLASHDATA flash = { 0x5432, { 0, 1.5e-3, 0 } };

// avoid overwriting security lock byte
//__code __at(0x1DFF) unsigned char lock_byte = 0xFF;

//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------
void main(void);
char move_updown(char bUp);
void alarm_reset();


//-----------------------------------------------------------------------------
// MAIN Routine
//-----------------------------------------------------------------------------
void main(void)
{
	__bit alarm, bButtonDown;

	// various initializations
	init();

	while (1)
	{
/*		// test
		unsigned char i;
		static unsigned short cnt = 0;
		for (i=0; i<N_ADCHANNELS; i++)
			ad[i] = getAD(i);

		if (cnt++ == 10000)
		{
			cnt = 0;
		}
		// test for DI/DO
		// set main relays when DN monitor is pressed
		RL_AUTO = !DI_DOWN;
		// set also TRIAC: in this case logic is reverse
		TRIAC_OFF = DI_DOWN;
		// set also extra relay for down
		RL_DOWN = !DI_DOWN;

		// change mode
		if (!DI_DOWN)
			bAutoDown = 0;
*/

		// reset alarm condition
		alarm = 0;

		// check if tent is manually actuated
		// check here, faster rate than 1s
		if (!DI_DOWN)
		{
			// manually commanded, assume down and exit automatic mode
			bAutoDown = 0;
			bDown = 1;
			// clear events memory for alarm detection
			alarm_reset();
			// set flag for water alarm relax
			bButtonDown = 1;
		}
		else
			bButtonDown = 0;

		// check if 1s has passed, in that case read A/D and counter
		if (seconds_cnt != prev_seconds)
		{
			__bit wind_pre, water_pre;	// pre-alarms
			
			// reset pre-alarms
			wind_pre = 0;
			water_pre = 0;

			// check WIND
			{
				unsigned char delta_counter, dc_th;

				// read WIND SENSOR with interrupts disabled
				EA = 0;
				// if more than one second passed, then ignore (by clear) wind reading. Almost certainly
				//   caused by a previous actuation of tents
				if (seconds_cnt != prev_seconds+1)
					delta_counter = 0;
				else
				{
					// normal condition, 1s has passed
					if (tm0_cnt-prev_counter > 255)
						// very unlikely, but...
						delta_counter = 255;
					else
						delta_counter = (unsigned char)(tm0_cnt-prev_counter);
				}
				prev_counter = tm0_cnt;
				prev_seconds = seconds_cnt;
				EA = 1;

				// read threshold from pot and compare: pre-alarm if threshold passed
				// set monitored range to 8-39 ticks per second (full CW: max sensitivity)
				dc_th = 39-(unsigned char)(getAD(2) >> 11);
				wind_pre = delta_counter > dc_th;
			}

			// check water: ratio of p-p measurement after and before R29
			// use dynamic threshold to allow reduced sensitivity after an alarm or
			//   after manual command down in case of sensor not completely dry
			{
				unsigned short wd, wd_th, wd_a, wd_b;
				short wd_th_delta;

				wd_b = getAD(0);
				wd_a = getAD(1);
				if (wd_b != 0)
					wd = (unsigned short)(wd_a*65536L/wd_b);
				else
					wd = 65535;
				water_pre = wd < water_threshold;

				// update threshold according to status
				// with R29=22k we have for wd:
				// short:5800, open:50447, 1k:8800, 10k:23700, 100k:35200
				// a good value seems to be around 14k, so we allow a range 8192-40960
				// wd_th is the user setpoint
				wd_th = (getAD(3) >> 1)+8192;

				// check if manually changed by rotating the pot: in this case align
				//   water_threshold with setpoint, otherwise calibration becomes difficult
				// compare with value 2s before, to be reasonably sure to catch trimmer rotation
				// (we monitor variation over last 2 cycles, but we repeat check on each cycle)
				wd_th_delta = (short)(wd_th-wd_th_prev2);
				if ((wd_th_delta > 1000) || (wd_th_delta < -1000))
				{
					// reset threshold
					water_threshold = wd_th;
				}
				wd_th_prev2 = wd_th_prev1;
				wd_th_prev1 = wd_th;

				// threshold adaptation algorithm
				if (bDown)
				{
					// if we are in manual mode with button down just pressed,
					//   set a threshold that allows the tent to remain down
					if (bButtonDown)	// manual mode is implicit
					{
						// force a threshold lower than current measure,
						//   so if commanded down it will stay there if conditions
						//   don't get worse
						water_threshold = wd-1000;
						// however, not higher than setpoint
						if (water_threshold > wd_th)
							water_threshold = wd_th;
					}
					else
					{
						// normal or automatic mode, but button not pressed
						// tent is down, threshold should gradually reach wd_th to restore
						//   maximum sensitivity
						if (water_threshold < wd_th)
						{
							// we have a lower threshold, due to a previous alarm or
							//   to manual command down with wet sensor
							// if actual measure has gone higher than user setpoint wd_th, restore it
							//   (sensor is finally dry), otherwise keep reduced threshold
							// keep some margin, to avoid getting an alarm on
							//   following cycles due to noise
							if (wd > wd_th+5000)
								// final update
								water_threshold = wd_th;
							else if (wd > water_threshold+5000)
								// gradually increase threshold while sensor dries
								water_threshold += 1000;
						}
						else
							// wd_th probably changed by rotating pot, straight copy
							water_threshold = wd_th;

						// reset sensor minimum reading
						water_min = 65535;
					}

				}
				else
				{
					// tent is up
					// different water threshold for manual and automatic modes
					if (bAutoDown)
					{
						// update minimum reading and threshold
						if (wd < water_min)
						{
							water_min = wd;
							// put threshold at mid between user setpoint and minimum reached
							// divide before add to avoid integer overflow
							water_threshold = (wd_th>>1)+(water_min>>1);
						}
					}
					else
						// manual mode, threshold doesn't matter, because it will be
						//   reset when button down is pressed.
						// reset it to setpoint to simplify tuning of pot looking at LEDR
						water_threshold = wd_th;
				}
			}

			// set LEDR (warning LED) on pre-alarm
			LEDR = (wind_pre || water_pre) ? 0:1;

			// handle alarm conditions: WATER_ALM_TIME s consecutive for water,
			//   5 times in WIND_GUST_TIME s for wind
			// do even if tents are up, because it is required by automatic mode
			{
				unsigned char iWind, nWindEvents, iFreeSlot;
				water_cnt = water_pre ? water_cnt+1 : 0;
				if (water_cnt >= WATER_ALM_TIME)
					alarm = 1;

				// decrease all active wind timers, count active ones
				nWindEvents = 0;
				for (iWind=0; iWind<WIND_GUST_EVENTS-1; iWind++)
					if (wind_timer[iWind] > 0)
					{
						wind_timer[iWind]--;
						// ignore the case of timer gone to zero now, unsignificant difference
						nWindEvents++;
					}
					else
						// copy index of free slot (we'll get the last one)
						iFreeSlot = iWind;
				
				// pre-alarm ?
				if (wind_pre)
				{
					if (nWindEvents >= WIND_GUST_EVENTS-1)
						// this was pre-alarm #WIND_GUST_EVENTS in WIND_GUST_TIME s -> WIND ALARM
						alarm = 1;
					else
						// load one free timer with WIND_GUST_TIME s timeout
						wind_timer[iFreeSlot] = WIND_GUST_TIME;
				}
			}

			// now different behaviour with tents up or down
			if (bDown)
			{
				// if alarm -> tents up
				if (alarm)
				{
					if (move_updown(1) == -1)
						// interrupted by user: go to manual mode, assume we are still down
						// (assuming to be down is the safest choice)
						// WARNING: on next loop bButtonDown will be probably set and water
						//   threshold changed (may not be what human wants...)
						bAutoDown = 0;
					else
					{
						// went up without interruptions: keep current auto/manual mode
						bDown = 0;
						// load timer for automatic mode with 4 hours (3600*4 s)
						auto_down_timer = FOUR_HOURS;
					}
	
					// clear events memory for alarm detection
					alarm_reset();
				}
			}
			else
			{
				// tents are up
				// restart timer for automatic mode in case of alarms
				if (alarm)
					auto_down_timer = FOUR_HOURS;
				else
				{
					// decrement timer in automatic mode
					if (bAutoDown)
					{
						if (auto_down_timer)
							auto_down_timer--;
						else
						{
							// timer has elapsed: tents can go down now, after 4 hours without alarms
							if (move_updown(0) == -1)
							{
								// interrupted by user: go to manual mode, assume we are down
								// (assuming to be down is the safest choice)
								// WARNING: on next loop bButtonDown will be probably set and water
								//   threshold changed (may not be what human wants...)
								bAutoDown = 0;
								bDown = 1;
							}
							else
								// tents went down without interruptions: remain in auto mode
								bDown = 1;
	
							// clear events memory for alarm detection
							alarm_reset();
						}
					}
				}
			}

		}	// end 1s timed loop


		// arrived here: restore soft watchdog counter
		WDcnt = SOFT_WD_COUNTS;

		// go idle until next interrupt to save power
		PCON = PCON_IDLE;
	}	// end while(1)

}


// command motor(s) to move up or down
// down requires 40s, up TBD
// check button to block motion return -1 if pressed
// if ok, return 0
char move_updown(char bUp)
{
	unsigned char s;
	char time_to_wait;
	__bit bBtnPressed = 0;

	// check button not pressed
	if (!DI_DOWN)
		return -1;

	// select relays according to required mode
	RL_AUTO = 1;
	RL_DOWN = bUp ? 0:1;

	// wait at least 1s, checking button (safe exit if pressed)
	// NO, don't check button here, we are safely disconnected from it and sometimes
	//   we get a glitch on DI_DOWN when the relay contact closes
	s = seconds_cnt;
	while ((char)(seconds_cnt-s) < 2) // && !bBtnPressed)
	{
		//bBtnPressed = !DI_DOWN;
		// we need to avoid watchdog resets
		WDcnt = SOFT_WD_COUNTS;
	}

	// actuate TRIAC, unless button was pressed
	// ok, now bBtnPressed is always == 0, but we leave the original code
	TRIAC_OFF = bBtnPressed;

	// now wait for completion of actuation, time is different according to direction
	// immediate exit if button was previously pressed
	s = seconds_cnt;
	time_to_wait = bUp ? TENTS_UP_TIME:TENTS_DOWN_TIME;
	while ((char)(seconds_cnt-s) < time_to_wait && !bBtnPressed)
	{
		bBtnPressed = !DI_DOWN;
		// go idle until next interrupt to save power
		// we need to avoid watchdog resets
		WDcnt = SOFT_WD_COUNTS;
		PCON = PCON_IDLE;
	}

	// terminate TRIAC actuation
	TRIAC_OFF = 1;
		
	// wait at least 1s, checking button
	// NO, don't check button, as above
	s = seconds_cnt;
	while ((char)(seconds_cnt-s) < 2) // && !bBtnPressed)
	{
		// bBtnPressed = !DI_DOWN;
		// we need to avoid watchdog resets
		WDcnt = SOFT_WD_COUNTS;
	}

	// now check if button is pressed, because we have removed the test above
	if (!DI_DOWN)
		bBtnPressed = 1;


	// if button was never pressed, we can safely turn off the relays and
	//   exit with success condition
	if (!bBtnPressed)
	{
		RL_DOWN = 0;
		RL_AUTO = 0;
		return 0;
	}

	// we must wait until button is released, then wait a further time
	//   before releasing the relays
	while (bBtnPressed)
	{
		bBtnPressed = 0;
		// wait at least 1s, checking button
		s = seconds_cnt;
		while ((char)(seconds_cnt-s) < 2 && !bBtnPressed)
		{
			bBtnPressed = !DI_DOWN;
			// we need to avoid watchdog resets
			WDcnt = SOFT_WD_COUNTS;
		}
	}

	// now release relays and exit with "button pressed" condition
	RL_DOWN = 0;
	RL_AUTO = 0;
	return -1;
}


void alarm_reset()
{
	unsigned char iWind;
	// reset variables for alarm detection
	// don't touch water thresholds
	water_cnt = 0;
	for (iWind = 0; iWind<4; iWind++)
		wind_timer[iWind] = 0;
}
