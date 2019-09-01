//-----------------------------------------------------------------------------
// F35x_ADC0.c, from F35x_ADC0_ExternalInput.c by SiLabs
//-----------------------------------------------------------------------------
// TENDONI V2
// rev1 - RV110402

#include "C8051F350.h"		// SFR declarations
#include "main.h"			// SYSCLK
#include "F35x_ADC0.h"

//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------

#define MDCLK 2450000	// Modulator clock in Hz (ideal is
						// (2.4576 MHz)
#define AD_T 20.062e-3	// A/D acquisition period in s (assuming ADC0DEC=383)

typedef union SHORTDATA
{							// access SHORTDATA as a
   unsigned short result; 	// short variable or
   char Byte[2];			// 2 byte variables
} SHORTDATA;

#define Byte1 1
#define Byte0 0

volatile unsigned short adFiltValue[N_ADCHANNELS];	// acquired and filtered AI
volatile unsigned short adPrevValue[2];				// previous AI for ch=0,1
// DAC output: constant around the A/D cycles 0 and 1 (ref and meas for water detector),
//   intermediate in the single remaining cycle. The A/D cycle is slow (no sampling?)
//   and uses a whole cycle, so we need a constant value one cycle before (for the
//   circuits to stabilize) and after (for the A/D)
// min is set to 0, max is set to 0.3mA, to give 2.04V on 6.8k (0.5mA f.s.)
//__code unsigned char da_val[DA_PERIOD] = { 154, 154, 77, 0, 0, 0, 77, 154 };

// DAC output: sinusoid of period 6 repeated two times (240/6 = 40 Hz)
// A/D is sampled on maxima (+ and -) in separate cycles for ch 0 and 1, so we get
//   the same timing for both channels (A/D acquisition lasts an entire cycle)
__code unsigned char da_val[DA_PERIOD] = { 154, 103, 51, 0, 51, 103, 154, 103, 51, 0, 51, 103 };
// this array defines the A/D channel currently acquired
__code unsigned char ad_ch_arr[DA_PERIOD] = { 0, 2, 3, 0, 2, 3, 1, 2, 3, 1, 2, 3 };


//-----------------------------------------------------------------------------
// ADC0_Init
//-----------------------------------------------------------------------------
void ADC0_Init (void)
{
   REF0CN |= 0x03;                     // enable internal Vref
   ADC0CF = 0x10;                      // internal VREF, interrupts upon FAST filter output

   // generate MDCLK for modulator, ideally MDCLK = 2.4576MHz
   // we have 24.5 MHz internal clock, so nearest MDCLK is 2.45 MHz
   ADC0CLK = (SYSCLK/MDCLK)-1;         

   // Program decimation rate for desired OWR
   // since we use fast filter, the rate (register+1) must be a multiple of 8
   // to get 240Hz, nearest choice is 10*8=80 (79 in register), which gives
   //   MDCLK/(128*80) = 239.26 Hz = 1/AD_T
   ADC0DEC = 79;

   ADC0BUF = 0x00;                     // turn off Input Buffers
   ADC0DAC = 0;						   // no DAC offset

   // calibrate range for g=1, single ended mode
   ADC0CN = 0x00;
   ADC0MD = 0x81;                      // start internal calibration
   while(AD0CALC != 1);                // wait until calibration is complete

   EIE1   |= 0x08;                     // Enable ADC0 Interrupts
   ADC0MUX = 0x08;                     // Select AIN0-GND
   ADC0MD  = 0x82;                     // Enable the ADC0 (single conversion mode)

   // enable the DAC
   IDA0CN = 0xF1;	// 0-5 mA f.s., enable IDA0, updates on write to register
}


// get data for one channel
// channels are rescaled to short range according to their FS
unsigned short getAD(unsigned char ch)

{
	unsigned short val;

	EA = 0;
	val = adFiltValue[ch];
	EA = 1;

	return val;
}


//-----------------------------------------------------------------------------
// ADC0_ISR
//-----------------------------------------------------------------------------
//
// The ISR is called after each ADC conversion.
//
//-----------------------------------------------------------------------------
void ADC0_ISR (void) __interrupt 10  __using 2
{
   static SHORTDATA rawValue;
   static unsigned char da_counter=0;
   unsigned char ad_ch;

   while(!AD0INT);                     // wait till conversion complete
   AD0INT = 0;                         // clear ADC0 conversion complete flag

   // copy the output value of the ADC, ignore LSB (keep 16 bits)
   rawValue.Byte[Byte1] = (unsigned char)ADC0FH;
   rawValue.Byte[Byte0] = (unsigned char)ADC0FM;
/*
	// code for 24 bits
   rawValue.Byte[Byte0] = (unsigned char)ADC0FL;
   // sign extend
   if (rawValue.Byte[Byte2] >= 0)
   		rawValue.Byte[Byte3] = 0;
   else
   		rawValue.Byte[Byte3] = 0xFF;
*/
	// get current A/D channel
	ad_ch = ad_ch_arr[da_counter];
	// for channels 0 and 1, compute 1st order difference and low-pass filter abs value
	// this because we have opposite DAC output at each cycle
	if (ad_ch < 2)
	{
		unsigned short temp;
		temp = adPrevValue[ad_ch];

		// compute(abs(diff(val)))
		if (rawValue.result > temp)
			temp = rawValue.result-temp;
		else
			temp -= rawValue.result;

		// use 32 bit math for filtering
		// for each channel we are running at (average) 240/6 = 40 Hz
		// (one 3/240 s cycle followed by 9/240 s -> 2 cycles in 12/240=1/20 s)
		// We want a time constant of 2s, so prev values at 1/n after 40*2=80 cycles
		// 1st order filter y(t)=a*y(t-1)+(1-a)*x(t); a=exp(-1/nCycles)
		// for nCycles=80 a=0.98758
		// we could use Q16, but we prefer Q15 to avoid problems with signed/unsigned long
		// in Q15 a_q15=0.98758*32768=32361; (1-a) becomes 32768-a_q15=407
		adFiltValue[ad_ch] = (unsigned short)((adFiltValue[ad_ch]*32361L + temp*407L) >> 15);

		// copy A/D value for next cycle
		adPrevValue[ad_ch] = rawValue.result;
	}
	else
	{
		// filter pots with 0.2s time constant
		// each measurement is taken every 3 cycles, or 80 Hz sampling
		// We want a time constant of 0.2s, so prev values at 1/n after 16 cycles
		// 1st order filter y(t)=a*y(t-1)+(1-a)*x(t); a=exp(-1/nCycles)
		// for nCycles=16 a=0.93941
		// in Q15 a_q15=0.92004*32768=30783; (1-a) becomes 32768-a_q15=1985
		adFiltValue[ad_ch] = (unsigned short)((adFiltValue[ad_ch]*30783L + rawValue.result*1985L) >> 15);
	}


	// prepare to acquire next channel
	da_counter++;
	if (da_counter == DA_PERIOD)
		da_counter=0;
	ad_ch = ad_ch_arr[da_counter];

	// always referred to AGND
	ADC0MUX = (ad_ch<<4) | 0x08;

	// change DAC using table
	IDA0 = da_val[da_counter];

	// start new conversion
	ADC0MD  = 0x82;				// enable the ADC0 (single conversion mode)
}
