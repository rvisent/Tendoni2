// F35x_ADC0.h
// TENDONI V2
// rev1 - RV110402

#ifndef _ADC0_H_
#define _ADC0_H_

//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------

#define N_ADCHANNELS 4		// DACOUT, WDET, WINDSENS, RAINSENS
#define DA_PERIOD 12

//-----------------------------------------------------------------------------
// Global FUNCTIONS
//-----------------------------------------------------------------------------

void ADC0_Init(void);		// Initialize ADC0
unsigned short getAD(unsigned char ch);

//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------


extern volatile unsigned short adValue[N_ADCHANNELS];	// acquired AI

#endif // _ADC0_H_
