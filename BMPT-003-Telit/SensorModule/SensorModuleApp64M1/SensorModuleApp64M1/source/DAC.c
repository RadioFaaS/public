
#include "DAC.h"

const WORD DAC_OUTPUT_2_5V = 512;


void initDAC(void)
{
    DACON = 0;
       ADMUX |= _BV(REFS0);
       ADMUX &= ~_BV(REFS1);
       ADCSRB |= _BV(AREFEN);
    // Config the output voltage value    
    setDAC(DAC_OUTPUT_2_5V);
    
    // Enable the DAC module and activate the output
    DACON |= (_BV(DAEN) | _BV(DAOE));        
}

// Configures the output voltage to the specified 10-bit value
// Vout = DAC * Vref/1023
void setDAC(WORD vout)
{
    DACL = (BYTE)(vout & 0xFF);
    DACH = (BYTE)((vout & 0x0300) >> 8);
}

// Returns the present output voltage of the DAC as a 10-bit value
WORD getDAC(void)
{
    WORD vout = 0;
    
    vout = DACL;
    vout |= ((WORD)(DACH & 0x03) << 8); 
    
    return vout;
}

