

#include "UART.h"
#include "ASIP.h"


void initUART(void)
{
    LINCR = 0;
    
    // Module enabled, UART Tx/Rx enabled, 8/N/1
    LINCR |= (_BV(LENA) | (_BV(LCMD2)) | (_BV(LCMD1)) | (_BV(LCMD0))); 
    
    // Config baud rate...
    // LINBTR defaults to 32 samples/bit
    // LINBRR => ((FOSC x 1000) / (32 * BAUDRATE)) - 1
    LINBRR = BAUD_RATE_9600_8MHZ;
    
    // Enable interrupts
    LINENIR |= ((_BV(LENTXOK)) | (_BV(LENRXOK)));
}

BYTE getUARTData(void)
{
    return LINDAT;
}

void putUARTData(BYTE data)
{
    LINDAT = data;
}

ISR(LIN_TC_vect)
{
    // For Tx...
    if(LINSIR & (_BV(LTXOK)))
    {
        /*if(more data to tx)
        {
            putUARTData(next byte);
        }
        else
        {*/
            // Clear flag by writing a '1'
            LINSIR |= _BV(LTXOK);
        //}
    }
    // For Rx...
    else if(LINSIR & (_BV(LRXOK)))
    {
        stepAsipStateMachine(getUARTData());   
    }        
}

