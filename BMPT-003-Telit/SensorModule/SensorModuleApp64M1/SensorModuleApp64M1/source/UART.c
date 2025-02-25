

#include "UART.h"
#include "ASIP.h"

#define WIFI_TX_BUFFER_SIZE         200

static BYTE wifiTxBuffer[WIFI_TX_BUFFER_SIZE]; // Outgoing data buffer
static BYTE wifiTxBufferHead; // Oldest data in the buffer is at this index
static BYTE wifiTxBufferTail; // Newest data goes here

static BOOL scfTX_Active;

static BYTE expectedWifiResponse = 0;
static BYTE scbUartBaud = BAUD_RATE_115200_16MHZ;//BAUD_RATE_115200_16MHZ;//BAUD_RATE_9600_16MHZ;
static BYTE scbDivisor = BAUD_RATE_115200_DIVISOR_16MHZ; //Default is 32
void initUART(void)
{
	wifiTxBufferHead = 0;
	wifiTxBufferTail = 0;
	
	scfTX_Active = FALSE;
	
	LINCR = _BV(LSWRES);
	
	/** LINBTR the register default value is 32 we need to make it accurate as per the calculation to support 115200 kbps 
	*  @info: Make sure this is initialized before the LENA = 1 and LENA should be 0 
	   LDISR will let to modify the Divisor Value 
	   @ref: Datasheet Topic : 17.5.6.1 Baud rate Generator
							   17.5.6.3 Handling LBT[5..0]
	*/
	LINBTR  = (1 << LDISR) | scbDivisor;
    
    // Module enabled, UART Tx/Rx enabled, 8/N/1
    LINCR |= (_BV(LENA) | (_BV(LCMD2)) | (_BV(LCMD1)) | (_BV(LCMD0))); 
    
    // Config baud rate...
    // LINBTR defaults to 32 samples/bit
    // LINBRR => ((FOSC) / (scbDivisor * BAUDRATE)) - 1
	LINBRR = scbUartBaud;
	
    // Enable interrupts
    LINENIR |= ((_BV(LENTXOK)) | (_BV(LENRXOK) | (_BV(LENERR))));
	SET_WIFI_CTS_LOW;
    ENABLE_UART_SHIFTERS;
}

BOOL getIsTxIdle(void)
{
    return ((wifiTxBufferHead == wifiTxBufferTail) && !(LINSIR & LBUSY));
}


ISR(LIN_ERR_vect)
{
    TOGGLE_ROW2_LED;
    LINSIR |= _BV(LERR);
    LINCR |= 0x80; // reset
    initUART();		
}

ISR(LIN_TC_vect)
{
	BYTE data;
	
	// For Rx...
    if(LINSIR & (_BV(LRXOK)))
    {
		// Read the data out of the receive FIFO
		data = LINDAT;

        stepAsipStateMachine(data);
	
		// Clear the IRQ
        LINSIR = _BV(LRXOK);
    } 
	
    // For Tx...
    if(LINSIR & (_BV(LTXOK)))
    {
		if(wifiTxBufferHead != wifiTxBufferTail)
		{
			// Push the next byte into the transmit FIFO
			LINDAT = wifiTxBuffer[wifiTxBufferHead++];
			
			if(wifiTxBufferHead >= WIFI_TX_BUFFER_SIZE)
			{
				wifiTxBufferHead = 0;
			}
		}
		else
		{
			scfTX_Active = FALSE;
		}
		// Clear the IRQ
		LINSIR = _BV(LTXOK);
    } 	     
}


BYTE getUART_TxBytesFree(void)
{
	BYTE free_bytes = 0;
	
	cli();
	{
		if(wifiTxBufferHead > wifiTxBufferTail)
		{			
			free_bytes = wifiTxBufferHead - wifiTxBufferTail - 1;
		}
		else if(wifiTxBufferTail > wifiTxBufferHead)
		{
			free_bytes = WIFI_TX_BUFFER_SIZE - wifiTxBufferTail + wifiTxBufferHead - 1;
		}
		else
		{
			free_bytes = WIFI_TX_BUFFER_SIZE - 1;
		}
	}
	sei();
	
	return free_bytes;
}

BOOL getUARTTxBuffEmpty(void)
{
	return (wifiTxBufferTail == wifiTxBufferHead);
}

BOOL addBytesToWifiTxBuffer(const BYTE * data, BYTE size)
{
    BYTE i;

	// Make sure there is enough room for the data
	if(size > getUART_TxBytesFree())
	{		
        return FALSE;			
	}	
	
	cli();
	{
		// Add data to buffer
		for(i = 0; i < size; i++)
		{
			wifiTxBuffer[wifiTxBufferTail++] = *data++;
			if(wifiTxBufferTail >= WIFI_TX_BUFFER_SIZE)
			{
				wifiTxBufferTail = 0;
			}
		}	
		
		// If the IRQ isn't going already, make it start
		if(scfTX_Active != TRUE)
		{
			scfTX_Active = TRUE;
			
			LINDAT = wifiTxBuffer[wifiTxBufferHead++];						
			if(wifiTxBufferHead >= WIFI_TX_BUFFER_SIZE)
			{
				wifiTxBufferHead = 0;
			}
		}
	}	
	sei();
	
	return TRUE;
}

void setExpectedWifiResponse(BYTE val)
{
    expectedWifiResponse = val;
}

