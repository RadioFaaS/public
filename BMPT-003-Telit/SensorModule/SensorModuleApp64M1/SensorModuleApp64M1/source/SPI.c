/*
 * SPI.c
 *
 * Created: 8/8/2012 1:12:59 PM
 *  Author: sbailey
 */ 
#include "SPI.h"
#include "WiFiModule.h"
#include "ASIP.h"
#include <string.h>
//#include <avr/pgmspace.h>

#define WIFI_TX_BUFFER_SIZE         128/////////256 TODO: Restore this after debug...

#define SPI_RX_BUFFER_SIZE          64
#define GAINSPAN_CTRL_ESC_CHAR      0xFB

#define SENSOR_DATA_BUFFER_SIZE     66

static BYTE sensorDataBuffer[SENSOR_DATA_BUFFER_SIZE];
static BYTE sensorDataBufferHead;
static BYTE sensorDataBufferTail;

static BYTE spiRxBuffer[SPI_RX_BUFFER_SIZE];  // Incoming data buffer
static BYTE spiRxBufferHead; // Oldest data in the buffer is at this index
static BYTE spiRxBufferTail; // Newest data goes here

static BYTE wifiTxBuffer[WIFI_TX_BUFFER_SIZE]; // Outgoing data buffer
static BYTE wifiTxBufferHead; // Oldest data in the buffer is at this index
static BYTE wifiTxBufferTail; // Newest data goes here

static BYTE flowControlXoffFlag;
static BYTE spiTxSequenceInProgress;

static BYTE spiResponseReceivedFlag;

void initSPI(void)
{
    // Set SPI functions to Alternate I/O pins
    MCUCR |= _BV(SPIPS);
    
    // Enable SPI Master
    // Clock Rate => fck/16
    // Data Order => LSB transmitted first
    // Clock Polarity = 0, Leading Edge Rising / Trailing Edge Falling
    // Clock Phase = 0, Leading Edge Sample / Trailing Edge Setup
    SPCR = 0;
    SPCR = ((_BV(SPE)) | (_BV(MSTR)) | (_BV(SPR0)));  

    flowControlXoffFlag = 0;
    spiTxSequenceInProgress = 0;

    sensorDataBufferHead = 0;
    sensorDataBufferTail = 0;

    spiRxBufferHead = 0;
    spiRxBufferTail = 0;

    wifiTxBufferHead = 0;
    wifiTxBufferTail = 0;

    spiResponseReceivedFlag = 0;

    SET_WIFI_CS_LOW;  // Pull the WiFi CS -- can be left low since this is the only SPI device
}

// Transmits a specified number of bytes from a buffer
//   and stores the received data in the provided buffer
//   (ignored if the Rx buffer is NULL)
/*void transmitSPI(BYTE * txData, BYTE size, BYTE * rxData)
{
    BYTE i;
    
    //SET_WIFI_CS_LOW;
    
    for(i = 0; i < size; i++)
    {    
        SPDR = *txData++;
    
        // Wait for the transmission to complete
        while(!(SPSR & (1<<SPIF)));
        
        // Store the incoming byte
        if(rxData != NULL)
        {
            *rxData++ = SPDR;
        }        
    }    
    
    //SET_WIFI_CS_HIGH;
}
*/
// ISR for SPI transfer complete
ISR(SPI_STC_vect)
{
    // Get last incoming byte...
    spiRxBuffer[spiRxBufferTail++] = SPDR;
    
    if(spiRxBufferTail >= SPI_RX_BUFFER_SIZE)
    {
        spiRxBufferTail = 0;
    }
    
    // Must Tx if...
    //  WAKE-UP input is high
    //  Tx buff is not empty (and no XOFF flag)
    //  XOFF is received (must Tx idle (dummy) bytes, wait for XON)        

    if(IS_WIFI_WAKEUP_HIGH)
    {
        if((wifiTxBufferTail != wifiTxBufferHead) && (!flowControlXoffFlag))
        {
            // Tx buffer not empty
            SPDR = wifiTxBuffer[wifiTxBufferHead++]; // write next buffer byte
            spiTxSequenceInProgress = 1;
        }
        else
        {
            // Tx idle character
            spiTxSequenceInProgress = 1;
            SPDR = 0xF5;
        }
    }
    else if(flowControlXoffFlag)
    {
        // Do not send buffered Tx data, send Tx idle character instead
        SPDR = 0xF5;        
        spiTxSequenceInProgress = 1;
    }
    else
    {
        // Sequence done...   
        spiTxSequenceInProgress = 0;
    }

    /*if(getWifiReadyFlag() & ())
    {
        
    }*/
}

// Parse incoming WiFi data...
// This function processes flow control escape sequences and pushes data through the ASIP parser.
// If WiFi config responses are expected the data is examined for specific response strings.
void processReceivedWifiData(void)
{
    BYTE controlEscByteFound = 0;
    BYTE tempData;
    
    while(spiRxBufferHead != spiRxBufferTail)
    {
        if(controlEscByteFound)
        {
            controlEscByteFound = 0;

            tempData = ((spiRxBuffer[spiRxBufferHead]) ^ 0x20);

            if(tempData == 0xFD)
            {
                // XON cmd
                flowControlXoffFlag = 0;
            }
            else if(tempData == 0xFA)
            {
                // XOFF cmd
                flowControlXoffFlag = 1;
            }
            else if(tempData == 0xF5)
            {
                // Idle character
            }
            else if(tempData == 0xFB)
            {
                // Control Escape
                controlEscByteFound = 1;
            }
        }
        else if(spiRxBuffer[spiRxBufferHead] == GAINSPAN_CTRL_ESC_CHAR)
        {
            // Set control escape found flag
            controlEscByteFound = 1;
        }
        else
        {
            tempData = getExpectedWifiResponseType();
            // If config is in progress, check the response
            if(tempData)//(expecting config response)
            {
                if(parseIncomingGainspanData(spiRxBuffer[spiRxBufferHead], getExpectedWifiResponsePtr(), tempData))
                {
                    // Success!! Do something here...
                    spiResponseReceivedFlag = 1;
                }
            }
            else
            {
                // Parse 'normal' data -- send to ASIP processor
                // This stream will contain header/wrapper bytes but the ASIP parser will ignore these
                stepAsipStateMachine(spiRxBuffer[spiRxBufferHead]);
            }
        }
        
        spiRxBufferHead++;
        if(spiRxBufferHead >= SPI_RX_BUFFER_SIZE)
        {
            spiRxBufferHead = 0;
        }
    }
}

void addBytesToSensorDataBuffer(BYTE * data, BYTE size)
{
    BYTE i;
    BYTE bufferFullFlag = 0;
    BYTE tempVals[4];
    
    for(i = 0; i < size; i++)
    {

/*
        if(sensorDataBufferTail >= SENSOR_DATA_BUFFER_SIZE)
        {
            sensorDataBufferTail = 0;
        }
*/
        if(sensorDataBufferTail >= SENSOR_DATA_BUFFER_SIZE)
        {
            bufferFullFlag = 1;
            break;
        }
        else
        {
            sensorDataBuffer[sensorDataBufferTail++] = *data++;       
        }
    }  
    if(bufferFullFlag)
    {
        // Package the ASIP/bulk data transfer here...
        buildAsipTransmitMessage(0xFF, 0xFF, SENSOR_DATA_BUFFER_SIZE, sensorDataBuffer); // TODO: Fix class and ID
        // send bulk transfer headers to wifi buff...
        sendWifiCommand(15); // "\x1BY"
        // send CID...
        tempVals[0] = '0' + getAsipServerCID(); // Get ASCII representation of the CID
        addBytesToWifiTxBuffer(tempVals, 1);
        sendWifiCommand(16); //"address:port:\0"
        // send length...66 + 7
        tempVals[0] = '0';
        tempVals[1] = '0';
        tempVals[2] = '7';
        tempVals[3] = '3';
        addBytesToWifiTxBuffer(tempVals, 4);
        // send ASIP data piecewise to wifi buff...
        transmitAsipMessage();

        sensorDataBufferTail = 0;
    }  
}

void addBytesToWifiTxBuffer(BYTE * data, BYTE size)
{
    BYTE i;
    //BYTE emptyFlag = 0;
/*    
    _CLI();
    
    if(wifiTxBufferHead == wifiTxBufferTail)
    {
        emptyFlag = 1;
    }
    _SEI();
*/    
    for(i = 0; i < size; i++)
    {
        wifiTxBuffer[wifiTxBufferTail++] = *data++;
    }
    
    //if(emptyFlag)
    if(!spiTxSequenceInProgress)
    {
        // Kick off Tx interrupts here...
        forceStartSPITxSequence();
    }
}

// Kicks off an interrupt-driven sequence of SPI transmissions
void forceStartSPITxSequence(void)
{
    // If seq not already in progress
    if(!spiTxSequenceInProgress)
    {
        if(IS_WIFI_WAKEUP_HIGH)
        {
            if((wifiTxBufferTail != wifiTxBufferHead) && (!flowControlXoffFlag))
            {
                // Tx buffer not empty
                SPDR = wifiTxBuffer[wifiTxBufferHead++]; // write next buffer byte
                spiTxSequenceInProgress = 1;
            }
            else
            {
                // Tx idle character
                SPDR = 0xF5;
                spiTxSequenceInProgress = 1;
            }
        }
        else if(flowControlXoffFlag)
        {
            // Do not send buffered Tx data, send Tx idle character instead
            SPDR = 0xF5;
            spiTxSequenceInProgress = 1;
        }      
    }
}

BYTE parseIncomingGainspanData(BYTE nextChar, char * expectedPtr, BYTE expectedType)
{
    char * result;
    
    // Add new byte to buffer
    spiRxBuffer[spiRxBufferTail++] = nextChar;
    if(spiRxBufferTail >= SPI_RX_BUFFER_SIZE)
    {
        spiRxBufferTail = 0;
    }
    // Scan for expected substring
    result = strstr((char *)spiRxBuffer, expectedPtr);

    if(result != NULL)
    {
        spiRxBufferTail = 0;
        spiRxBufferHead = 0;

        // Set Rx OK flag here...
        // CID = *(result + 8)
        // Clear 'expecting config data' flag
        // Clear the buffer?
        if(expectedType == 1)
        {
            setAsipCID(*(result + 8));
        }
        else if(expectedType == 2)
        {
            setAsipServerCID(*(result + 8));
        }
        // Clear the buffer
        memset(spiRxBuffer, 0, SPI_RX_BUFFER_SIZE);

        return 1;
    }
    return 0;
}

BYTE getSpiResponseReceivedFlag(void)
{
    if(spiResponseReceivedFlag)
    {
        spiResponseReceivedFlag = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}