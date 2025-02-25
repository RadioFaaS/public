#include <stdio.h>
#include <stdlib.h>
#include <avr/eeprom.h>
#include "NValloc.h"
#include "ASIP.h"
#include "SPI.h"
#include "WiFiModule.h"
#include "SensorModule.h"
#include "CAN.h"

#define MAX_ASIP_PAYLOAD_SIZE   64//256 TODO: Determine this size (for incoming ASIP only)
#define FLETCHER_ZERO           0x00FF


enum
{
    ASIP_SYNC1 = 0,
    ASIP_SYNC2,
    ASIP_CLASS,
    ASIP_ID,
    ASIP_LENGTH,
    ASIP_PAYLOAD,
    ASIP_CS1,
    ASIP_CS2
};

// ASIP Class enumerations 
enum
{
    ASIP_CLASS_DEVICE = 0x00,
    ASIP_CLASS_ACK,
    ASIP_CLASS_BIT = 0x03,
    ASIP_CLASS_FACTORY_CAL = 0x07,
    ASIP_CLASS_GENERIC = 0xFF,
};

static BYTE AsipSMState = ASIP_SYNC1;
static BYTE Msg_Class;
static BYTE Msg_ID;
static BYTE Msg_Length;
static BYTE Msg_Payload[MAX_ASIP_PAYLOAD_SIZE];
static BYTE Msg_CS1;
static BYTE Msg_CS2;

static const BYTE ASIP_SYNC_CHAR_1 = 0xC2;
static const BYTE ASIP_SYNC_CHAR_2 = 0x53;

static BYTE PayloadIndex = 0;
static BYTE PayloadCount = 0;

static BYTE ASIPMessageReady = 0;

static AsipMsgStruct outgoingMsg; 

// Checksums for validating incoming ASIP messages
static WORD runningCS1 = 0;
static WORD runningCS2 = 0;
static BYTE CSByteCount = 0;

// Checksums for generating outgoing ASIP messages
static WORD transmitRunningChecksum1 = 0;
static WORD transmitRunningChecksum2 = 0;
static BYTE transmitFinalChecksum1 = 0;
static BYTE transmitFinalChecksum2 = 0;

//static BYTE ackReceivedFlag = 0;
//static BYTE nackReceivedFlag = 0;

static BYTE deviceUpdateMsgReceivedFlag = 0;
//static BYTE deviceConfigMsgReceivedFlag = 0; // Indicates at least one config ASIP msg has been received


//Function forward declarations
static void addByteToChecksum(BYTE data);
static void resetChecksum(void);

static void transmitAddByteToChecksum(BYTE data);
static void resetTransmitChecksums(void);

/*********************************************************************
 This function runs the ASIP parser state machine, and processes each
 incoming byte per the ASIP message protocol. The 'ASIPMessageReady'
 flag is set to 1 when a complete valid message has been received,
 and is 0 otherwise.
 *********************************************************************/
void stepAsipStateMachine(BYTE newVal)
{
    switch(AsipSMState)
    {
        case ASIP_SYNC1:
            if(newVal == ASIP_SYNC_CHAR_1)
            {
                addByteToChecksum(ASIP_SYNC_CHAR_1);
                AsipSMState = ASIP_SYNC2;
            }
            break;

        case ASIP_SYNC2:
            if(newVal == ASIP_SYNC_CHAR_2)
            {
                addByteToChecksum(ASIP_SYNC_CHAR_2);
                AsipSMState = ASIP_CLASS;
            }
            else if(newVal == ASIP_SYNC_CHAR_1) // Could be start of new message, stay in this state
            {
                resetChecksum();
                addByteToChecksum(ASIP_SYNC_CHAR_1);
                AsipSMState = ASIP_SYNC2;                 
            }
            else
            {
                resetChecksum();
                AsipSMState = ASIP_SYNC1;                 
            }
            break;

        case ASIP_CLASS:
            Msg_Class = newVal;
            addByteToChecksum(Msg_Class);
            AsipSMState = ASIP_ID;
            break;

        case ASIP_ID:
            Msg_ID = newVal;
            addByteToChecksum(Msg_ID);
            AsipSMState = ASIP_LENGTH;
            break;

        case ASIP_LENGTH:
            Msg_Length = newVal;

            if(Msg_Length == 0)
            {
                // No payload so this is final pre-checksum byte
                AsipSMState = ASIP_CS1;
                addByteToChecksum(Msg_Length);
            }
            else
            {
                AsipSMState = ASIP_PAYLOAD;
                addByteToChecksum(Msg_Length);
            }
            break;

        case ASIP_PAYLOAD:
            // Keep track of number of iterations in this state
            if(++PayloadCount == Msg_Length)
            {
                // Last byte of payload...
                Msg_Payload[PayloadIndex++] = newVal;
                addByteToChecksum(newVal);
                PayloadCount = 0;
                PayloadIndex = 0;
                AsipSMState = ASIP_CS1;
            }
            else
            {
                Msg_Payload[PayloadIndex++] = newVal;
                addByteToChecksum(newVal);
            }
            break;

        case ASIP_CS1:
            // Check for valid checksums here
            if(Msg_CS1 == newVal)
            {
                AsipSMState = ASIP_CS2;
            }
            else
            {
                // Checksum fails...
                
                resetChecksum();
                AsipSMState = ASIP_SYNC1;               
            }
            break;

        case ASIP_CS2:
            if(Msg_CS2 == newVal)
            {
                ASIPMessageReady = 1;
            }
           
            resetChecksum();
            AsipSMState = ASIP_SYNC1;
            break;

        default:
            break;
    }

}


/*********************************************************************
 This function processes buffered ASIP data. If a valid ASIP
 message is found, an appropriate action is triggered based on the
 message class and ID.
*********************************************************************/
//void processAsipMessage(unsigned char* data, int size)
void processValidAsipMessages(void)
{
    //BYTE i;  
    
    // Process the incoming data byte by byte
    //for(i = 0; i < size; i++)
    //{
        //stepAsipStateMachine(data[i]);
    BYTE tempVal = 0;
    
    if(ASIPMessageReady)
    {
        switch(Msg_Class)
        {
            case ASIP_CLASS_DEVICE:     
                if(Msg_ID == 0x2F) // FW update command
                {
                    // TODO:
                    // Check if Serial Num  matches this module...
                    // If not, re-Tx the command via CAN
                    if(compareSerialNums(&Msg_Payload[0], (getSerialNumPtr() + 4))) // Serial Num matches this module
                    {
                        _CLI();
                        // TODO: set eeprom flag here...
                        // jump to bootloader...
                        eeprom_busy_wait();
                        eeprom_write_byte((BYTE *)EE_FW_UPDATE_PENDING_LOC, 0x01);
                        jumpToBootloader();
                    }
                    else
                    {
                        // Re-Tx via CAN
                        sendCANStartFwUpdateMsg(&Msg_Payload[0]);
                    }
                }
                                                  
                break;

            case ASIP_CLASS_GENERIC:
                if(Msg_ID == 0x00)
                {
                    // Active blockage
                    if(compareSerialNums(&Msg_Payload[0], (getSerialNumPtr() + 4))) // Serial Num matches this module
                    {
                            // TODO: Store blockage info here...Make this a common function with CAN parser??                           
                            storeActiveBlockageData(Msg_Payload[4]);                                             
                    }
                    else
                    {
                        // Re-Tx via CAN
                        sendCANActiveBlockageMsg(&Msg_Payload[0], Msg_Payload[4]);
                    }
                }
                else if(Msg_ID == 0x01)
                {
                    // Version request (broadcast)
                    // TODO: Set version request flag here...
                    sendCANRequestVersionsMsg();
                }
                break;

            case ASIP_CLASS_ACK: // This class by definition handles both Ack and Nack messages
                if(Msg_ID == 0x00)
                {
                    // Ack
                    //ackReceivedFlag = 1;
                }
                else if(Msg_ID == 0x01)
                {
                    // Nack
                    //nackReceivedFlag = 1;
                }
                else if(Msg_ID == 0xFF)
                {
                    // Unknown message
                    //nackReceivedFlag = 1;                     
                }
                                         
                break;
            case ASIP_CLASS_BIT:                    
                break;
                    
            case ASIP_CLASS_FACTORY_CAL:
                /*if(Msg_ID == 0x00)
                {
                    startSensorCalibration();
                    // send sensor cal CAN command...
                }*/
                break;

            default:
                break;

        }
        ASIPMessageReady = 0;
    }
    //}
}

/********************************************************************
 Calculates a running checksum byte by byte. Only for use with
 the incoming ASIP state machine parser.
 ********************************************************************/
static void addByteToChecksum(BYTE data)
{
    runningCS1 = (runningCS1 + data);
    if(runningCS1 > FLETCHER_ZERO)
    {
        runningCS1 -= FLETCHER_ZERO;
    }
    runningCS2 = (runningCS2 + runningCS1);
    if(runningCS2 > FLETCHER_ZERO)
    {
        runningCS2 -= FLETCHER_ZERO;
    }
    
    Msg_CS1 = (BYTE)runningCS1;
    Msg_CS2 = (BYTE)runningCS2;

}


/*********************************************************************
 This function performs a simple reset of the running checksum values
 for a given byte stream. Should be called when a valid message is 
 found or whenever the parser state machine requires a reset.
 *********************************************************************/
static void resetChecksum(void)
{
    runningCS1 = FLETCHER_ZERO;
    runningCS2 = FLETCHER_ZERO;
    Msg_CS1 = FLETCHER_ZERO;
    Msg_CS2 = FLETCHER_ZERO;
    CSByteCount = 0;
}

/********************************************************************
 Resets the ASIP state machine.
 ********************************************************************/
void resetAsipStateMachine(void)
{
    resetChecksum();
#ifdef TEST_MODE
    
    totalAsipCompleteMsgCount = 0;
    totalAsipDeviceInfoCount = 0;
    totalAsipBITCount = 0;
    totalAsipAckCount = 0;
    failedAsipChecksumCount = 0;
    totalAsipFailedMsgCount = 0;
    
#endif
    AsipSMState = ASIP_SYNC1;
    ASIPMessageReady = 0;
    
}


/********************************************************************
 Allocates and assembles an ASIP message struct from the passed 
 in parameters. 
 ********************************************************************/
void buildAsipTransmitMessage(BYTE msgClass, BYTE msgId, BYTE payloadLength, BYTE *payloadData)
{
    BYTE i;
    
    // Populate the struct
    outgoingMsg.MsgClass = msgClass;
    outgoingMsg.MsgId = msgId;
    outgoingMsg.PayloadLength = payloadLength;
    outgoingMsg.PayloadPtr = payloadData;
    
    // Now calculate the checksums for this message
    resetTransmitChecksums();
    
    transmitAddByteToChecksum(ASIP_SYNC_CHAR_1);
    transmitAddByteToChecksum(ASIP_SYNC_CHAR_2);
    transmitAddByteToChecksum(msgClass);
    transmitAddByteToChecksum(msgId);
    transmitAddByteToChecksum(payloadLength);
    
    for(i = 0; i < payloadLength; i++)
    {
        transmitAddByteToChecksum(payloadData[i]);
    }
    
    outgoingMsg.Checksum1 = transmitFinalChecksum1;
    outgoingMsg.Checksum2 = transmitFinalChecksum2;
    
    return;
}


/********************************************************************
 Accepts an ASIP message struct and serializes the data into a
 BYTE array for easy transmission.
 ********************************************************************/
void serializeAsipMsgStruct(AsipMsgStruct * message, BYTE *outputBuffer)
{
    BYTE i;
    //BYTE *outputBuffer = (BYTE *)malloc(message->PayloadLength + 7);
    
    outputBuffer[0] = ASIP_SYNC_CHAR_1;
    outputBuffer[1] = ASIP_SYNC_CHAR_2;
    outputBuffer[2] = message->MsgClass;
    outputBuffer[3] = message->MsgId;
    outputBuffer[4] = message->PayloadLength;
    
    for(i = 0; i < message->PayloadLength; i++)
    {
        outputBuffer[5 + i] = *(message->PayloadPtr + i);
    }
    
    outputBuffer[5 + message->PayloadLength] = message->Checksum1;
    outputBuffer[6 + message->PayloadLength] = message->Checksum2;
    
    //return outputBuffer;
}

/********************************************************************
 Calculates a running checksum byte by byte. Only for use in 
 generating an outgoing ASIP message.
 ********************************************************************/
static void transmitAddByteToChecksum(BYTE data)
{
    transmitRunningChecksum1 = (transmitRunningChecksum1 + data);
    if(transmitRunningChecksum1 > FLETCHER_ZERO)
    {
        transmitRunningChecksum1 -= FLETCHER_ZERO;
    }
    transmitRunningChecksum2 = (transmitRunningChecksum2 + transmitRunningChecksum1);
    if(transmitRunningChecksum2 > FLETCHER_ZERO)
    {
        transmitRunningChecksum2 -= FLETCHER_ZERO;
    }
    
    transmitFinalChecksum1 = (BYTE)transmitRunningChecksum1;
    transmitFinalChecksum2 = (BYTE)transmitRunningChecksum2;
    
}

/********************************************************************
 Resets the static checksums used with outgoing ASIP messages.
 ********************************************************************/
static void resetTransmitChecksums(void)
{
    transmitRunningChecksum1 = FLETCHER_ZERO;
    transmitRunningChecksum2 = FLETCHER_ZERO;
    transmitFinalChecksum1 = FLETCHER_ZERO;
    transmitFinalChecksum2 = FLETCHER_ZERO;
}


/********************************************************************
 Returns the flag indicating whether an ASIP Fw Update message
 has been received.
 ********************************************************************/
BYTE getDeviceUpdateMsgReceivedFlag(void)
{
    return deviceUpdateMsgReceivedFlag;
}

/********************************************************************
 Clears the 'deviceUpdateMsgReceivedFlag'.
 ********************************************************************/
void clearDeviceUpdateMsgReceivedFlag(void)
{
    deviceUpdateMsgReceivedFlag = 0;
}


/*********************************************************************
 Builds and transmits an ASIP message.
 *********************************************************************/
//void transmitAsipMessage(BYTE msgClass, BYTE msgId, BYTE payloadLength, BYTE *payloadData)
void transmitAsipMessage(void)
{
/*
    BYTE serializedMsgData[payloadLength + 7];  // Total packet size is always 7 wrapper bytes plus the payload
    
    buildAsipTransmitMessage(msgClass, msgId, payloadLength, payloadData);
    serializeAsipMsgStruct(&outgoingMsg, serializedMsgData);
   
    ////transmitUdpPacketToDevice(serializedMsgData, (7 + payloadLength));
    addBytesToWifiTxBuffer(serializedMsgData, (7 + payloadLength));
*/
    BYTE tempData;
    
    tempData = outgoingMsg.MsgClass;
    addBytesToWifiTxBuffer(&tempData, 1);
    tempData = outgoingMsg.MsgId;
    addBytesToWifiTxBuffer(&tempData, 1);
    tempData = outgoingMsg.PayloadLength;
    addBytesToWifiTxBuffer(&tempData, 1);
    addBytesToWifiTxBuffer(outgoingMsg.PayloadPtr, outgoingMsg.PayloadLength);
    tempData = outgoingMsg.Checksum1;
    addBytesToWifiTxBuffer(&tempData, 1);
    tempData = outgoingMsg.Checksum2;
    addBytesToWifiTxBuffer(&tempData, 1);
       
}

void buildAsipDeviceInfoMessage(void)
{
        BYTE i;
        BYTE * tempPtr;
        BYTE deviceInfoMsg[74];
        BYTE tempVals[4];

        tempPtr = getSerialNumPtr();

        // Build payload
        for(i = 0; i < 10; i++)
        {
            deviceInfoMsg[i] = *tempPtr++;
        }

        tempPtr = (BYTE *)getFwVersionPtr();

        for(i = 10; i < 26; i++)
        {
            deviceInfoMsg[i] = *tempPtr++;
        }

        readHwVersion(&deviceInfoMsg[42]);
        readHwPartNum(&deviceInfoMsg[58]);
        
        // Package the ASIP/bulk data transfer here...
        buildAsipTransmitMessage(0x00, 0x00, 74, deviceInfoMsg); // TODO: Fix class and ID
        // send bulk transfer headers to wifi buff...
        sendWifiCommand(15); // "\x1BY"
        // send CID...
        tempVals[0] = '0' + getAsipServerCID(); // Get ASCII representation of the CID
        addBytesToWifiTxBuffer(tempVals, 1);
        sendWifiCommand(16); //"address:port:\0"
        // send ASCII length...74 + 7
        tempVals[0] = '0';
        tempVals[1] = '0';
        tempVals[2] = '8';
        tempVals[3] = '1';
        addBytesToWifiTxBuffer(tempVals, 4);
        // send ASIP data piecewise to wifi buff...
        transmitAsipMessage();
}

