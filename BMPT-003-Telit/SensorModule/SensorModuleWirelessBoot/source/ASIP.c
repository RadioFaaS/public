#include <stdio.h>
#include <stdlib.h>
#include <avr/eeprom.h>
#include "NValloc.h"
#include "ASIP.h"
#include "UART.h"
#include "WiFiModule.h"
#include "SensorModuleWirelessBoot.h"

#define MAX_ASIP_PAYLOAD_SIZE   256
#define FLETCHER_ZERO           0x00FF

const char * commandSuccessfulPtr = "OK\r\0";
const char * connectResponsePtr = "CONNECT \0";

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

enum
{
    RESPONSE_IDLE = 0,
    RESPONSE_OK_1,
    RESPONSE_CONNECT,
    RESPONSE_GET_CID
};

// ASIP Class enumerations 
enum
{
    ASIP_CLASS_DEVICE = 0x00,
    ASIP_CLASS_ACK,
    ASIP_CLASS_BIT = 0x03,
    ASIP_CLASS_FACTORY_CAL = 0x07,
    ASIP_CLASS_FLOW = 0x0A,
    ASIP_CLASS_GENERIC = 0xFF,
};

static BYTE AsipSMState = ASIP_SYNC1;
static BYTE Msg_Class;
static BYTE Msg_ID;
static BYTE Msg_Length;
static BYTE Msg_Payload[MAX_ASIP_PAYLOAD_SIZE];
static BYTE Msg_CS1;
static BYTE Msg_CS2;

//static BYTE responseSMState = RESPONSE_IDLE;
static BYTE responseType = 0;

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

// Reprogramming variables
static DWORD scdwNextWriteAddress = 0;
static WORD scbBytesWrittenInPacket = 0;
static DWORD receivedChecksum = 0;

//Function forward declarations
static void addByteToChecksum(BYTE data);
static void resetChecksum(void);

static void transmitAddByteToChecksum(BYTE data);
static void resetTransmitChecksums(void);
static void sendStartFwUpdateAckMessage(void);
static void sendFwUpdatePacketAckMessage(void);
static void sendFwUpdatePacketNackMessage(void);
static void sendFwUpdateCompleteAck(void);
static void sendDeviceInfo(void);

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

BYTE getResponseType(void)
{
    BYTE retVal;

    retVal = responseType;
    responseType = 0;

    return retVal;
}

/*********************************************************************
 This function processes buffered ASIP data. If a valid ASIP
 message is found, an appropriate action is triggered based on the
 message class and ID.
*********************************************************************/
void processValidAsipMessages(void)
{
	BOOL clearMessage = TRUE;
    
    if(ASIPMessageReady)
    {
        switch(Msg_Class)
        {
            case ASIP_CLASS_DEVICE:     
                if(Msg_ID == 0x30) // FW Update complete
                {
                    sendFwUpdateCompleteAck();
                    setFwUpdateCompleteFlag();
                }
                else if(Msg_ID == 0x00) // Device Info request
                {
                     sendDeviceInfo();
                }               
                else if(Msg_ID == 0x31) // FW Update data
                {
					DWORD msgFWAddress;
					if(scbBytesWrittenInPacket == 0)
					{
						// Make sure the address is correct
						msgFWAddress = (DWORD)Msg_Payload[0] | ((DWORD)Msg_Payload[1] << 8) | ((DWORD)Msg_Payload[2] << 16) | ((DWORD)Msg_Payload[3] << 24);					
                        
                        if(msgFWAddress != scdwNextWriteAddress)
						{
							// Out of sequence. Send NACK
							//addBytesToWifiTxBuffer(&updatePacketNackMessage[0], 10);
                            sendFwUpdatePacketNackMessage();
							break;
						}
						else if(msgFWAddress != 0)
						{
							scdwNextWriteAddress += (Msg_Length - 4);
							scbBytesWrittenInPacket += 4;
						}
                        else // Address is 0, so this is the first packet...compensate for the extra header bytes
                        {
                            receivedChecksum = (DWORD)Msg_Payload[8] | ((DWORD)Msg_Payload[9] << 8) | ((DWORD)Msg_Payload[10] << 16) | ((DWORD)Msg_Payload[11] << 24);
                            scdwNextWriteAddress += (Msg_Length - 4);//(Msg_Length - 44); 
                            scbBytesWrittenInPacket += 44; // 4 address bytes + 40 header bytes
                        }
					}						

				    scbBytesWrittenInPacket += pushPageBuffer( &Msg_Payload[scbBytesWrittenInPacket], Msg_Length - scbBytesWrittenInPacket);      
                                  
					if(scbBytesWrittenInPacket < Msg_Length)
					{
						// Don't clear out the message yet
						clearMessage = FALSE;
						// Try adding in pieces of the receive buffer to the page write buffer

                        TOGGLE_ROW4_LED;
					}
					else
					{
						// All done, cleanup and ack
						scbBytesWrittenInPacket = 0;
						//addBytesToWifiTxBuffer(&updatePacketAckMessage[0], 9);
                        sendFwUpdatePacketAckMessage();
                        TOGGLE_LED;
					}												
                }
                else if(Msg_ID == 0x32) // Start FW Update w/ Serial Number
                {
                    // Check if Serial Num  matches this module...payload is 10-byte serial num string
                    // If not, re-Tx the command via CAN
                    if(compareSerialNums(&Msg_Payload[4], (getSerialNumPtr() + 4))) // Serial Num matches this module
                    {
                        sendReadyForFwUpdate();
                    }
                    else
                    {
						
                    }
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
		if(clearMessage)
		{
			ASIPMessageReady = 0;
		}        
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
    sendUdpPacket(&outgoingMsg);    
}


void transmitTcpAsipMessage(void)
{
    sendTcpPacket(&outgoingMsg);
};



static void sendStartFwUpdateAckMessage(void)
{
    BYTE payload[] = {0x00, 0x32};

    outgoingMsg.MsgClass = 0x01;
    outgoingMsg.MsgId = 0x00;
    outgoingMsg.PayloadLength = 2;
    outgoingMsg.PayloadPtr = &payload;
    outgoingMsg.Checksum1 = 0x4B;
    outgoingMsg.Checksum2 = 0x84;

    transmitAsipMessage();
    //transmitTcpAsipMessage();
}

static void sendFwUpdatePacketAckMessage(void)
{
    BYTE payload[] = {0x00, 0x31};

    outgoingMsg.MsgClass = 0x01;
    outgoingMsg.MsgId = 0x00;
    outgoingMsg.PayloadLength = 2;
    outgoingMsg.PayloadPtr = &payload;
    outgoingMsg.Checksum1 = 0x4A;
    outgoingMsg.Checksum2 = 0x83;

    transmitAsipMessage();
    //transmitTcpAsipMessage();
}


static void sendFwUpdatePacketNackMessage(void)
{
    BYTE payload[] = {0x00, 0x31, 0x00};

    outgoingMsg.MsgClass = 0x01;
    outgoingMsg.MsgId = 0x01;
    outgoingMsg.PayloadLength = 3;
    outgoingMsg.PayloadPtr = &payload;
    outgoingMsg.Checksum1 = 0x4C;
    outgoingMsg.Checksum2 = 0xD6;

    transmitAsipMessage();
    //transmitTcpAsipMessage();
}

static void sendFwUpdateCompleteAck(void)
{
    BYTE payload[] = {0x00, 0x30};

    outgoingMsg.MsgClass = 0x01;
    outgoingMsg.MsgId = 0x00;
    outgoingMsg.PayloadLength = 2;
    outgoingMsg.PayloadPtr = &payload;
    outgoingMsg.Checksum1 = 0x49;
    outgoingMsg.Checksum2 = 0x82;

    transmitAsipMessage();   
    //transmitTcpAsipMessage();
}

DWORD getReceivedChecksum(void)
{
    return receivedChecksum;
}

static void sendDeviceInfo(void)
{
    BYTE i;
    BYTE * tempPtr;
    BYTE deviceInfoMsg[74];
    //BYTE tempVals[4];

    tempPtr = (BYTE *)getSerialNumPtr();

    for(i = 0; i < 74; i++)
    {
        deviceInfoMsg[i] = 0;
    }

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

    readFwPartNum(&deviceInfoMsg[26]);

    readHwVersion(&deviceInfoMsg[42]);
    readHwPartNum(&deviceInfoMsg[58]);
    
    buildAsipTransmitMessage(0x00, 0x01, 74, deviceInfoMsg);
    transmitAsipMessage();
    //transmitTcpAsipMessage();
}

