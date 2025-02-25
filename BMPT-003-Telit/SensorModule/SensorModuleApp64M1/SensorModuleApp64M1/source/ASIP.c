#include <stdio.h>
#include <stdlib.h>
#include <avr/eeprom.h>
#include "NValloc.h"
#include "ASIP.h"
#include "UART.h"
#include "WiFiModule.h"
#include "SensorModule.h"
#include "CAN.h"
#include "ADC.h"
#include "TaskList.h"

#define MAX_ASIP_PAYLOAD_SIZE   256
#define FW_UPDATE_DATA_BUFF_SIZE       256
#define FLETCHER_ZERO           0x00FF

//const char * commandSuccessfulPtr = "OK\r\0";
//const char * connectResponsePtr = "CONNECT \0";

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
    ASIP_CLASS_FIELD_CAL = 0x08,
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

static BYTE responseType = 0;

static const BYTE ASIP_SYNC_CHAR_1 = 0xC2;
static const BYTE ASIP_SYNC_CHAR_2 = 0x53;

static BYTE fwUpdateDataBuff[FW_UPDATE_DATA_BUFF_SIZE];

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

static BYTE deviceUpdateMsgReceivedFlag = 0;

static BYTE deviceInfoForwardingBuff[80];   // Buffer for assembling the full dev info payload from a wired sensor
static BYTE deviceInfoIndex = 0;            // Keeps track of how many bytes of a dev info message have been rx'd from a wired sensor

static DWORD receivedAddress = 0;
static DWORD nextAddress = 0;

static BYTE runDeviceInfoSeq = 0;

static BYTE calStatusDataAckReceived = 0;

//Function forward declarations
static void addByteToChecksum(BYTE data);
static void resetChecksum(void);

static void transmitAddByteToChecksum(BYTE data);
static void resetTransmitChecksums(void);

static void fillFwDataBuffer(void);

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
    BYTE reqFlags;

    reqFlags = getCANDataRequestFlags();
    
    if(ASIPMessageReady)
    {
        switch(Msg_Class)
        {
            case ASIP_CLASS_DEVICE:     
            if(Msg_ID == 0x00)
                {
                    if(!getRunDeviceInfoSeq())  // Ignore if a previous request is still being processed
                    {
                        disableSensorDataTx();
                        setCANDataRequestFlags(reqFlags | SYSTEM_DEV_INFO_REQ_BITMASK);
                        resetConnectedSensors();
						sendPingAllSensors();   // Request all connected sensors to reply with serial number
                    }
                }
                else if(Msg_ID == 0x30) // Update complete / jump to app command
                {
                    sendCANFwUpdateComplete();
                    sendCANSensorDataEnableMsg();
                    sendFwUpdateCompleteAckMessage();
                }
                else if(Msg_ID == 0x31) // Firmware write command with address
                {
                    setCANDataRequestFlags(reqFlags & ~(START_FW_UPDATE_RESP_BITMASK)); // Clear the start FW Update flag
                    
                    // Make sure the address is correct
                    receivedAddress = (DWORD)Msg_Payload[0] | ((DWORD)Msg_Payload[1] << 8) | ((DWORD)Msg_Payload[2] << 16) | ((DWORD)Msg_Payload[3] << 24);
                    
                    if(receivedAddress != nextAddress)
                    {
                        sendFwUpdatePacketNackMessage();
                    }
                    else
                    {
                        fillFwDataBuffer();
                        reqFlags = getCANDataRequestFlags();
                        setCANDataRequestFlags(reqFlags | FW_UPDATE_DATA_RX_BITMASK);
                        nextAddress += 250;
                    }
                }
                else if(Msg_ID == 0x32) // Start FW Update w/ Serial Number 
                {
                    sendCANSensorDataDisableMsg(); // Disable sensor CAN traffic until the update is done
                    
                    receivedAddress = 0;
                    nextAddress = 0;

                    // Check if Serial Num  matches this module...payload is 10-byte serial num string
                    // If not, re-Tx the command via CAN
                    if(compareSerialNums(&Msg_Payload[4], (getSerialNumPtr() + 4))) // Serial Num matches this module
                    {
                        _CLI();
                        // Set eeprom flag here...
                        // jump to bootloader...
						eeprom_busy_wait();
						while(eeprom_read_byte((BYTE *)EE_FW_UPDATE_PENDING_LOC) != 0x01)
						{
							eeprom_busy_wait();
							eeprom_write_byte((BYTE *)EE_FW_UPDATE_PENDING_LOC, 0x01);
							eeprom_busy_wait();
						}
                        jumpToBootloader();
                    }
                    else
                    {
                        // Re-Tx via CAN
                        sendCANStartFwUpdateMsg(&Msg_Payload[4]);
                    }                    
                }
                                                  
                break;

            case ASIP_CLASS_ACK:
                if(Msg_ID == 0x00) // Ack Ack
                {
                    if((Msg_Payload[0] == ASIP_CLASS_FIELD_CAL) && (Msg_Payload[1] == 0x0B))
                    {
                        calStatusDataAckReceived = 1;
                    }
                }
                break;
                   
            case ASIP_CLASS_FIELD_CAL:
                if(Msg_ID == 0x0A) // Field Cal command
                {
                    // send cal CAN command to all sensors on the bus,
                    //  then start a self-cal
                    // Save the connected sensor count, then wait for 
                    //    that many cal status responses
                    if(!getRunCalibrationFlag())
                    {
                        disableSensorDataTx();
                        clearCalStatusResponseCount();
                        sendStartCalAckMessage();
                        setCANDataRequestFlags(reqFlags | FIELD_CAL_REQ_BITMASK);
                        sendStartCalCommand(); 
                        startSensorCalibration();    
                    }
                }
                break;

			case ASIP_CLASS_FLOW:
				if(Msg_ID == 0x03) // Blockage Info
				{
                    if((Msg_Length % 5) == 0)
                    {
                        fillBlockageDataBuffer(&Msg_Payload[0], Msg_Length);
                    }

				}				
				break;
				
            default:
                break;

        }
        ASIPMessageReady = 0;
    }
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

/*********************************************************************
 Transmits the ASIP message previously built with buildAsipTransmitMessage
 *********************************************************************/
void transmitAsipMessage(void)
{
	sendUdpPacket(&outgoingMsg);
}

// Adds the assembled ASIP message to the transmit queue 
//  without any UDP header or prefix bytes
void transmitRawAsipMessage(void)
{
    BYTE cmdBuffer[5];

	cmdBuffer[0] = 0xC2;
	cmdBuffer[1] = 0x53;
	cmdBuffer[2] = outgoingMsg.MsgClass;
	cmdBuffer[3] = outgoingMsg.MsgId;
	cmdBuffer[4] = outgoingMsg.PayloadLength;
	
	addBytesToWifiTxBuffer(cmdBuffer, 5);
	// Add in payload
	addBytesToWifiTxBuffer(outgoingMsg.PayloadPtr, outgoingMsg.PayloadLength);
	// Add in checksum bytes
	cmdBuffer[0] = outgoingMsg.Checksum1;
	cmdBuffer[1] = outgoingMsg.Checksum2;
	addBytesToWifiTxBuffer(cmdBuffer, 2);    
}


void transmitTcpAsipMessage(void)
{
    sendTcpPacket(&outgoingMsg);
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


#define MAXIMUM_FLOW_RATE_MESSAGES 24
static BYTE scbFlowRateTxQueue[MAXIMUM_FLOW_RATE_MESSAGES*6];
static BYTE scFlowRateMessageCount = 0;

/*********************************************************************
 ASIP equivalent to the sendCANSensorDataMsg function. This does not
 transmit immediately, but adds the levels to the flow rate message queue
 *********************************************************************/
void sendASIPSensorDataMsg(BYTE * levels)
{
	BYTE * pMessage;
	_CLI();
	BYTE * serialNumPtr = getSerialNumPtr() + 4;
	
	if(scFlowRateMessageCount < MAXIMUM_FLOW_RATE_MESSAGES)
	{
		// Write pointer
		pMessage = scbFlowRateTxQueue + (scFlowRateMessageCount * 6);
		
		pMessage[0] = *(serialNumPtr); // Serial Num 
		pMessage[1] = *(serialNumPtr + 1); // Serial Num
		pMessage[2] = *(serialNumPtr + 2); // Serial Num 
		pMessage[3] = *(serialNumPtr + 3); // Serial Num 
		pMessage[4] = ((levels[0] << 4) | (levels[1] & 0x0F)); // Rows 1 & 2
		pMessage[5] = ((levels[2] << 4) | (levels[3] & 0x0F)); // Rows 3 & 4
		
		scFlowRateMessageCount++;
	}
	_SEI();
}


/********************************************************************
 Adds a CAN flow rate message to the flow rate transmit queue.
 ********************************************************************/
void addFlowRateMessage(BYTE * message)
{
	BYTE i;
	BYTE * pMessage;
	if(scFlowRateMessageCount < MAXIMUM_FLOW_RATE_MESSAGES)
	{
		pMessage = scbFlowRateTxQueue + (scFlowRateMessageCount * 6);

		for(i = 0; i < 6; i++)
		{
			pMessage[i] = message[i];	
		}

		scFlowRateMessageCount++;
	}	
}


/********************************************************************
 Takes the flow rate transmit queue and sends it out as an ASIP message.
 ********************************************************************/
void sendFlowRateMessages(void)
{
	if(scFlowRateMessageCount > 0)
	{
		buildAsipTransmitMessage(0x0A, 0x01, scFlowRateMessageCount * 6, scbFlowRateTxQueue);
		transmitAsipMessage();
		scFlowRateMessageCount = 0;
	}
}

void sendFwUpdatePacketAckMessage(void)
{
    BYTE payload[] = {0x00, 0x31};
    
    outgoingMsg.MsgClass = 0x01;
    outgoingMsg.MsgId = 0x00;
    outgoingMsg.PayloadLength = 2;
    outgoingMsg.PayloadPtr = payload;
    outgoingMsg.Checksum1 = 0x4A;
    outgoingMsg.Checksum2 = 0x83;

	transmitAsipMessage();    
}


void sendStartFwUpdateAckMessage(void)
{
    BYTE mode = 3;

    outgoingMsg.MsgClass = 0x00;
    outgoingMsg.MsgId = 0x05;
    outgoingMsg.PayloadLength = 1;
    outgoingMsg.PayloadPtr = &mode;
    outgoingMsg.Checksum1 = 0x1F;
    outgoingMsg.Checksum2 = 0x45;

    transmitAsipMessage();
}

void sendStartCalAckMessage(void)
{
    BYTE payload[] = {ASIP_CLASS_FIELD_CAL, 0x0A};

    outgoingMsg.MsgClass = 0x01;
    outgoingMsg.MsgId = 0x00;
    outgoingMsg.PayloadLength = 2;
    outgoingMsg.PayloadPtr = payload;
    outgoingMsg.Checksum1 = 0x2B;
    outgoingMsg.Checksum2 = 0x6C;

    transmitAsipMessage();
}

void sendFwUpdatePacketNackMessage(void)
{
    BYTE payload[] = {0x00, 0x31, 0x00};

    outgoingMsg.MsgClass = 0x01;
    outgoingMsg.MsgId = 0x01;
    outgoingMsg.PayloadLength = 3;
    outgoingMsg.PayloadPtr = payload;
    outgoingMsg.Checksum1 = 0x4C;
    outgoingMsg.Checksum2 = 0xD6;

    transmitAsipMessage();
}

void sendFwUpdateCompleteAckMessage(void)
{
    BYTE payload[] = {0x00, 0x30};

    outgoingMsg.MsgClass = 0x01;
    outgoingMsg.MsgId = 0x00;
    outgoingMsg.PayloadLength = 2;
    outgoingMsg.PayloadPtr = payload;
    outgoingMsg.Checksum1 = 0x49;
    outgoingMsg.Checksum2 = 0x82;

    transmitAsipMessage();
}

void sendBITMessage(WORD id, BYTE severity)
{
    BYTE payload[5];

    payload[0] = 1;
    payload[1] = (BYTE)(id & 0xFF);
    payload[2] = (BYTE)(id >> 8);
    payload[3] = severity;
    payload[4] = 0; // 0 => fail
    
    buildAsipTransmitMessage(0x03, 0x09, 5, &payload[0]);
    transmitAsipMessage();
}

void sendDeviceInfo(void)
{
    BYTE i;
    BYTE * tempPtr;

    tempPtr = getSerialNumPtr();

    for(i = 0; i < 74; i++)
    {
        deviceInfoForwardingBuff[i] = 0;
    }

    // Build payload
    for(i = 0; i < 10; i++)
    {
        deviceInfoForwardingBuff[i] = *tempPtr++;
    }

    tempPtr = (BYTE *)getFwVersionPtr();

    for(i = 10; i < 26; i++)
    {
        deviceInfoForwardingBuff[i] = *tempPtr++;
    }

    readFwPartNum(&deviceInfoForwardingBuff[26]);

    readHwVersion(&deviceInfoForwardingBuff[42]);
    readHwPartNum(&deviceInfoForwardingBuff[58]);
    
	transmitDeviceInfoASIP();

}

static void fillFwDataBuffer(void)
{
    BYTE i;

    for(i = 0; i < 250; i++)
    {
        // Copy the FW image data, but ignore the 4 byte address
        fwUpdateDataBuff[i] = Msg_Payload[i + 4];
    }
}

BYTE * getFwDataPtr(void)
{
    return &fwUpdateDataBuff[0];
}


// Collects data packets from a wired sensor and assembles 
//  them into a device info response message
void storeDeviceInfoData(BYTE * data)
{
    BYTE i;

    for(i = 0; i < 8; i++)
    {
        deviceInfoForwardingBuff[deviceInfoIndex] = *(data++);
        deviceInfoIndex++;
    }

    if(deviceInfoIndex >= 74)
    {
        deviceInfoIndex = 0;
        i = getCANDataRequestFlags();
        setCANDataRequestFlags(i | DEV_INFO_READY_BITMASK);
        
    }
}

BYTE getRunDeviceInfoSeq(void)
{
    return runDeviceInfoSeq;
}

void setRunDeviceInfoSeq(void)
{
    runDeviceInfoSeq = 1;
}

void clearRunDeviceInfoSeq(void)
{
    runDeviceInfoSeq = 0;
}

BYTE getCalStatusAckReceivedFlag(void)
{
    return calStatusDataAckReceived;
}

void clearCalStatusAckReceivedFlag(void)
{
    calStatusDataAckReceived = 0;
}

void transmitDeviceInfoASIP(void)
{
    BYTE sensorCount = 0;
	
	buildAsipTransmitMessage(0x00, 0x01, 74, deviceInfoForwardingBuff);
	transmitAsipMessage();
	sensorCount = getConnectedSensorCount() + 1; // Get wired sensor count and add one for the wireless
	buildAsipTransmitMessage(0x0A, 0x02, 1, &sensorCount);
	transmitAsipMessage();
}

// Configures the calibration results ASIP message and copies 
//  the data into the WiFi transmit buffer.
void transmitCalStatusInfoASIP(BYTE sensorCount, BYTE* dataBuffer)
{
    BYTE totalLength;
    
    totalLength = (sensorCount * 7) + 1;
    *dataBuffer = sensorCount; // Set payload field of sensor count
    buildAsipTransmitMessage(ASIP_CLASS_FIELD_CAL, 0x0B, totalLength, dataBuffer);
    transmitAsipMessage();
}

// Resets the count of received Device Info data
//  in the event of an abort/timeout situation
void resetDeviceInfoBuffer(void)
{
	BYTE flags = 0;
	
	deviceInfoIndex = 0;
    flags = getCANDataRequestFlags();
	
	if(!(flags & SYSTEM_DEV_INFO_REQ_BITMASK))
	{
		// Only do this if the wireless sensor's device info has been sent
		setCANDataRequestFlags(flags | DEV_INFO_READY_BITMASK);	
	}	
}




