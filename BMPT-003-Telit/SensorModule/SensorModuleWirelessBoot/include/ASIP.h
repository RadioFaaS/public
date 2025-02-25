#ifndef ASIP_H
#define ASIP_H

#include "Types.h"

typedef struct
{
    BYTE MsgClass;
    BYTE MsgId;
    BYTE PayloadLength;
    BYTE *PayloadPtr;
    BYTE Checksum1;
    BYTE Checksum2;
    
} AsipMsgStruct;


//void processAsipMessage(unsigned char* data, int size);
void processValidAsipMessages(void);
void resetAsipStateMachine(void);
void stepAsipStateMachine(BYTE newVal);

void buildAsipTransmitMessage(BYTE msgClass, BYTE msgId, BYTE payloadLength, BYTE *payloadData);
void serializeAsipMsgStruct(AsipMsgStruct * message, BYTE *outputBuffer);

BYTE getAckReceivedFlag(void);
void clearAckReceivedFlag(void);
BYTE getNackReceivedFlag(void);
void clearNackReceivedFlag(void);

BYTE getDeviceUpdateMsgReceivedFlag(void);
void clearDeviceUpdateMsgReceivedFlag(void);

//void transmitAsipMessage(BYTE msgClass, BYTE msgId, BYTE payloadLength, BYTE *payloadData);
void transmitAsipMessage(void);
void transmitTcpAsipMessage(void);

BYTE getResponseType(void);

void stepResponseMonitorStateMachine(BYTE newByte, BYTE expectedResponseType);

BYTE * getAsipPayloadPtr(void);
BYTE getAsipPayloadSize(void);

DWORD getReceivedChecksum(void);

#endif // ASIP_H
