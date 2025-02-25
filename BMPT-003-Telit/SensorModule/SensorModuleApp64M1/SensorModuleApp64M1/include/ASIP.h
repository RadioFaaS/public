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


void processValidAsipMessages(void);
void resetAsipStateMachine(void);
void stepAsipStateMachine(BYTE newVal);

void buildAsipTransmitMessage(BYTE msgClass, BYTE msgId, BYTE payloadLength, BYTE *payloadData);
void transmitAsipMessage(void);
void transmitTcpAsipMessage(void);

BYTE getAckReceivedFlag(void);
void clearAckReceivedFlag(void);
BYTE getNackReceivedFlag(void);
void clearNackReceivedFlag(void);

BYTE getDeviceUpdateMsgReceivedFlag(void);
void clearDeviceUpdateMsgReceivedFlag(void);

BYTE getResponseType(void);

void stepResponseMonitorStateMachine(BYTE newByte, BYTE expectedResponseType);

void sendASIPSensorDataMsg(BYTE * levels);
void addFlowRateMessage(BYTE * message);
void sendFlowRateMessages(void);

void sendFwUpdatePacketAckMessage(void);
void sendFwUpdatePacketNackMessage(void);
void sendStartFwUpdateAckMessage(void);
void sendFwUpdateCompleteAckMessage(void);

void sendDeviceInfo(void);
BYTE * getFwDataPtr(void);

void storeDeviceInfoData(BYTE * data);

BYTE getRunDeviceInfoSeq(void);
void setRunDeviceInfoSeq(void);
void clearRunDeviceInfoSeq(void);

void transmitDeviceInfoASIP(void);
void transmitRawAsipMessage(void);

void sendBITMessage(WORD id, BYTE severity);

void transmitCalStatusInfoASIP(BYTE length, BYTE* dataBuffer);
void sendStartCalAckMessage(void);
BYTE getCalStatusAckReceivedFlag(void);
void clearCalStatusAckReceivedFlag(void);

void resetDeviceInfoBuffer(void);

#endif // ASIP_H
