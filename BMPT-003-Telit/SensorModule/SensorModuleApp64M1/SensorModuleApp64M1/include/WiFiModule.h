

#ifndef WIFIMODULE_H_
#define WIFIMODULE_H_

#include "Config.h"
#include "ASIP.h"


void sendWifiCommand(PGM_P command);
void readAsipOutCID(void);

void sendUdpPacket(AsipMsgStruct * pMessage);
void sendTcpPacket(AsipMsgStruct * pMessage);

void startUdpPacketTransmit(WORD size);

#endif /* WIFIMODULE_H_ */