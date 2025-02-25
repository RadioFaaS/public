

#ifndef WIFIMODULE_H_
#define WIFIMODULE_H_

#include "Config.h"
#include "ASIP.h"

enum 
{
    WIFI_RESPONSE_NONE = 0,
    WIFI_RESPONSE_OK,
    WIFI_RESPONSE_CONNECT_OUT,
    WIFI_RESPONSE_CONNECT_IN
};

enum
{
    CID_TYPE_ASIP_OUT = 0,
    CID_TYPE_ASIP_IN
};

//const char udpBulkTransferSeq[] PROGMEM = "\x1BZ";

void stepInitWiFiModule(void);
void setWifiResetLow(void);
void setWifiResetHigh(void);
//void sendWifiCommand(BYTE arrayIndex);
void sendWifiCommand(PGM_P command);
//void initWifiWakeupInterrupt(void);
BYTE getWifiReadyFlag(void);
BYTE getExpectedWifiResponseType(void);
//char * getExpectedWifiResponsePtr(void);
//void setAsipCID(BYTE val);
void setAsipCID(BYTE connection, BYTE val);
//void setAsipServerCID(BYTE val);
//BYTE getAsipServerCID(void);
BYTE * getAsipCIDPtr(void);
BYTE getWifiInitSuccess(void);

void sendUdpPacket(AsipMsgStruct * pMessage);

//void setupFwUpdateTcpPort(void);
#endif /* WIFIMODULE_H_ */