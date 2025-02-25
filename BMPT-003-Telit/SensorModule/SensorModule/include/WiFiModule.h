

#ifndef WIFIMODULE_H_
#define WIFIMODULE_H_

#include "Config.h"



void initWiFiModule(void);
void setWifiResetLow(void);
void setWifiResetHigh(void);
void sendWifiCommand(BYTE arrayIndex);
void initWifiWakeupInterrupt(void);
BYTE getWifiReadyFlag(void);
BYTE getExpectedWifiResponseType(void);
char * getExpectedWifiResponsePtr(void);
void setAsipCID(BYTE val);
void setAsipServerCID(BYTE val);
BYTE getAsipServerCID(void);

#endif /* WIFIMODULE_H_ */