
#ifndef BIT_H_
#define BIT_H_

#include "Config.h"

enum
{
    ERROR_STATE_NORMAL,
    ERROR_STATE_NO_CAN,
    ERROR_STATE_SENSOR_OOR,
    ERROR_STATE_NO_WIFI    
};

void initBitMonitor(void);
void stepBitMonitorStateMachine(void);
BYTE getActiveSystemError(void);

void setLastActiveBIT(BYTE val);
BYTE getLastActiveBIT(void);

#endif /* BIT_H_ */