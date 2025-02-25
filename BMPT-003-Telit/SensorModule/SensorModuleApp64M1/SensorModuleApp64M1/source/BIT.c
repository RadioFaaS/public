/*
 * BIT.c
 *
 * Created: 8/7/2012 4:59:01 PM
 *  Author: sbailey
 */ 

#include "BIT.h"
#include "CAN.h"
#include "TaskList.h"
#include "BlockageProcessor.h"
#include "SensorModule.h"

static BYTE systemErrorState;
static BYTE lastActiveBIT = 0xFF;

void initBitMonitor(void)
{
    systemErrorState = ERROR_STATE_NORMAL;
}

void stepBitMonitorStateMachine(void)
{
    switch(systemErrorState)
    {
        case ERROR_STATE_NORMAL:
            if((!getCANTrafficDetected()) && (get30SecStartupTimer()) && (getHwConfig() == HW_TYPE_WIRED))
            {
                sendCANBitResultsMsg(0x01);
                systemErrorState = ERROR_STATE_NO_CAN;
            }
            else if(getSensorOORFlag())
            {
                sendCANBitResultsMsg(0x02);
                systemErrorState = ERROR_STATE_SENSOR_OOR;
            }
            break;
        
        case ERROR_STATE_NO_CAN:            
            break;

        case ERROR_STATE_SENSOR_OOR:            
            break;

        case ERROR_STATE_NO_WIFI:
            // TODO: Send CAN status message here...            
            break;
        
        default:
            break;
    }
}

BYTE getActiveSystemError(void)
{
    return (systemErrorState == ERROR_STATE_NORMAL)?0:1;     
}

void setLastActiveBIT(BYTE val)
{
    lastActiveBIT = val;
}

BYTE getLastActiveBIT(void)
{
    return lastActiveBIT;
}