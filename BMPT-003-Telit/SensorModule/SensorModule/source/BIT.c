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

static BYTE systemErrorState;


void initBitMonitor(void)
{
    systemErrorState = ERROR_STATE_NORMAL;
}

void stepBitMonitorStateMachine(void)
{
    switch(systemErrorState)
    {
        case ERROR_STATE_NORMAL:
            if((!getCANTrafficDetected()) && (get30SecStartupTimer()))
            {
                systemErrorState = ERROR_STATE_NO_CAN;
            }
            break;
        
        case ERROR_STATE_NO_CAN:     
            if(getCANTrafficDetected())
            {
                // CAN is active again
                systemErrorState = ERROR_STATE_NORMAL;
            }       
            break;

        case ERROR_STATE_SENSOR_OOR:            
            break;

        case ERROR_STATE_NO_WIFI:           
            break;
        
        default:
            break;
    }
}

BYTE getActiveSystemError(void)
{
    return (systemErrorState == ERROR_STATE_NORMAL)?0:1;     
}