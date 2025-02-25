/*******************************************************************************
 * Copyright 2012 Appareo Systems
 * All rights reserved
 * This software and/or material is the property of Appareo Systems.
 * All use, disclosure, and/or reproduction not specifically authorized in
 * writing by Appareo Systems is prohibited.
 *
 * PROJECT:       Neptune LBMS
 *
 * FILE NAME:     $Workfile:   TaskList.c  $
 *
 * DESCRIPTION:   Task List.
 *
 * REVISION HISTORY:
* $Log:$
*******************************************************************************/
/*==============================================================================
   INCLUDE FILES
 *============================================================================*/
#include "TaskList.h"							  
#include "SensorModule.h"
#include "CAN.h"
#include "ADC.h"
#include "DAC.h"
#include "BlockageProcessor.h"
#include "LED.h"
#include "SPI.h"
#include "WiFiModule.h"
#include "BIT.h"
#include "ASIP.h"

/*==============================================================================
   DEFINES AND ENUMERATIONS
 *============================================================================*/


/*==============================================================================
   TYPEDEFS
 *============================================================================*/


/*==============================================================================
   GLOBAL VARIABLES
 *============================================================================*/




/*==============================================================================
   STATIC VARIABLES
 *============================================================================*/
 static BYTE FourHzTimer;
 static BYTE thirtySecStartupTimer = 0;
/*==============================================================================
   STATIC FUNCTIONS
 *============================================================================*/



/*******************************************************************************
FUNCTION NAME: AppInit

DESCRIPTION:   Application initialization. This function is called before the
               scheduler starts executing application periodic functions.

               List all OSI calls here.

ARGUMENTS:     None

RETURN:        None

REVISION HISTORY:

DATE       WHO      CHANGE DESCRIPTION:
======================================================================
03/27/2005  J       Initial Creation
*******************************************************************************/
void appInit(void)
{
   initCAN();
   initADC();
   initDAC(); 
   initBitMonitor();
   initBlockageProcessor();
   initLEDStateMachine();

} /* AppInit */
/*******************************************************************************/

// Returns the ticks of a 4 Hz timer
BYTE get4HzTimer(void)
{
    return FourHzTimer;
}

// Clears the tick count of the 4 Hz timer
void clear4HzTimer(void)
{
    FourHzTimer = 0;
}

BYTE get30SecStartupTimer(void)
{
    return thirtySecStartupTimer;
}

/*******************************************************************************
FUNCTION NAME: Run0TickTasks

DESCRIPTION:   This function is called whenever Ticks Pending is 0 and Time before 
next tick is less than TBD 

ARGUMENTS:     None

RETURN:        None

REVISION HISTORY:

DATE       WHO      CHANGE DESCRIPTION:
======================================================================
01/12/2006  RVW       Initial Creation
*******************************************************************************/
void Run0TickTasks(void)
{
    //background tasks running when no task is running
}
/*******************************************************************************/


/*******************************************************************************
FUNCTION NAME: Run1TickTasks

DESCRIPTION:   This function is called  every ~0.97 milli sec period

ARGUMENTS:     None

RETURN:        None
REVISION HISTORY:

DATE       WHO      CHANGE DESCRIPTION:
======================================================================
01/12/2006  RVW       Initial Creation
*******************************************************************************/
void Run1TickTasks(void)
{

}
/*******************************************************************************/
/*******************************************************************************
FUNCTION NAME: Run2TickTasks

DESCRIPTION:   This function is called every ~1.95 milli sec period

ARGUMENTS:     None

RETURN:        None

REVISION HISTORY:

DATE       WHO      CHANGE DESCRIPTION:
======================================================================
01/12/2006  RVW       Initial Creation
*******************************************************************************/
void Run2TickTasks(void)
{

} /* Run2TickTasks */
/*******************************************************************************/

/*******************************************************************************
FUNCTION NAME: Run4TickTasks

DESCRIPTION:   This function is called every ~3.9 milli sec period

ARGUMENTS:     None

RETURN:        None

REVISION HISTORY:

DATE       WHO      CHANGE DESCRIPTION:
======================================================================
01/12/2006  RVW       Initial Creation
*******************************************************************************/
void Run4TickTasks(void)
{

} 
/*******************************************************************************/
/*******************************************************************************
FUNCTION NAME: Run8TickTasks

DESCRIPTION:   This function is called every ~7.8 milli sec period

ARGUMENTS:     None

RETURN:        None

REVISION HISTORY:

DATE       WHO      CHANGE DESCRIPTION:
======================================================================
01/12/2006  RVW       Initial Creation
*******************************************************************************/
void Run8TickTasks(void)
{   

}
/*******************************************************************************/


/*******************************************************************************
FUNCTION NAME: Run16TickTasks

DESCRIPTION:   This function is called  every ~15.6  milli sec

ARGUMENTS:     None

RETURN:        None

REVISION HISTORY:

DATE       WHO      CHANGE DESCRIPTION:
======================================================================
01/12/2006  RVW       Initial Creation
*******************************************************************************/
void Run16TickTasks(void)
{
    
} 
/*******************************************************************************/


/*******************************************************************************
FUNCTION NAME: Run32TickTasks

DESCRIPTION:   This function is called  every ~31.2 milli sec period

ARGUMENTS:     None

RETURN:        None

REVISION HISTORY:

DATE       WHO      CHANGE DESCRIPTION:
======================================================================
01/12/2006  RVW       Initial Creation
*******************************************************************************/
void Run32TickTasks(void)
{
  startADCConversion();
} 
/*******************************************************************************/


/*******************************************************************************
FUNCTION NAME: Run64TickTasks

DESCRIPTION:   This function is called every ~62.1 milli sec period

ARGUMENTS:     None

RETURN:        None

REVISION HISTORY:

DATE       WHO      CHANGE DESCRIPTION:
======================================================================
01/12/2006  RVW       Initial Creation
*******************************************************************************/
void Run64TickTasks(void)
{     
   stepBitMonitorStateMachine();
   checkVersionInfoRequests();
} 
/*******************************************************************************/


/*******************************************************************************
FUNCTION NAME: Run128TickTasks

DESCRIPTION:   This function is called every ~124.1 milli sec

ARGUMENTS:     None

RETURN:        None

REVISION HISTORY:

DATE       WHO      CHANGE DESCRIPTION:
======================================================================
01/12/2006  RVW       Initial Creation
*******************************************************************************/
void Run128TickTasks(void)
{
    if(getSensorsReadyFlag()) // If all sensors have been sampled 4x, do the averaging
    {
        averageADCData();
        applyCalibrationToData();
        clearSensorsReadyFlag();
    }
    if(!getRunCalibrationFlag())
    {
        stepLEDStateMachine();
    }
} 
/*******************************************************************************/


/*******************************************************************************
FUNCTION NAME: Run256TickTasks

DESCRIPTION:   This function is called every ~248.3 milli sec 

ARGUMENTS:     None

RETURN:        None

REVISION HISTORY:

DATE       WHO      CHANGE DESCRIPTION:
======================================================================
01/12/2006  RVW       Initial Creation
*******************************************************************************/
void Run256TickTasks(void)
{  
    FourHzTimer++;

    // Set a flag 30 seconds after startup
    if(FourHzTimer > 120)
    {
        thirtySecStartupTimer = 1;
    }
    processBlockageData();  
	stepEEUpdateTask();
} 
/*******************************************************************************
                                End of File
*******************************************************************************/
