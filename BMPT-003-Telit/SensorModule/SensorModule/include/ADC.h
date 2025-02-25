/*******************************************************************************
 * Copyright 2005 Appareo Systems
 * All rights reserved
 * This software and/or material is the property of Appareo Systems.
 * All use, disclosure, and/or reproduction not specifically authorized in
 * writing by Appareo Systems is prohibited.
 *
 * PROJECT:       GAU
 *
 * FILE NAME:     $Workfile:ADCBios.h
 *
 * DESCRIPTION:   Main file header.
 *
 * REVISION HISTORY:
 *
 * $Log:$
*******************************************************************************/

/*==============================================================================
   COMPILER DIRECTIVES
 *============================================================================*/
#ifndef _ADC_BIOS_H_
#define _ADC_BIOS_H_

/*==============================================================================
   INCLUDE FILES
 *============================================================================*/
#include "config.h"

/*==============================================================================
   DEFINES AND ENUMERATIONS
 *============================================================================*/
// Enum for ADC channels
// Ordered to provide 'top' to 'bottom' channel sequence per hardware layout
enum
{  
   ADC_CHANNEL_2 = 0,
   ADC_CHANNEL_3,
   ADC_CHANNEL_5,
   ADC_CHANNEL_6,
   ADC_CHANNEL_7,
   ADC_CHANNEL_8,
   ADC_CHANNEL_9,
   ADC_LAST_CHANNEL
};

enum
{
    CAL_AVG_TOO_LOW = 0,
    CAL_AVG_TOO_HIGH,
    CAL_AVG_IN_RANGE
};

/*==============================================================================
   TYPEDEFS
 *============================================================================*/


/*==============================================================================
   GLOBAL FUNCTIONS
 *============================================================================*/
void initADC(void);
void startADCConversion(void);
void clearADCData(void);
void averageADCData(void);

WORD * getAveragedADCDataPtr(void);
WORD * getAccumulatedADCDataPtr(void);

void setNextSensorRow(void);

void selectRow1(void);
void selectRow2(void);
void selectRow3(void);
void selectRow4(void);

void startSensorCalibration(void);

BYTE getSensorsReadyFlag(void);
void clearSensorsReadyFlag(void);

BYTE getRunCalibrationFlag(void);

#endif