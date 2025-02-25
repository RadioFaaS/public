/*******************************************************************************
 * Copyright 2005 Appareo Systems
 * All rights reserved
 * This software and/or material is the property of Appareo Systems.
 * All use, disclosure, and/or reproduction not specifically authorized in
 * writing by Appareo Systems is prohibited.
 *
 * PROJECT:       GAU
 *
 * FILE NAME:     $Workfile:    $
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
#ifndef _GAU_MAIN_H_
#define _GAU_MAIN_H_


/*==============================================================================
   INCLUDE FILES
 *============================================================================*/
#include "Config.h"


/*==============================================================================
   DEFINES AND ENUMERATIONS
 *============================================================================*/
	   

/*==============================================================================
   TYPEDEFS
 *============================================================================*/

 #define OCR1A_COUNT_FOR_TICK   15625 // 1024 Hz ticks @ 16 MHz
 //#define OCR1A_COUNT_FOR_TICK   7812 // 1024 Hz ticks @ 8 MHz

/*==============================================================================
   EXTERN GLOBAL VARIABLES
 *============================================================================*/


/*==============================================================================
   GLOBAL FUNCTIONS
 *============================================================================*/ 
 BYTE getHwConfig(void);
 BYTE * getSerialNumPtr(void);
 void readHwPartNum(BYTE * partNumPtr);
 void readHwVersion(BYTE * versionPtr);
 char * getFwVersionPtr(void);
 void jumpToBootloader(void);
 void checkVersionInfoRequests(void);

 void writeSerialNumToEeprom(BYTE * serialPtr);
 void writeHwVersionToEeprom(BYTE * versionPtr);
 void writeHwPartNumToEeprom(BYTE * hwPartNumPtr, BYTE segmentNumber);

#endif // _GAU_MAIN_H_
/*******************************************************************************/


/*******************************************************************************
                                End of File
*******************************************************************************/
