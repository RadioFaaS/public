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

 //#define OCR3A_COUNT_FOR_TICK   15625
 #define OCR1A_COUNT_FOR_TICK   15625 // 1024 Hz ticks @ 16 MHz
 //#define OCR1A_COUNT_FOR_TICK   7812 // 1024 Hz ticks @ 8 MHz
 
 #define UNIT_DEV_INFO_REQ_BITMASK		0x01
 #define FW_UPDATE_ACK_RESP_BITMASK		0x02
 #define START_FW_UPDATE_RESP_BITMASK	0x04
 #define FW_UPDATE_DATA_RX_BITMASK		0x08
 #define DEV_INFO_READY_BITMASK			0x10	
 #define SYSTEM_DEV_INFO_REQ_BITMASK	0x20
 #define BIT_DATA_RX_BITMASK			0x40
 #define FIELD_CAL_REQ_BITMASK			0x80

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
 void readFwPartNum(BYTE * fwPartNum);
 char * getFwVersionPtr(void);
 void jumpToBootloader(void);
 void checkSystemDataRequests(void);

 void writeSerialNumToEeprom(BYTE * serialPtr);
 void writeHwVersionToEeprom(BYTE * versionPtr);
 void writeHwPartNumToEeprom(BYTE * hwPartNumPtr, BYTE segmentNumber);

 void readWifiStatus(void);

#endif // _GAU_MAIN_H_
/*******************************************************************************/


/*******************************************************************************
                                End of File
*******************************************************************************/
