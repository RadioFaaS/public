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
#ifndef _TASKLIST_H_
#define _TASKLIST_H_


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


/*==============================================================================
   EXTERN GLOBAL VARIABLES
 *============================================================================*/


/*==============================================================================
   GLOBAL FUNCTIONS
 *============================================================================*/
void appInit(void);
void Run0TickTasks(void);
void Run1TickTasks(void);
void Run2TickTasks(void);
void Run4TickTasks(void);
void Run8TickTasks(void);
void Run16TickTasks(void);
void Run32TickTasks(void);
void Run64TickTasks(void);
void Run128TickTasks(void);
void Run256TickTasks(void);

BYTE get4HzTimer(void);
void clear4HzTimer(void);
BYTE get30SecStartupTimer(void);

#endif // _GAU_MAIN_H_
/*******************************************************************************/


/*******************************************************************************
                                End of File
*******************************************************************************/
