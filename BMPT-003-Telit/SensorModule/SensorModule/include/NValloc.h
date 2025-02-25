/*******************************************************************************
* Copyright 2005 Appareo Systems
* All rights reserved
* This software and/or material is the property of Appareo Systems.
* All use, disclosure, and/or reproduction not specifically authorized in
* writing by Appareo Systems is prohibited.
*
* PROJECT:       Navigation Suite
*
* FILE NAME:     $Workfile:   NValloc.h  $
*
* DESCRIPTION:   Eeprom allocation.
*
* REVISION HISTORY:
*******************************************************************************/

/*==============================================================================
   COMPILER DIRECTIVES
*============================================================================*/
#ifndef __NAV_EE_ALLOC_H__
#define __NAV_EE_ALLOC_H__

/*==============================================================================
   INCLUDE FILES
*============================================================================*/

/*==============================================================================
   DEFINES AND ENUMERATIONS
*============================================================================*/

#define MAX_EE_SIZE				   512 

#define LENGTH_EE_VER              16 //Format "x.x.x.xxxx\0"
#define LENGTH_SW_VER              16 //Format "x.x.x.xxxx\0"
#define LENGTH_HW_VER              16 //Format "x.x.x\0"
#define LENGTH_FW_PART_NUM         16 //Format "501010-xxxxxx\0"
#define LENGTH_HW_PART_NUM         16 //Format "xxxxxx-xxxxxx\0"
#define LENGTH_SERIAL_NUM          10 //Format "ABC-xxxx\0"  - Last 4 chars alpha-numeric
#define LENGTH_SENSOR_CAL          56 //Little Endian words storing the no-ball ADC reading for each sensor
#define LENGTH_SENSOR_BASE         56


//Description of Parameter                                                           
//************************* 


//Note: the below three Version must be together in that order in EEPROM
//EE version to be stored here while updating EE by _EE file
#define EE_EE_VER_LOC				    (MAX_EE_SIZE - LENGTH_EE_VER)
//Software Version to be stored from CFG file while bootloading
#define EE_SW_VER_LOC				    (EE_EE_VER_LOC - LENGTH_SW_VER)

#define EE_HW_VER_LOC                   (EE_SW_VER_LOC - LENGTH_SW_VER)

#define EE_FW_PART_NUM_LOC              (EE_HW_VER_LOC - LENGTH_HW_VER)

#define EE_HW_PART_NUM_LOC              (EE_FW_PART_NUM_LOC - LENGTH_HW_PART_NUM)

#define EE_SERIAL_NUM_LOC               (EE_HW_PART_NUM_LOC - LENGTH_SERIAL_NUM)

#define EE_SENSOR_CAL_LOC               (EE_SERIAL_NUM_LOC - LENGTH_SENSOR_CAL)

//this is the location that has program data validity flag stored
#define EE_PROGM_DATA_VALID_LOC       (EE_SENSOR_CAL_LOC - 1)
//location where EE update requested flag is saved
#define EE_UPDATE_REQ_LOC             (EE_PROGM_DATA_VALID_LOC - 1)	

#define EE_HW_CONFIG_LOC           (EE_UPDATE_REQ_LOC - 1)

#define EE_FW_UPDATE_PENDING_LOC    (EE_HW_CONFIG_LOC - 1)        

#define EE_SENSOR_BASE_LOC          (EE_FW_UPDATE_PENDING_LOC - LENGTH_SENSOR_BASE)

#define EE_NO_MAG_MASK_LOC          (EE_SENSOR_BASE_LOC - 1)

#endif