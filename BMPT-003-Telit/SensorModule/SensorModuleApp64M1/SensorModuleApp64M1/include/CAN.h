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
#ifndef _GAU_CAN_H_
#define _GAU_CAN_H_


/*==============================================================================
   INCLUDE FILES
 *============================================================================*/
#include "Config.h"
#include "can_lib.h"


/*==============================================================================
   DEFINES AND ENUMERATIONS
 *============================================================================*/
//NOTE: Any change here must accompany same change in the Display app/bootloader code and Sensor Module app/bootloader code
enum
{
    CANID_UNASSIGNED = 0,           //CAN ID 0 not assigned to any message

    #ifdef CAN_FILTER_ENABLED
    CANID_RESERVED,
    #else
    CANID_SENSOR_DATA,              //Magnet position data for rows of a Sensor Module
    #endif

    CANID_ACTIVE_BLOCKAGE,         //Command for Sensor Module to indicate blockage on given row(s)
    CANID_ENABLE_WIFI,             //Command to turn on WiFi module on Wireless Sensor Modules
    CANID_DISABLE_WIFI,            //Command to turn off WiFi module on Wireless Sensor Modules
    CANID_SERIAL_NUM,
    CANID_FW_PART_NUM,
    CANID_FW_VERSION,              //Sensor Module sends its FW Ver
    CANID_HW_PART_NUM,
    CANID_HW_VERSION,
    CANID_SENSOR_EE_VER,            //Sensor Module sends its EE ver
    CANID_DISPLAY_PROG_DATA_CMD,    //command to write 8 bytes of program data
    CANID_SENSOR_PROG_DATA_CMD,
    CANID_PROG_DATA_ACK,           //Ack of program data 8 bytes received
    CANID_CMD_JUMP_APP,         //Command from Display to Sensor Module Bootloader to jump to App
    CANID_REQ_CHECKSUM,         //Command to Display/Sensor Module bootloader requesting checksum of programmed data
    CANID_CHECKSUM_REPLY,          //Response from bootloader with checksum of programmed data
    CANID_SENSOR_INVALID_PROG,     //Sensor Module Bootloader response that program data is invalid
    CANID_DISPLAY_INVALID_PROG,    //Basic Display Bootloader response that program data is invalid
    CANID_START_EE_UPDATE,          // Commands system device to initiate EEPROM update for itself
    CANID_EE_DATA,                  //EEPROM data for Sensor Module sent from the Display
    CANID_EE_DATA_ACK,             //Acknowledgment for each packet of EEPROM data received
    CANID_DO_BIT,                   //Display directing Sensor Module to perform Bit
    CANID_SENSOR_BIT_RESULT,        //Sensor Module responding with Bit results
    CANID_DISPLAY_BIT_RESULT,      // Basic Display responding with Bit results

    CANID_REQ_DISPLAY_VERSIONS,
    CANID_REQ_SENSOR_VERSIONS,     //Display requests to know version info from Sensor Modules
    CANID_REQ_FW_PART_NUM,
    CANID_REQ_HW_VERSION,
    CANID_REQ_HW_PART_NUM,

    CANID_START_SENSOR_FW_UPDATE,
    CANID_START_DISPLAY_FW_UPDATE,
    CANID_READY_FOR_FW_UPDATE,     //Signals device has entered bootloader in prep for FW update

    CANID_DEVICE_INFO,
    CANID_WRITE_FW_VERS_1,
    CANID_WRITE_FW_VERS_2,
    CANID_WRITE_SERIAL_NUM,

    CANID_WRITE_HW_VERSION,
    CANID_DISABLE_SENSOR_DATA,
    CANID_ENABLE_SENSOR_DATA,

    CANID_FW_UPDATE_COMPLETE,

    CANID_START_CAL,               //Command to start Sensor Module factory calibration
    CANID_CAL_STATUS,
    CANID_SENSOR_PING_ALL,
    CANID_PING_RESPONSE,
    CANID_TARGETED_DEVICE_INFO_REQ,
    CANID_WRITE_HW_PART_NUM_1,
    CANID_WRITE_HW_PART_NUM_2,
    CANID_ENABLE_DEBUG_DATA,
    CANID_DISABLE_DEBUG_DATA,

	CANID_RAW_ADC1 = 90,
	CANID_RAW_ADC2,
	CANID_RAW_ADC3,
	CANID_RAW_ADC4,
	CANID_RAW_ADC5,
	CANID_RAW_ADC6,
	CANID_RAW_ADC7,
	
	
    #ifdef CAN_FILTER_ENABLED
    CANID_SENSOR_DATA = 128 // Must be 0x80 for message filtering to work
    #endif
};



/*==============================================================================
   TYPEDEFS
 *============================================================================*/


/*==============================================================================
   EXTERN GLOBAL VARIABLES
 *============================================================================*/



/*==============================================================================
   GLOBAL FUNCTIONS
 *============================================================================*/
void initCAN(void);

void ApiCANcmdShutdown(void);
void ApiSendEeUpdateReq(void);
void ApiAckEePktRecvd(void);

BOOL ApiIsEePktRecvd(void);
void ApiSendGauBitStatus(BYTE byStatus);
BOOL ApiIsBitReqd(void);
BOOL ApiStartBitCmd(void);
BOOL ApiIsEeCfgVerRecd(void);

void sendCANSensorDataMsg(BYTE * levels);

BYTE getActiveBlockageFlag(void);
void setActiveBlockageFlag(void);
void clearActiveBlockageFlag(void);
BYTE getActiveBlockageRows(void);
void setActiveBlockageRows(BYTE val);
BYTE * getActiveBlockageStartTimes(void);
BYTE getCANDataRequestFlags(void);
void setCANDataRequestFlags(BYTE val);

void sendCANActiveBlockageMsg(BYTE * serialNumPtr, BYTE rowMask);
void sendCANStartFwUpdateMsg(BYTE * serialNumPtr);
void sendCANRequestVersionsMsg(void);
void sendCANBitResultsMsg(BYTE bitVal);

void storeActiveBlockageData(BYTE blockageMask);

BYTE compareSerialNums(BYTE * num1, BYTE * num2);

BYTE getCANTrafficDetected(void);

void sendCANRawADC(WORD * rawPtr);
void sendCANRawADC2(WORD * rawPtr);
void sendCANRawADC3(WORD * rawPtr);
void sendCANRawADC4(WORD * rawPtr);
void sendCANRawADC5(WORD * rawPtr);
void sendCANRawADC6(WORD * rawPtr);
void sendCANRawADC7(WORD * rawPtr);

void retransmitBlockageData(void);
void fillBlockageDataBuffer(BYTE * data, BYTE size);

void sendCANFwUpdateDataPacket(BYTE * dataPtr);
void sendCANFwUpdateShortDataPacket(BYTE * dataPtr, BYTE count);

void sendCANSensorDataDisableMsg(void);
void sendCANSensorDataEnableMsg(void);

void sendCalStatusMsg(BYTE status, BYTE row, BYTE sensorNum);
void sendDeviceInfoData(BYTE * data);
void sendPingAllSensors(void);
void sendStartCalCommand(void);
void sendCANGetDeviceInfoBySerial(BYTE * serialNumPtr);
BYTE sendNextTargetedDeviceInfoReq(void);
BYTE getConnectedSensorCount(void);
void sendCANFwUpdateComplete(void);

BYTE getCalStatusResponseCount(void);
void clearCalStatusResponseCount(void);
void copyCalStatusDataToTxBuffer(BYTE* newData);

void stepEEUpdateTask(void);

void sendUdpRawADC(void);
void resetConnectedSensors(void);
void sendSensorCount(BYTE count);
void resendLastTargetedDeviceInfoReq(void);
#endif // _GAU_MAIN_H_
/*******************************************************************************/


/*******************************************************************************
                                End of File
*******************************************************************************/
