/*******************************************************************************
 * Copyright 2012 Appareo Systems
 * All rights reserved
 * This software and/or material is the property of Appareo Systems.
 * All use, disclosure, and/or reproduction not specifically authorized in
 * writing by Appareo Systems is prohibited.
 *
 * PROJECT:       Neptune LBMS Sensor Module
 *
 * FILE NAME:     $Workfile:   SensorModule.c  $
 *
 * DESCRIPTION:   Main task scheduler file.
 *
 * REVISION HISTORY:
 *
 * $Log:$
*******************************************************************************/
/*==============================================================================
   INCLUDE FILES
 *============================================================================*/
#include "Config.h"
#include "SensorModule.h"
#include "TaskList.h" 
#include "NValloc.h"
#include "BlockageProcessor.h"
#include "CAN.h"
#include "LED.h"
#include "UART.h"
#include "ASIP.h"
#include "ADC.h"
#include "VersionString.h"
#include "BIT.h"
#include "WiFiModule.h"
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>

/*==============================================================================
   DEFINES AND ENUMERATIONS
 *============================================================================*/
// Task scheduler defines.
#define RUN_2_TICK_MASK         0x01
#define RUN_4_TICK_MASK         0x02
#define RUN_8_TICK_MASK         0x04
#define RUN_16_TICK_MASK        0x08
#define RUN_32_TICK_MASK        0x10
#define RUN_64_TICK_MASK        0x20
#define RUN_128_TICK_MASK       0x40
#define RUN_256_TICK_MASK       0x80

#define LAST_TICK               0xFF

#define DEV_INFO_TX_RETRY_COUNT	4
#define DEV_INFO_DELAY_TIME		4
#define SENSOR_PING_RETRY_COUNT 3
#define TIMER_COUNT_1_5_SEC_4HZ	6


#ifndef SENSOR_MODULE_BUILD_VERSION
#define BUILD_VERSION   "Local build\0"
#else
#define BUILD_VERSION_ESCAPE(x) #x
#define BUILD_VERSION_TEMP(x) BUILD_VERSION_ESCAPE(x)
#define BUILD_VERSION BUILD_VERSION_TEMP(SENSOR_MODULE_BUILD_VERSION)
#endif

const char buildVersion[] PROGMEM = "2.0.0.0001\0\0\0\0\0\0";//Format "x.x.x.xxxx\0"//BUILD_VERSION;

//const char * const FwVersion   = buildVersion;
const char FwPartNum[] PROGMEM  = "601010-000001\0\0\0";

//the timer will take 0.9765625 ms with no clk prescaler (16 MHz clk) to reach 
//this count of 15625. So our 1 Tick period will be ~0.9765625 ms which is close
// to 1 milli sec
/*==============================================================================
   TYPEDEFS
 *============================================================================*/


/*==============================================================================
   GLOBAL VARIABLES
 *============================================================================*/
//DWORD gdwTickCount;

PGM_P const versionInfoArray[] PROGMEM = {
    buildVersion,
    FwPartNum
};

/*==============================================================================
   STATIC VARIABLES
 *============================================================================*/
static BYTE scbyCurrentTick; 
static BYTE scbyTicksPending;
static BYTE hwConfig;
static char serialNum[LENGTH_SERIAL_NUM];
static char fwVersion[LENGTH_SW_VER];
static BYTE wifiStatus;

static BYTE outgoingDevInfoMsg[74];
static BYTE calStatusDataReady = 0;
/*==============================================================================
   STATIC FUNCTIONS
 *============================================================================*/
static void SchedulerInit(void);
static void hwInit(void);
static void readHwConfig(void);
static void readSerialNum(void);
static void readFwVersion(void);
static void buildCANDeviceInfo(void);
static void fillLocalCalStatusData();
/*******************************************************************************
FUNCTION NAME: SchedulerInit

DESCRIPTION:   Scheduler initialization.

ARGUMENTS:     None

RETURN:        None

REVISION HISTORY:

DATE         WHO              CHANGE DESCRIPTION:
======================================================================
10/10/2006   Vijay Soni       Initial Creation
10/16/2006   Robert Weinmann  Added Interrupt Enable for Output Compare match
*******************************************************************************/
static void SchedulerInit(void)
{
    TCCR1A = 0; // Use OCR1A for TOP count
    TCCR1B = _BV(WGM12) | _BV(CS10); // CTC mode, no clock pre-scaler
    TIMSK1 |= _BV(OCIE1A); // Enable Output Compare A Match interrupt
    OCR1A = OCR1A_COUNT_FOR_TICK;    

    scbyCurrentTick = LAST_TICK; //initial value, next value will be 0
    scbyTicksPending = 0;
}

/*******************************************************************************
FUNCTION NAME: GauHwInit

DESCRIPTION:   Ports initialization.

ARGUMENTS:     None

RETURN:        None

REVISION HISTORY:

DATE       WHO      CHANGE DESCRIPTION:
======================================================================
07/07/2005  VS       Initial Creation
*******************************************************************************/
static void hwInit(void)
{
    CONFIG_PORTB_DDR;
    CONFIG_PORTC_DDR;
    CONFIG_PORTD_DDR;
    CONFIG_PORTE_DDR;
    
    PORTB = 0;
    PORTD = 0;
    
} /* hwInit */
/*******************************************************************************/

// Read and store the HW config (Wired vs Wireless) from EEPROM 
static void readHwConfig(void)
{
    eeprom_busy_wait();

    if (eeprom_read_byte((BYTE *)EE_HW_CONFIG_LOC) == 1)
    {
        hwConfig = HW_TYPE_WIRELESS;
    }
    else
    {
        hwConfig = HW_TYPE_WIRED;
    }
}

// Read and store the device Serial Number from EEPROM
static void readSerialNum(void)
{
    BYTE i;
    
    for(i = 0; i < LENGTH_SERIAL_NUM; i++)
    {
        eeprom_busy_wait();
        serialNum[i] = eeprom_read_byte((BYTE *)(EE_SERIAL_NUM_LOC + i));
    }
}

void readHwPartNum(BYTE * partNumPtr)
{
    BYTE i;
    
    for(i = 0; i < LENGTH_HW_PART_NUM; i++)
    {
        eeprom_busy_wait();
        partNumPtr[i] = eeprom_read_byte((BYTE *)(EE_HW_PART_NUM_LOC + i));
    }    
}

void readHwVersion(BYTE * versionPtr)
{
    BYTE i;
    
    for(i = 0; i < LENGTH_HW_VER; i++)
    {
        eeprom_busy_wait();
        versionPtr[i] = eeprom_read_byte((BYTE *)(EE_HW_VER_LOC + i));
    }
}

void readFwPartNum(BYTE * fwPartNum)
{  
    strcpy_P((char *)fwPartNum, FwPartNum);
}

// Read and store the device firmware version from Flash
static void readFwVersion(void)
{
    PGM_P p; // Pointer used to locate const strings stored in PROGMEM
    
    // Retrieve the version string from Flash and then copy
    memcpy_P(&p, &versionInfoArray[0], sizeof(PGM_P));
    strcpy_P(fwVersion, p);
}

void readWifiStatus(void)
{
    eeprom_busy_wait();
    wifiStatus = eeprom_read_byte((BYTE *)(EE_WIFI_CONFIG_STATE_LOC));
}


void writeSerialNumToEeprom(BYTE * serialPtr)
{
    BYTE i;

    _CLI();
    for(i = 0; i < 8; i++)
    {
        eeprom_busy_wait();
        eeprom_write_byte((BYTE *)(EE_SERIAL_NUM_LOC + i), *serialPtr);
        serialPtr++;
    }
    eeprom_busy_wait();
    eeprom_write_byte((BYTE *)(EE_SERIAL_NUM_LOC + 8), '\0');
    _SEI();
}

void writeHwVersionToEeprom(BYTE * versionPtr)
{
    BYTE i;

    _CLI();
    for(i = 0; i < 5; i++)
    {
        eeprom_busy_wait();
        eeprom_write_byte((BYTE *)(EE_HW_VER_LOC + i), *versionPtr);
        versionPtr++;
    }
    eeprom_busy_wait();
    eeprom_write_byte((BYTE *)(EE_HW_VER_LOC + 5), '\0');
    _SEI();
}

void writeHwPartNumToEeprom(BYTE * hwPartNumPtr, BYTE segmentNumber)
{
    BYTE i;
    BYTE writeSize = 0;
    BYTE writeOffset = 0;

    if(segmentNumber == 0)
    {
        writeSize = 8;
        writeOffset = 0;
    }
    else if(segmentNumber == 1)
    {
        writeSize = 5;
        writeOffset = 8;
    }

    _CLI();
    for(i = 0; i < writeSize; i++)
    {
        eeprom_busy_wait();
        eeprom_write_byte((BYTE *)(EE_HW_PART_NUM_LOC + writeOffset + i), *hwPartNumPtr);
        hwPartNumPtr++;
    }
    eeprom_busy_wait();
    eeprom_write_byte((BYTE *)(EE_HW_PART_NUM_LOC + 13), '\0');
    _SEI();
}

/*******************************************************************************
FUNCTION NAME: MainTimerInterrupt

DESCRIPTION:   Handle the timer interrupt.

ARGUMENTS:     None

RETURN:        None

REVISION HISTORY:

DATE:       WHO:  CHANGE DESCRIPTION:
======================================================================
05/08/2004  BWT   Created.
*******************************************************************************/
ISR(TIMER1_COMPA_vect)
{
    scbyTicksPending++;
    OCR1A  = OCR1A_COUNT_FOR_TICK; // reset output compare value;		
} 
/*******************************************************************************/

/*******************************************************************************/

int main(void)
{    

    _CLI(); //disable all interrupts
    //move interrupts back to application section
    //at this point IVSEL bit is set and we need to reset it
    MCUCR |= 0x01;//set the IVCE bit
    MCUCR = 0x00; //reset the IVSEL and IVCE bit    
    //Hardware port initialization
    hwInit();
    // Read config data from EEPROM
    readHwConfig();
    readSerialNum();
    readFwVersion();
    readCalValues();
    
    // Initialize application level routines.
    appInit();
    turnOnAllLEDs();
	initUART();
    // Initialize scheduler
    SchedulerInit();
    // Enable interrupts
    _SEI();

    // Enable watch dog timer for 2 second duration
    wdt_enable(WDTO_2S);
    //sendDeviceInfo();
	resetConnectedSensors();
    sendPingAllSensors(); // Prevent connected sensors from 'No CAN traffic' error state
    /* Start the task manager loop. */
    for (;;)
    {
        //Run0TickTask calls functions that are not time critical.
        //As the 0Tick Task is only serviced when idle
        if (scbyTicksPending == 0)//Ticks Pending == 0
        {
            wdt_reset();
            Run0TickTasks();
            
        }
        else  // Ticks Pending is > 0
        {            
			// _CLI();// disable interrupts as scbyTicksPending  and scbyCurrentTick is read by interrupt routines
            wdt_reset(); //service the watch dog timer every 1 ms
            scbyTicksPending--;
            // _SEI(); // enable interrupts
            if (scbyCurrentTick == LAST_TICK)// Last Tick
            {
                scbyCurrentTick = 0;
            }
            else
            {
                scbyCurrentTick++;
            }
            
            Run1TickTasks(); //every ~0.97 ms

            //Run one of the following tasks every time as well.
            if (scbyCurrentTick & RUN_2_TICK_MASK)//Run 2 Tick Tasks
            {
                Run2TickTasks(); // every ~1.95 ms
            }
            else if (scbyCurrentTick & RUN_4_TICK_MASK)// Run 4 Tick Tasks
            {
                Run4TickTasks(); // every ~3.9 ms
            }
            else if (scbyCurrentTick & RUN_8_TICK_MASK)// Run 8 Tick Tasks
            {
                Run8TickTasks(); //every ~7.8 ms
            }
            else if (scbyCurrentTick & RUN_16_TICK_MASK)// Run 16 Tick Tasks
            {
                Run16TickTasks(); //every ~15.6 ms
            }
            else if (scbyCurrentTick & RUN_32_TICK_MASK)// Run 32 Tick Tasks
            {
                Run32TickTasks(); // every ~31.2 ms
            }
            else if (scbyCurrentTick & RUN_64_TICK_MASK)
            {
                Run64TickTasks(); //every ~62.1 ms
            }
            else if (scbyCurrentTick & RUN_128_TICK_MASK)
            {
                Run128TickTasks(); //every ~124.1 ms
            }
            else if (scbyCurrentTick & RUN_256_TICK_MASK)
            {
                Run256TickTasks(); // every ~248.3 ms
            }			
        }// Ticks Pending > 0*/
    } //for(;;)

    return (0);

}

// Returns the device HW config (Wired vs Wireless) 
BYTE getHwConfig(void)
{
    return hwConfig;
}

// Returns a pointer to the characters of the serial number string
// This assumes a format of ABC-xxxx
BYTE * getSerialNumPtr(void)
{
    return (BYTE *)(&serialNum[0]); 
}

// Returns a pointer to the characters of the firmware version string
char * getFwVersionPtr(void)
{
    return (char *)(&fwVersion[0]);
}

void jumpToBootloader(void)
{
    _CLI(); //disable all interrupts
    //move interrupts back to application section
    //at this point IVSEL bit is set and we need to reset it
    MCUCR |= 0x01;//set the IVCE bit
    MCUCR = 0x02; //reset the IVSEL and IVCE bit
#ifdef MCU_64M1
    asm("jmp (0x7000*2)");  	//jump to boot sector
#else
    asm("jmp (0x1800*2)");  	//jump to boot sector
#endif
}

// Handle data ready to be relayed between
//  the display and sensors via WiFi or CAN
void checkSystemDataRequests(void)
{
    BYTE reqFlags;
    static BYTE pendingTxFlag = 0;
    static BYTE pendingDevInfoTxFlag = 0;
    static BYTE txBytesRemaining = 0;
    static BYTE txDevInfoBytesRemaining = 0;
    static BYTE txIndex = 0;
    static BYTE txDevInfoIndex = 0;
    //static BYTE ackReceived = 0;
    static BYTE txRetryCount = 0;
    static BYTE devInfoDelayCount = 0;
	static BYTE pingCount = 0;
	static BYTE devInfoTxCount = 0;
	static BYTE lastDevInfoSent = 0;
	static BYTE wirelessDevInfoPending = 0;
	static BYTE devInfoTimeoutFlag = 0;
	static BYTE devInfoTimeoutTxRetry = 0;
    BYTE tempByte;
    BYTE devInfoDone;
	
	// Check if the CAN seq for 
	//  gathering device info has timed out
	if((getRunDeviceInfoSeq()) && (getDevInfoTimer() >= TIMER_COUNT_1_5_SEC_4HZ))
	{
		// Abort the dev info seq and move on
		resetDeviceInfoBuffer();
		devInfoTimeoutFlag = 1;
		reqFlags = getCANDataRequestFlags();
	}
	
    reqFlags = getCANDataRequestFlags();	
    
    if(reqFlags)
    {
        if(reqFlags & UNIT_DEV_INFO_REQ_BITMASK)
        {
            // build FW version msg...
            buildCANDeviceInfo();
            pendingDevInfoTxFlag = 1;
            txDevInfoBytesRemaining = 74;
            txDevInfoIndex = 0;
			reqFlags = getCANDataRequestFlags();
            setCANDataRequestFlags(reqFlags & ~(UNIT_DEV_INFO_REQ_BITMASK));

        }
        if(reqFlags & FW_UPDATE_ACK_RESP_BITMASK)
        {
            // Send FW packet ack message
			reqFlags = getCANDataRequestFlags();
            setCANDataRequestFlags(reqFlags & ~(FW_UPDATE_ACK_RESP_BITMASK));
        }
        if(reqFlags & START_FW_UPDATE_RESP_BITMASK)
        {
            // Send FW Update Start ack message
            disableSensorDataTx();
            if((getIsTxIdle()) && (!getRunDeviceInfoSeq()))
            {
                //Don't clear the flag until a data packet arrives - resend until successful
                sendStartFwUpdateAckMessage();
            }
        }
        if(reqFlags & FW_UPDATE_DATA_RX_BITMASK)
        {
            // FW Update data 
            pendingTxFlag = 1;
            txBytesRemaining = 250;
            txIndex = 0;
			reqFlags = getCANDataRequestFlags();
			setCANDataRequestFlags(reqFlags & ~(FW_UPDATE_DATA_RX_BITMASK));
        }
        if(reqFlags & DEV_INFO_READY_BITMASK) // Send a device info packet via ASIP
        {
			if(!getUARTTxBuffEmpty())			
			{
				clearDevInfoTimer();
				return;
			}
			if(!devInfoTimeoutFlag) // Transmit if the last CAN sequence did not time out
			{
				transmitDeviceInfoASIP();
				devInfoTxCount++;
				devInfoTimeoutTxRetry = 0;
			}
			else // A timeout has occurred
			{
				if(devInfoTimeoutTxRetry < 2)
				{
					//Re-send the last dev info query
					resendLastTargetedDeviceInfoReq();
					devInfoTimeoutFlag = 0;
					devInfoTimeoutTxRetry++;				
					clearDevInfoTimer();
				}
				else
				{
					// Abort and move on to the next sensor
					devInfoTimeoutTxRetry = 0;
				}				
			}			
			if((devInfoTxCount > DEV_INFO_TX_RETRY_COUNT) || (devInfoTimeoutFlag))
			{
				devInfoTxCount = 0;
				reqFlags = getCANDataRequestFlags();
				setCANDataRequestFlags(reqFlags & ~(DEV_INFO_READY_BITMASK));
				
              // Kick off the next device info request
              devInfoDone = sendNextTargetedDeviceInfoReq();
			  clearDevInfoTimer(); // Reset the timer for receiving CAN data
              
              if(devInfoDone)
              {
	              if(lastDevInfoSent)
				  {
					lastDevInfoSent = 0;
					clearRunDeviceInfoSeq();
					enableSensorDataTx();
				  }
				  else
				  {
					lastDevInfoSent = 1; // Force to iterate through one more time to Tx the last data
				  }
              }				
			}
			devInfoTimeoutFlag = 0;            
            wdt_reset();
        }
        if(reqFlags & SYSTEM_DEV_INFO_REQ_BITMASK) // Prep for Device Info sequence
        {
			setRunDeviceInfoSeq();
			devInfoTimeoutFlag = 0;
			
			if(wirelessDevInfoPending)
			{
				if(!getUARTTxBuffEmpty())			
				{
					clearDevInfoTimer();
					return;
				}			
				sendDeviceInfo();
				devInfoTxCount++;
				
				if(devInfoTxCount > DEV_INFO_TX_RETRY_COUNT)
				{
					reqFlags = getCANDataRequestFlags();
					setCANDataRequestFlags(reqFlags & ~(SYSTEM_DEV_INFO_REQ_BITMASK));
					devInfoTxCount = 0;
					wirelessDevInfoPending = 0;
					
					if(getConnectedSensorCount() > 0)
					{
						sendNextTargetedDeviceInfoReq();
						clearDevInfoTimer(); // Reset the timer for receiving CAN data
					}
					else
					{
						clearRunDeviceInfoSeq();
						enableSensorDataTx();
					}
				}				
				return;
			}
			
            devInfoDelayCount++;

            if(devInfoDelayCount >= DEV_INFO_DELAY_TIME)  // Wait ~250ms for ping responses
            {
                if(pingCount < SENSOR_PING_RETRY_COUNT)
				{
					sendPingAllSensors();
					pingCount++;
					devInfoDelayCount = 0;
					return;
				}
				
				devInfoDelayCount = 0;
                pingCount = 0;
				
                // Get the # of wired sensors that have responded to the ping plus the wireless
                tempByte = getConnectedSensorCount() + 1;
				sendSensorCount(tempByte);
                sendDeviceInfo(); // Push the wireless sensor device info into the Tx queue

				wirelessDevInfoPending = 1;
                wdt_reset();        
            }
        }
        if(reqFlags & BIT_DATA_RX_BITMASK) // BIT failure reported by CAN
        {
            tempByte = getLastActiveBIT();
			
			if((tempByte != 0xFF) && (!getRunDeviceInfoSeq()))
			{
				sendBITMessage((WORD)(tempByte), 0); // Severity 'critical'
				reqFlags = getCANDataRequestFlags();
				setCANDataRequestFlags(reqFlags & ~(BIT_DATA_RX_BITMASK));
			}
        }
        if(reqFlags & FIELD_CAL_REQ_BITMASK) // Cal Status messages pending
        {
            tempByte = getConnectedSensorCount();
            
            if((getCalStatusResponseCount() >= tempByte) && (!getRunCalibrationFlag()))
            {                
                tempByte++; // Increment sensor count to include this sensor
                
                if(!calStatusDataReady)
                {
                    fillLocalCalStatusData();
                    calStatusDataReady = 1;
                }
                if((!getCalStatusAckReceivedFlag()) && (txRetryCount < 80)) // Retry results message for ~5 sec @ 62ms retry loop
                {
                    // Now Tx cal data via ASIP/WiFi
                    transmitCalStatusInfoASIP(tempByte, getFwDataPtr());
                    txRetryCount++;
                }
                else
                {
					reqFlags = getCANDataRequestFlags();
                    setCANDataRequestFlags(reqFlags & ~(FIELD_CAL_REQ_BITMASK));
                    clearCalStatusAckReceivedFlag();
                    calStatusDataReady = 0;
                    txRetryCount = 0;
                    enableSensorDataTx();
                    sendCANSensorDataEnableMsg();
                }
            }
        }
    }
    
    // Send more FW image data if the sensor module is ready
    if((pendingTxFlag) && (txBytesRemaining))
    {        
        if(txBytesRemaining < 8)
        {
            // With a starting packet size of 250 this remainder should always be 2 bytes
            sendCANFwUpdateShortDataPacket(getFwDataPtr() + txIndex, txBytesRemaining);
            txBytesRemaining = 0;
            pendingTxFlag = 0;
            txIndex = 0;

            sendFwUpdatePacketAckMessage(); // Send the Packet Ack to the display
        }
        else
        {
            sendCANFwUpdateDataPacket(getFwDataPtr() + txIndex);
            txIndex += 8;
            txBytesRemaining -= 8;
        }
    }
    // Also check for device info CAN requests
    if((pendingDevInfoTxFlag) && (txDevInfoBytesRemaining))
    {
        sendDeviceInfoData(&outgoingDevInfoMsg[txDevInfoIndex]);
        txDevInfoIndex += 8;
        if(txDevInfoBytesRemaining < 8)
        {
            txDevInfoBytesRemaining = 0;
            pendingDevInfoTxFlag = 0;
            txDevInfoIndex = 0;
        }
        else
        {
            txDevInfoBytesRemaining -= 8;
        }
    }

}

// Assemble this sensor's device info data in the pre-allocated buffer
static void buildCANDeviceInfo(void)
{
    BYTE i;
    BYTE * tempPtr;

    tempPtr = getSerialNumPtr();

    for(i = 0; i < 74; i++)
    {
        outgoingDevInfoMsg[i] = 0;
    }

    // Build payload
    for(i = 0; i < 10; i++)
    {
        outgoingDevInfoMsg[i] = *tempPtr++;
    }

    tempPtr = (BYTE *)getFwVersionPtr();

    for(i = 10; i < 26; i++)
    {
        outgoingDevInfoMsg[i] = *tempPtr++;
    }

    strcpy_P(&outgoingDevInfoMsg[26], FwPartNum);

    readHwVersion(&outgoingDevInfoMsg[42]);
    readHwPartNum(&outgoingDevInfoMsg[58]);

}

// Copy the local calibration results
//  into the ASIP transmit buffer
static void fillLocalCalStatusData(void)
{
    BYTE statusData[7];
    BYTE* serialNumPtr;

    getCalStatusData(&statusData[0], &statusData[1], &statusData[2]);

    serialNumPtr = getSerialNumPtr();
    statusData[3] = *(serialNumPtr + 4);
    statusData[4] = *(serialNumPtr + 5);
    statusData[5] = *(serialNumPtr + 6);
    statusData[6] = *(serialNumPtr + 7);

    copyCalStatusDataToTxBuffer(&statusData[0]);
}

/*******************************************************************************
                                End of File
*******************************************************************************/