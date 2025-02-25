/*
 * SensorModuleBoot.c
 *
 * Created: 7/16/2012 4:30:42 PM
 *  Author: sbailey
 */

#include <avr/io.h>

/*******************************************************************************
 * Copyright 2012 Appareo Systems
 * All rights reserved
 * This software and/or material is the property of Appareo Systems.
 * All use, disclosure, and/or reproduction not specifically authorized in
 * writing by Appareo Systems is prohibited.
 *
 * PROJECT:       Neptune Sensor Module bootloader
 *
 * FILE NAME:     $Workfile:   .c  $
 *
 * DESCRIPTION:   Main  file.
 *
 * REVISION HISTORY:
 *
 * $Log:$
 *  DATE:       WHO:  CHANGE DESCRIPTION:
 * ======================================================================
 02/16/2007  VS   Created. Rev 0.1.0
 *******************************************************************************/
/*==============================================================================
   INCLUDE FILES
 *============================================================================*/
#include <avr/boot.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include "config.h"
//#include "can_lib.h"
#include "NValloc.h"
#include "UART.h"
#include "WiFiModule.h"
#include "SensorModuleWirelessBoot.h"
#include "ASIP.h"

/*==============================================================================
   DEFINES AND ENUMERATIONS
 *============================================================================*/
// Task scheduler defines.

 #define OCR1A_COUNT_FOR_TICK   15625 // 1024 Hz ticks @ 16 MHz
 //#define OCR1A_COUNT_FOR_TICK   7812 // 1024 Hz ticks @ 8 MHz

#define RUN_2_TICK_MASK         0x01
#define RUN_4_TICK_MASK         0x02
#define RUN_8_TICK_MASK         0x04
#define RUN_16_TICK_MASK        0x08

#define LAST_TICK               0xFF

//BL must start programming within a timeout period:
#define BL_START_TIMEOUT	    	30000 //# of 1.95 ms loops = ~60 sec
#define BL_START_TIMEOUT_NO_RMS     3000  //6 sec
// We hope that Display will bootload itself and start bootloading Sensor Module within 15 sec

#define DLC_MAX            8
#define CH_DISABLE         0x00
#define CH_RxENA           0x80
#define CH_TxENA           0x40

#define PAGE_BUFFER_SIZE   (SPM_PAGESIZE + 1) // +1 for circular buffer head/tail overlap


 /*==============================================================================
   TYPEDEFS
 *============================================================================*/


/*==============================================================================
   GLOBAL VARIABLES
 *============================================================================*/
DWORD gdwTickCount;
//WORD toggleCount = 0;

/*==============================================================================
   STATIC VARIABLES
 *============================================================================*/
static BYTE scbyCurrentTick;
static BYTE scbyTicksPending;
//static BYTE jumpToAppFlag;
//static WORD bootloaderTimer;

//flag that is set every time a packet is received and reset when packet is read
static U8 programPacketReceived;
static WORD programPacketCount;
//flag to indicate that bootloading has started
static BYTE appProgrammingStarted;
//prog page buffer of page size SPM_PAGESIZE i.e. 256
static BYTE scbyProgPgBuff[PAGE_BUFFER_SIZE];
static WORD scwProgPgHead = 0;
static WORD scwProgPgTail = 0;
//static WORD scwIndex;  //index in the above bufer

//static BYTE checksumRequestFlag; //req for chksum received flag
static DWORD scdwAddress;//address in program memory being programmed
//static BYTE scbySleep;//no action once this flag becomes set. that is, invalid program
static BYTE scflEeUpdateReq;//flag gets set when EEPROM update is required as per Rms
static DWORD receivedChecksum;
static BYTE scbyChksm1;
static DWORD scbyChksm2;
static DWORD scbyChksm3;
static BYTE scflVerReq; //flag to indicate if Rms has requested for  versions
static char fwVersion[LENGTH_SW_VER];
static BYTE appErrorMsgSent;
static BYTE startBootMsgSent;
static BYTE readyToUpdateFlag;
static BYTE fwUpdateCompleteFlag;
static BYTE wifiConfigDone;
static char serialNum[LENGTH_SERIAL_NUM];
//static BYTE outgoingMsg[80];
static WORD flashCount;

//static BYTE scflRmsBlStarted;

const char FwPartNum[] PROGMEM  = "601010-000001\0\0\0"; // Wireless Sensor bootloader
const char FwVersion[] PROGMEM =  "2.0.0.0001\0\0\0\0\0\0";//Format "x.x.x.xxxx\0"//"Unknown\0";

/*==============================================================================
   STATIC FUNCTIONS
 *============================================================================*/
static void sendProgDataAck(void);
static WORD getPageBufferUsed(void);
static BYTE popPageBuffer(void);
static void readSerialNum(void);
static void readFwVersion(void);

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
FUNCTION NAME: hwInit

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
    DDRB = 0;
    DDRC = 0;
    DDRD = 0;
    CONFIG_PORTB_DDR;
    CONFIG_PORTC_DDR;
    CONFIG_PORTD_DDR;
    CONFIG_PORTE_DDR;

    PORTB = 0;
    PORTD = 0;

    SET_WIFI_RESET_HIGH;

} /* hwInit */


static void driverInit(void)
{
	initUART();
}


static void JumpToApp(void)
{
   _CLI(); //disable all interrupts
   //move interrupts back to application section
   //at this point IVSEL bit is set and we need to reset it
   MCUCR |= 0x01;//set the IVCE bit
   MCUCR = 0x00; //reset the IVSEL and IVCE bit
   asm("jmp (0x0000*2)");  	//jump to application
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

static void sendProgDataAck(void)
{
    BYTE ackData[] = {0x00, 0x31};

    buildAsipTransmitMessage(0x01, 0x00, 2, &ackData[0]);
    transmitAsipMessage();
}

void sendReadyForFwUpdate(void)
{
    BYTE mode;

    mode = 3; // FW update mode
    buildAsipTransmitMessage(0x00, 0x05, 1, &mode);
    transmitAsipMessage();
}


static void sendInvalidProgMsg(void)
{

}

//this will send the Sensor Module FW version as read from Eeprom
static void sendSensorFwVer(void)
{

}

void setWifiDoneFlag(void)
{
    wifiConfigDone = 1;
}


/*******************************************************************************
FUNCTION NAME: main

DESCRIPTION:   Main application entry point and task scheduler.

ARGUMENTS:     None

RETURN:        None

REVISION HISTORY:

DATE       WHO      CHANGE DESCRIPTION:
======================================================================
03/27/2005  J       Initial Creation
01/12/2006 RVW      Scheduler period changed to 250 ms
                    Tasks changed to correspond with a power of 2 samples per 250 ms
*******************************************************************************/
int main(void)
{
   //BYTE byI;
   WORD wJ;
   //BYTE* pbyVal;

   BYTE pendingUpdateFlag = 0xFF;
   BYTE validAppFlag = 0xFF;
   appErrorMsgSent = 0;
   startBootMsgSent = 0;
   readyToUpdateFlag = 0;
   fwUpdateCompleteFlag = 0;
   WORD toggleCount = 0;
   wifiConfigDone = 0;
   flashCount = 512;
   //BYTE wifiInitDone = 0;
   //WORD wifiInitTimer = 0;

	BYTE pageTemp[2];

   _CLI(); //disable all interrupts
   //move interrupts to bootloader section from application section
   MCUCR = 0x01; //set the IVCE bit in MCUCR
   MCUCR = 0x02; //set the IVSEL bit and reset the IVCE bit within 4clk cycles from last line

   // Enable watch dog timer for 2 second duration
   wdt_enable(WDTO_2S);

   //initialization of other static vars
   programPacketReceived = 0;
   programPacketCount = 0;
   appProgrammingStarted = 0;
   scdwAddress = 0;
   //checksumRequestFlag = 0;
   //scbySleep = 0;
   scflEeUpdateReq = 0;
   scbyChksm1 = 0;
   scbyChksm2 = 0;
   scbyChksm3 = 0;
   receivedChecksum = 0;
   scflVerReq = 0;
   //bootloaderTimer = 0;

   hwInit();

   driverInit();

   SchedulerInit();
   readSerialNum();
   readFwVersion();

   TURN_LED_ON;

   	eeprom_busy_wait();
   	validAppFlag = eeprom_read_byte((BYTE *)EE_PROGM_DATA_VALID_LOC);
   	eeprom_busy_wait();
   	pendingUpdateFlag = eeprom_read_byte((BYTE *)EE_FW_UPDATE_PENDING_LOC);

	// Clear out update pending flag right away
	if(pendingUpdateFlag != 0)
	{
		eeprom_busy_wait();
		eeprom_write_byte((BYTE *)EE_FW_UPDATE_PENDING_LOC, 0x00);
        eeprom_busy_wait();
        eeprom_write_byte((BYTE*)EE_PROGM_DATA_VALID_LOC, 0xFF);  // Clear the valid app flag
        eeprom_busy_wait();
        validAppFlag = 0xFF;
	}

    // Clear the flag indicating that the Wifi module is properly configured
    eeprom_busy_wait();
    eeprom_write_byte((BYTE *)EE_WIFI_CONFIG_STATE_LOC, 0xFF);
   /* Enable interrupts */
   _SEI();

   /* Start the task manager loop. */
   //while(1);
   for (;;)
   {
	 wdt_reset();
     //if (scbySleep == 0)
	 //{


    //

        //if timed out waiting for start of programming/jump cmd
	//jump to application if last programming attempt was successful
	if((validAppFlag == 0) && (pendingUpdateFlag != 0x01) && (wifiConfigDone))
	{
		eeprom_busy_wait();
		// EEPROM update not required as we did not receive any command
		eeprom_write_byte((BYTE*)EE_UPDATE_REQ_LOC, 0xFF);
		eeprom_busy_wait();
		JumpToApp();
	}
	else if(validAppFlag != 0)//last programming was not successful if it ever happened
	{
		//scbySleep = 1;
		if(!appErrorMsgSent)
        {
            appErrorMsgSent = 1;
            sendInvalidProgMsg();
            // TODO: Fast flash LEDs here...
            flashCount = 128;
        }

	}
    if(pendingUpdateFlag == 0x01)
    {
		if(!startBootMsgSent)
		{
    		startBootMsgSent = 1;
            readyToUpdateFlag = 1;
    		sendReadyForFwUpdate();
		}
    }


    if (scbyTicksPending != 0) //Ticks Pending != 0
    {
        scbyTicksPending--;
        if (scbyCurrentTick == LAST_TICK)
        {
            scbyCurrentTick = 0;
        }
        else
        {
            scbyCurrentTick++;
        }

        //every ~0.97 milli sec
        if (scbyCurrentTick & RUN_2_TICK_MASK)//Run 2 Tick Tasks
        {

			if((pendingUpdateFlag != 0x01) && (!wifiConfigDone))
			{
				stepInitWiFiModule();
			}
            if((wifiConfigDone) && (validAppFlag == 0))
            {
                JumpToApp();
            }

            toggleCount++;

            if(toggleCount > flashCount)
            {
                if(!wifiConfigDone)
                {
                    TOGGLE_ROW3_LED;
                }
                else
                {
                    TOGGLE_ROW3_LED;
                    TOGGLE_ROW4_LED;
                    TOGGLE_ROW2_LED;
                    TOGGLE_LED;
                }
                toggleCount = 0;
            }

            wdt_reset();

            processValidAsipMessages();

        }//every ~0.97 milli sec
    }
	else//will run all the time otherwise when a 1 tick or 2 tick task is not running
	{
		wdt_reset();
        if (getPageBufferUsed() >= SPM_PAGESIZE)  // Enough data has arrived to process
		{

			_CLI();//disable interrupts
			boot_page_erase(scdwAddress);
			while(boot_rww_busy())
			{
				boot_rww_enable();
			}

			//prepare for the page write
			for (wJ = 0; wJ < SPM_PAGESIZE; wJ += 2)
			{
				pageTemp[0] = popPageBuffer();
				pageTemp[1] = popPageBuffer();
				boot_page_fill(wJ, *(WORD*) pageTemp);
				scbyChksm2 = scbyChksm2 + pageTemp[0] + pageTemp[1];
			}

			//finally write to the page
			boot_page_write(scdwAddress);
			scdwAddress = scdwAddress + SPM_PAGESIZE;
			while(boot_rww_busy())
			{
				boot_rww_enable();
			}
			_SEI();//enable interrupts

			////sendProgDataAck(); //send Ack
		}//if (getPageBufferUsed() > SPM_PAGESIZE)

		//if (checksumRequestFlag == 1) //request for checksum has come from display
		if(fwUpdateCompleteFlag)
        {
			receivedChecksum = getReceivedChecksum();

            //checksumRequestFlag = 0; //reset the request
			//now we will calculate the checksum
			scdwAddress = 0;

 			while (scdwAddress < 57344)
			{
				scbyChksm3 = scbyChksm3 + pgm_read_byte_near((WORD)scdwAddress);
				scdwAddress++;
			}

            if((scbyChksm2 == receivedChecksum) && (scbyChksm3 == receivedChecksum))
            {
                // Success!!
                // set the flag in EE as program is valid
                _CLI();
                eeprom_busy_wait();
                eeprom_write_byte((BYTE*)EE_PROGM_DATA_VALID_LOC, 0);
                eeprom_busy_wait();
                eeprom_write_byte((BYTE*)EE_FW_UPDATE_PENDING_LOC, 0);
                validAppFlag = 0;
                eeprom_busy_wait();
                JumpToApp();

            }
            else
            {
                // Checksum fail :(

                // Reset everything for future update attempts
                //appErrorMsgSent = 0;
                startBootMsgSent = 0;
                readyToUpdateFlag = 0;
                fwUpdateCompleteFlag = 0;
                programPacketReceived = 0;
                programPacketCount = 0;
                appProgrammingStarted = 0;
                //scwIndex = 0;
                scdwAddress = 0;
                //checksumRequestFlag = 0;
                //scbySleep = 0;
                //scflEeUpdateReq = 0;
                //scbyChksm1 = 0;
                scbyChksm2 = 0;
                scbyChksm3 = 0;
                receivedChecksum = 0;
                //scflVerReq = 0;
                sendInvalidProgMsg();
            }

			//sendChecksumToDisplay(); //send the checksum to the display
		}
	}
     //}//if (scbySleep == 0)
  } //for(;;)
   return (0);
}


static WORD getPageBufferUsed(void)
{
	WORD nUsed = 0; // 0 if head = tail

	if(scwProgPgTail > scwProgPgHead)
	{
		nUsed = scwProgPgTail - scwProgPgHead;// + 1;
	}
	else if(scwProgPgTail < scwProgPgHead)
	{
		nUsed = PAGE_BUFFER_SIZE - scwProgPgHead + scwProgPgTail;
	}

	return nUsed;
}

static BYTE popPageBuffer(void)
{
	BYTE ret = 0x00;

	if(scwProgPgTail != scwProgPgHead)
	{
		ret = scbyProgPgBuff[scwProgPgHead++];
		if(scwProgPgHead >= PAGE_BUFFER_SIZE)
		{
			scwProgPgHead = 0;
		}
	}

	return ret;
}


/************************************************************************/
/* Pushes a buffer onto the page write buffer. If the buffer given is
   larger than the available amount of ram, the write will be truncated.
   Number of bytes written is returned.								*/
/************************************************************************/
WORD pushPageBuffer(BYTE * pBuf, WORD nBytes)
{
    WORD nMaxPush;
    WORD i;

    nMaxPush = PAGE_BUFFER_SIZE - getPageBufferUsed() - 1;
    if(nBytes > nMaxPush)
    {
        nBytes = nMaxPush;
    }

    for(i = 0; i < nBytes; i++)
    {
        scbyProgPgBuff[scwProgPgTail++] = pBuf[i];
        if(scwProgPgTail >= PAGE_BUFFER_SIZE)
        {
            scwProgPgTail = 0;
        }
    }

    return nBytes;
}

void setFwUpdateCompleteFlag(void)
{
    fwUpdateCompleteFlag = 1;
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
    strcpy_P(fwVersion, FwVersion);
}

char * getSerialNumPtr(void)
{
    return &serialNum[0];
}

// Returns a pointer to the characters of the firmware version string
char * getFwVersionPtr(void)
{
    return (char *)(&fwVersion[0]);
}

BYTE compareSerialNums(BYTE * num1, BYTE * num2)
{
    BYTE i;
    BYTE result = 0;

    for(i = 0; i < 4; i++)
    {
        if((*(num1 + i)) == (*(num2 + i)))
        {
            result = 1;
        }
        else
        {
            result = 0;
            break;
        }
    }

    return result;
}

/*******************************************************************************
                                End of File
*******************************************************************************/




