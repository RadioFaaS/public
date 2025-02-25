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
#include "VersionString.h"
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


#ifndef SENSOR_MODULE_BUILD_VERSION
#define BUILD_VERSION "Local build\0"
#else
#define BUILD_VERSION_ESCAPE(x) #x
#define BUILD_VERSION_TEMP(x) BUILD_VERSION_ESCAPE(x)
#define BUILD_VERSION BUILD_VERSION_TEMP(SENSOR_MODULE_BUILD_VERSION)
#endif

const char buildVersion[] PROGMEM = BUILD_VERSION;

const char FwPartNum[] PROGMEM  = "501010-000052\0\0\0";

//the timer will take 0.9765625 ms with no clk prescaler (16 MHz clk) to reach 
//this count of 15625. So our 1 Tick period will be ~0.9765625 ms which is close
// to 1 milli sec
/*==============================================================================
   TYPEDEFS
 *============================================================================*/


/*==============================================================================
   GLOBAL VARIABLES
 *============================================================================*/

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

static BYTE outgoingMsg[74];

/*==============================================================================
   STATIC FUNCTIONS
 *============================================================================*/
static void SchedulerInit(void);
static void hwInit(void);
static void readHwConfig(void);
static void readSerialNum(void);
static void readFwVersion(void);
static void buildDeviceInfo(void);
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

// Read and store the device firmware version from Flash
static void readFwVersion(void)
{
    PGM_P p; // Pointer used to locate const strings stored in PROGMEM
    
    // Retrieve the version string from Flash and then copy
    memcpy_P(&p, &versionInfoArray[0], sizeof(PGM_P));
    strcpy_P(fwVersion, p);
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
};


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
    // Initialize scheduler
    SchedulerInit();

    // Enable interrupts
    _SEI();

    // Enable watch dog timer for 2 second duration
    wdt_enable(WDTO_2S);

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
    asm("jmp (0x1800*2)");  	//jump to boot sector
}

void checkVersionInfoRequests(void)
{
    BYTE reqFlags;
    static BYTE pendingTxFlag = 0;
    static BYTE txBytesRemaining = 0;
    static BYTE txIndex = 0;
    
    if(getPingRequestFlag())
    {
        sendCANPingResponseMsg();
        clearPingRequestFlag();
    }
    
    reqFlags = getCANDataRequestFlags();
    if(reqFlags)
    {
        buildDeviceInfo();
        pendingTxFlag = 1;
        txBytesRemaining = 74;
        txIndex = 0;
        // Clear the appropriate flag
        setCANDataRequestFlags(reqFlags & ~(0x01));
    }
    if((pendingTxFlag) && (txBytesRemaining))
    {
        sendDeviceInfoData(&outgoingMsg[txIndex]);
        txIndex += 8;
        if(txBytesRemaining < 8)
        {
            txBytesRemaining = 0;
            pendingTxFlag = 0;
            txIndex = 0;
        } 
        else
        {
            txBytesRemaining -= 8;
        }
    }
}


static void buildDeviceInfo(void)
{
    BYTE i;
    BYTE * tempPtr;

    tempPtr = getSerialNumPtr();

    for(i = 0; i < 74; i++)
    {
        outgoingMsg[i] = 0;
    }

    // Build payload
    for(i = 0; i < 10; i++)
    {
        outgoingMsg[i] = *tempPtr++;
    }

    tempPtr = (BYTE *)getFwVersionPtr();

    for(i = 10; i < 26; i++)
    {
        outgoingMsg[i] = *tempPtr++;
    }

    strcpy_P(&outgoingMsg[26], FwPartNum);

    readHwVersion(&outgoingMsg[42]);
    readHwPartNum(&outgoingMsg[58]);
}

/*******************************************************************************
                                End of File
*******************************************************************************/