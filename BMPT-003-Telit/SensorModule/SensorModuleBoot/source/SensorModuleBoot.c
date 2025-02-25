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
#include "can_lib.h"
#include "NValloc.h"


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
//#define BL_START_TIMEOUT	    	30000 //# of 1.95 ms loops = ~60 sec
//#define BL_START_TIMEOUT_NO_RMS     3000  //6 sec
// We hope that Display will bootload itself and start bootloading Sensor Module within 15 sec

#define DLC_MAX            8
#define CH_DISABLE         0x00
#define CH_RxENA           0x80
#define CH_TxENA           0x40

#define INV_APP_RETRY_TIME  5000  // 500 ticks per second

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

    #ifdef CAN_FILTER_ENABLED
    CANID_SENSOR_DATA = 128 // Must be 0x80 for message filtering to work
    #endif
};

 /*==============================================================================
   TYPEDEFS
 *============================================================================*/


/*==============================================================================
   GLOBAL VARIABLES
 *============================================================================*/
DWORD gdwTickCount;
WORD toggleCount = 0;

/*==============================================================================
   STATIC VARIABLES
 *============================================================================*/
static BYTE scbyCurrentTick;
static BYTE scbyTicksPending;
static BYTE jumpToAppFlag;
//static WORD bootloaderTimer;

static st_cmd_t scCANmsgTx;	//used for msg transfer
//8 bytes Tx buffer used for CAN msg txmsn
U8 abyCANTxBuff[8];
//Rx buffer one is enough for rx interrupt routine
U8 abyCANRxBuff[8];
//flag that is set every time a packet is received and reset when packet is read
static U8 programPacketReceived;
static BYTE shortProgramPacketReceived;
static WORD programPacketCount;
//flag to indicate that bootloading has started
static BYTE appProgrammingStarted;
//prog page buffer of page size SPM_PAGESIZE i.e. 256
static BYTE scbyProgPgBuff[SPM_PAGESIZE];
static WORD scwIndex;  //index in the above bufer
static BYTE checksumRequestFlag; //req for chksum received flag
static DWORD scdwAddress;//address in program memory being programmed
//static DWORD scdwChecksumAddress;
//static BYTE scbySleep;//no action once this flag becomes set. that is, invalid program
static BYTE scflEeUpdateReq;//flag gets set when EEPROM update is required as per Rms
static DWORD receivedChecksum;
//static BYTE scbyChksm1;
static DWORD scbyChksm2;
static DWORD scbyChksm3;
static BYTE scflVerReq; //flag to indicate if Rms has requested for  versions
static BYTE scabySwVer[LENGTH_SW_VER];
static BYTE appErrorMsgSent;
static BYTE startBootMsgSent;
static BYTE readyToUpdateFlag;
static BYTE fwUpdateCompleteFlag;
static BYTE deviceInfoReadyFlag;
static BYTE pingRequest;
static BYTE fwUpdateRequest;
static BYTE receivedFwByteCount;
static BYTE activeError;
static WORD commTimer;
//static BYTE scflRmsBlStarted;

static BYTE outgoingMsg[80];
static char serialNum[LENGTH_SERIAL_NUM];
//static char fwVersion[LENGTH_SW_VER];

const char FwPartNum[] PROGMEM  = "501010-000052\0\0\0"; // Wired Sensor bootloader
const char FwVersion[] PROGMEM = "Unknown\0";
/*==============================================================================
   STATIC FUNCTIONS
 *============================================================================*/
 static void sendReadyForFwUpdate(void);
 static void buildDeviceInfo(void);
 static void readSerialNum(void);
 static void sendPingResponse(void);
 static void sendChecksums(void);

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
    CONFIG_PORTB_DDR;
    CONFIG_PORTC_DDR;
    CONFIG_PORTD_DDR;
    CONFIG_PORTE_DDR;

    PORTB = 0;
    PORTD = 0;

} /* hwInit */

static void JumpToApp(void)
{
   //TOGGLE_LED_1;
PORTB |= (_BV(PB4) | (_BV(PB1)) | (_BV(PB3)));
PORTC |= (_BV(PC6));
   _CLI(); //disable all interrupts
   //move interrupts back to application section
   //at this point IVSEL bit is set and we need to reset it
   MCUCR |= 0x01;//set the IVCE bit
   MCUCR = 0x00; //reset the IVSEL and IVCE bit
   asm("jmp (0x0000*2)");  	//jump to application
}


/*******************************************************************************/

static void CANinit(void)
{
  Can_reset();
  //- Set CAN Bit-timing
  can_init((U16)CAN_BAUDRATE);        // c.f. macro in "can_drv.h"
  //- Set CAN Timer Prescaler
  CANTCON = CANBT1;

  scCANmsgTx.pt_data = &abyCANTxBuff[0]; //point to buffer
  scCANmsgTx.status = STATUS_CLEARED;
  scCANmsgTx.cmd = CMD_TX_DATA;
  scCANmsgTx.dlc = 8;

  /* MOB  init for reception */
  CANPAGE = (5 << 4);
  CANSTMOB  = 0x00;                            /* reset channel status */
  CANCDMOB = CH_DISABLE;                       /* reset control and dlc register */
  /* Reception Channel : mask */
  CANIDM1 = 0x00;
  CANIDM2 &= ~0xE0;
  CANIDM4 = 0;

  /* Reception Channel configuration */
  CANIDT4 &=~0x04;                             /* clear bit rtr in CANIDT4. */
  CANCDMOB |= DLC_MAX;                         /* Reception 8 bytes.*/
  CANCDMOB |= CH_RxENA;                        /* Reception enabled without buffer.*/

  /* interrupt configuration - enable interrupts at reception channel*/
  //CANIE1 |= 0x40; //turn on IEMOB14 bit to enable interrupts on MOB 14
  CANIE2 |= 0x20; //turn on IEMOB5 bit to enable interrupts on MOB 5
  CANGIE = ((1<<ENRX) | (1<<ENIT));       /* Can_Rx & IT enable */
}

ISR(CAN_INT_vect)
{
	   unsigned int id;       /* can_data index */
	   BYTE* byPtr;
	   BYTE	byCtr;
       BYTE dlc;
       //PORTB |= (_BV(PB1));
	   CANPAGE = (5 << 4);    /*  select channel  to see if it is Rx interrupt */
	   if((CANSTMOB & MOB_RX_COMPLETED) == MOB_RX_COMPLETED)
	   {
	      id = (((int)(CANIDT2))>>5) + (((int)(CANIDT1))<<3);       // V2.0 part A

		    byPtr = (BYTE*)&abyCANRxBuff[0];

			//Now take appropriate action based on id and data received
			
            if(id == CANID_SENSOR_PROG_DATA_CMD)
            {
			    for (byCtr = 0; byCtr < 8; byCtr++)
			    {
    			    *byPtr = CANMSG;
    			    byPtr++;
			    }
			    if(readyToUpdateFlag)
			    {
    			    programPacketReceived = 1;
                    // Check the DLC for data length
                    dlc = (CANCDMOB & 0x0F);

                    if(dlc < 8)
                    {
                        shortProgramPacketReceived = 1;
                        receivedFwByteCount = dlc;
                    }
    			    if (appProgrammingStarted == 0)
    			    {
        			    //set the start flag as first can msg has arrived with prog data
        			    appProgrammingStarted = 1;
        			    //we are going to program now. So set the flag in EE as invalid
        			    eeprom_busy_wait();
        			    eeprom_write_byte((BYTE*)EE_PROGM_DATA_VALID_LOC, 0xFF);
        			    eeprom_busy_wait();
    			    }
			    }
            }
            else if(id == CANID_REQ_CHECKSUM)
            {
			    for (byCtr = 0; byCtr < 8; byCtr++)
			    {
    			    *byPtr++ = CANMSG;
			    }
			    checksumRequestFlag = 1;          
            }
            else if(id == CANID_START_SENSOR_FW_UPDATE)
            {      
                startBootMsgSent = 1;
                readyToUpdateFlag = 1;   
                fwUpdateRequest = 1;      
            }
            else if(id == CANID_FW_UPDATE_COMPLETE)
            {
                fwUpdateCompleteFlag = 1;
            }            
            else if(id == CANID_CMD_JUMP_APP)
            {
				//command received to jump to app
				jumpToAppFlag = 1;
				scflEeUpdateReq = CANMSG;//flag for ee to be updated
				for (byCtr = 0; byCtr < LENGTH_SW_VER; byCtr++)
				{
    				scabySwVer[byCtr] = CANMSG;
				}                
            }         
            else if(id == CANID_REQ_SENSOR_VERSIONS)
            {
                                
            }
            else if(id == CANID_TARGETED_DEVICE_INFO_REQ)
            {
			    for (byCtr = 0; byCtr < 4; byCtr++)
			    {
    			    *byPtr++ = CANMSG;
			    }
			    if((serialNum[4] == abyCANRxBuff[0]) && (serialNum[5] == abyCANRxBuff[1]) &&
			    (serialNum[6] == abyCANRxBuff[2]) && (serialNum[7] == abyCANRxBuff[3]))
			    {
    			    buildDeviceInfo();
    			    deviceInfoReadyFlag = 1;
			    }                
            }
            else if(id == CANID_SENSOR_PING_ALL)
            {
                pingRequest = 1;
            }

	   }
	   else if (CANSTMOB & MOB_NOT_REACHED) //error in MOB 1
	   {
		  _NOP();
	   }
	   byCtr = CANSTMOB;
       
       CANSTMOB = 0x00;                             /* reset channel 0 status */
	   CANCDMOB = DLC_MAX;                         /* receive 8 bytes */
	   CANCDMOB |= CH_RxENA;                       /* reception enable */
	   CANGIT = CANGIT;                            /* reset all flags */
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
    while (can_get_status(&scCANmsgTx) == CAN_STATUS_NOT_COMPLETED);
    scCANmsgTx.dlc = 0;
    //*((DWORD*)&abyCANTxBuff[0]) = 0;       //fill the data bytes
	//*((DWORD*)&abyCANTxBuff[4]) = 0;	     //fill the data bytes
	scCANmsgTx.id.std = CANID_PROG_DATA_ACK;
	while (can_cmd(&scCANmsgTx) != CAN_CMD_ACCEPTED);
}

static void sendReadyForFwUpdate(void)
{  
    while (can_get_status(&scCANmsgTx) == CAN_STATUS_NOT_COMPLETED);
    scCANmsgTx.dlc = 0;
	scCANmsgTx.id.std = CANID_READY_FOR_FW_UPDATE; 
	while (can_cmd(&scCANmsgTx) != CAN_CMD_ACCEPTED);
}

static void sendPingResponse(void)
{
    while (can_get_status(&scCANmsgTx) == CAN_STATUS_NOT_COMPLETED);
    abyCANTxBuff[0] = serialNum[4];
    abyCANTxBuff[1] = serialNum[5];
    abyCANTxBuff[2] = serialNum[6];
    abyCANTxBuff[3] = serialNum[7];
    scCANmsgTx.dlc = 4;
    scCANmsgTx.id.std = CANID_PING_RESPONSE;
    while (can_cmd(&scCANmsgTx) != CAN_CMD_ACCEPTED);
}


static void sendInvalidProgMsg(void)
{
    while (can_get_status(&scCANmsgTx) == CAN_STATUS_NOT_COMPLETED);
    scCANmsgTx.dlc = 0;
    //*((DWORD*)&abyCANTxBuff[0]) = 0;
    //*((DWORD*)&abyCANTxBuff[4]) = 0;
    scCANmsgTx.id.std = CANID_SENSOR_INVALID_PROG;
	while (can_cmd(&scCANmsgTx) != CAN_CMD_ACCEPTED);
}


static void sendDeviceInfoData(void)
{
    BYTE i;

    for(i = 0; i < 10; i++)
    {
        while (can_get_status(&scCANmsgTx) == CAN_STATUS_NOT_COMPLETED);
        scCANmsgTx.dlc = 8;
        abyCANTxBuff[0] = outgoingMsg[i * 8];
        abyCANTxBuff[1] = outgoingMsg[(i * 8) + 1];
        abyCANTxBuff[2] = outgoingMsg[(i * 8) + 2];
        abyCANTxBuff[3] = outgoingMsg[(i * 8) + 3];
        abyCANTxBuff[4] = outgoingMsg[(i * 8) + 4];
        abyCANTxBuff[5] = outgoingMsg[(i * 8) + 5];
        abyCANTxBuff[6] = outgoingMsg[(i * 8) + 6];
        abyCANTxBuff[7] = outgoingMsg[(i * 8) + 7];
        scCANmsgTx.id.std = CANID_DEVICE_INFO;
	    while (can_cmd(&scCANmsgTx) != CAN_CMD_ACCEPTED);     
        
        wdt_reset();   
    }   

    deviceInfoReadyFlag = 0;
}

static void sendChecksums(void)
{
    while (can_get_status(&scCANmsgTx) == CAN_STATUS_NOT_COMPLETED);
    scCANmsgTx.dlc = 8;
    abyCANTxBuff[0] = (BYTE)(scbyChksm2 & 0xFF);
    abyCANTxBuff[1] = (BYTE)((scbyChksm2 >> 8) & 0xFF);
    abyCANTxBuff[2] = (BYTE)((scbyChksm2 >> 16) & 0xFF);
    abyCANTxBuff[3] = (BYTE)((scbyChksm2 >> 24) & 0xFF);
    abyCANTxBuff[4] = (BYTE)(scbyChksm3 & 0xFF);
    abyCANTxBuff[5] = (BYTE)((scbyChksm3 >> 8) & 0xFF);
    abyCANTxBuff[6] = (BYTE)((scbyChksm3 >> 16) & 0xFF);
    abyCANTxBuff[7] = (BYTE)((scbyChksm3 >> 24) & 0xFF);
    scCANmsgTx.id.std = 0x50;//0x7F;
    while (can_cmd(&scCANmsgTx) != CAN_CMD_ACCEPTED);

}
/*
static void sendFwUpdateCompleteAck(void)
{

}
*/
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

static void readHwPartNum(BYTE * partNumPtr)
{
    BYTE i;
    
    for(i = 0; i < LENGTH_HW_PART_NUM; i++)
    {
        eeprom_busy_wait();
        partNumPtr[i] = eeprom_read_byte((BYTE *)(EE_HW_PART_NUM_LOC + i));
    }
}

static void readHwVersion(BYTE * versionPtr)
{
    BYTE i;
    
    for(i = 0; i < LENGTH_HW_VER; i++)
    {
        eeprom_busy_wait();
        versionPtr[i] = eeprom_read_byte((BYTE *)(EE_HW_VER_LOC + i));
    }
}


static void buildDeviceInfo(void)
{
    BYTE i;

    for(i = 0; i < 74; i++)
    {
        outgoingMsg[i] = 0;
    }

    // Build payload
    for(i = 0; i < 10; i++)
    {
        outgoingMsg[i] = serialNum[i];
    }

    strcpy_P(&outgoingMsg[10], FwVersion);

    strcpy_P(&outgoingMsg[26], FwPartNum);

    readHwVersion(&outgoingMsg[42]);
    readHwPartNum(&outgoingMsg[58]);

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
   BYTE byI;
   WORD wJ;
   BYTE* pbyVal;

   BYTE pendingUpdateFlag = 0xFF;
   BYTE validAppFlag = 0xFF;
   BYTE pageWriteCount = 0;
   appErrorMsgSent = 0;
   startBootMsgSent = 0;
   readyToUpdateFlag = 0;
   fwUpdateCompleteFlag = 0;
   deviceInfoReadyFlag = 0;
   pingRequest = 0;
   fwUpdateRequest = 0;
   activeError = 0;
   commTimer = 0;

   ////WORD toggleCount = 0;

   _CLI(); //disable all interrupts
   //move interrupts to bootloader section from application section
   MCUCR = 0x01; //set the IVCE bit in MCUCR
   MCUCR = 0x02; //set the IVSEL bit and reset the IVCE bit within 4clk cycles from last line

   // Enable watch dog timer for 2 second duration
   wdt_enable(WDTO_2S);

   //initialization of other static vars
   programPacketReceived = 0;
   shortProgramPacketReceived = 0;
   programPacketCount = 0;
   appProgrammingStarted = 0;
   scwIndex = 0;
   scdwAddress = 0;
   checksumRequestFlag = 0;
   //scbySleep = 0;
   scflEeUpdateReq = 0;
   //scbyChksm1 = 0;
   scbyChksm2 = 0;
   scbyChksm3 = 0;
   receivedChecksum = 0;
   scflVerReq = 0;
   //bootloaderTimer = 0;
   for (byI = 0; byI < LENGTH_SW_VER ; byI++)
   {
      scabySwVer[byI] = 0;
   }

   hwInit();
   readSerialNum();

   SchedulerInit();

   CANinit();

   TURN_LED_ON;

   	eeprom_busy_wait();
   	validAppFlag = eeprom_read_byte((BYTE *)EE_PROGM_DATA_VALID_LOC);
   	eeprom_busy_wait();
   	pendingUpdateFlag = eeprom_read_byte((BYTE *)EE_FW_UPDATE_PENDING_LOC);
   /* Enable interrupts */
   _SEI();

   /* Start the task manager loop. */
   //while(1);
   for (;;)
   {
	 wdt_reset();
     //if (scbySleep == 0)
	 //{

        //if timed out waiting for start of programming/jump cmd
	//jump to application if last programming attempt was successful
	if((validAppFlag == 0) && (pendingUpdateFlag != 0x01))
	{
		eeprom_busy_wait();
		// EEPROM update not required as we did not receive any command
		eeprom_write_byte((BYTE*)EE_UPDATE_REQ_LOC, 0xFF);
		eeprom_busy_wait();
		JumpToApp();
	}
	else if(validAppFlag != 0)//last programming was not successful if it ever happened
	{
        if(activeError == 0)
        {
            activeError = 1;
            // Turn off all LEDs to sync flashing
            PORTB &= ~(_BV(PB1));
            PORTC &= ~(_BV(PC6));
            PORTB &= ~(_BV(PB4));
            PORTB &= ~(_BV(PB3));
        }
		if(commTimer > INV_APP_RETRY_TIME)//!appErrorMsgSent)
        {
            appErrorMsgSent = 1;
            commTimer = 0;
            sendInvalidProgMsg();
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
	//if jump to app command has come from display
	if (jumpToAppFlag == 1)
	{
		eeprom_busy_wait();
		if (scflEeUpdateReq == 1)
		{
			//eeprom update requested
			eeprom_write_byte((BYTE*)EE_UPDATE_REQ_LOC, 0);
			eeprom_busy_wait();
		}
		else
		{
			if (eeprom_read_byte((BYTE*)EE_UPDATE_REQ_LOC) != 0xFF) //if flag already not 0xFF
			{
			    //set eeprom update not requested
				eeprom_busy_wait();
			    eeprom_write_byte((BYTE*)EE_UPDATE_REQ_LOC, 0xFF);
			}
		}

		//either programming was successful or no programming was requested
		//in either case we need to jump to app, but in former case we need
		//to update the EE flag	before jump to app and in later case we need
		//to see if EE flag allows jump
		if (appProgrammingStarted == 0)  //programming was not going on
		{
            eeprom_busy_wait();
			if (validAppFlag == 0)
			{
                JumpToApp();
			}
			else
			{
			//scbySleep = 1;
			    if(!appErrorMsgSent)
			    {
    			    appErrorMsgSent = 1;
				    sendInvalidProgMsg();
			    }
		    }
	    }
	    else
	    {
		    //programming was going on
		    // set the flag in EE as program is valid
		    eeprom_busy_wait();
		    eeprom_write_byte((BYTE*)EE_PROGM_DATA_VALID_LOC, 0);
		    eeprom_busy_wait();
            eeprom_write_byte((BYTE*)EE_FW_UPDATE_PENDING_LOC, 0);
            validAppFlag = 0;
			    eeprom_busy_wait();
			    //eeprom_write_byte((BYTE*)(EE_SW_VER_LOC + byI), scabySwVer[byI]);
	    }
			//eeprom_busy_wait();
	    JumpToApp();
	}
		////} //if (jumpToAppFlag == 1)

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

            // Debug/Demo to toggle the LED at 1 Hz
            toggleCount++;
            commTimer++;

            if((toggleCount > 128) && (activeError) && (!appProgrammingStarted))
            {
                TOGGLE_LED_1;
                TOGGLE_LED_2;
                TOGGLE_LED_3;
                TOGGLE_LED_4;
                toggleCount = 0;
            }

            wdt_reset();
            if(deviceInfoReadyFlag)
            {
                sendDeviceInfoData();
            }
            if(pingRequest)
            {
                sendPingResponse();
                pingRequest = 0;
            }
            if(fwUpdateRequest)
            {
                sendReadyForFwUpdate();
                fwUpdateRequest = 0;
            }

        }//every ~0.97 milli sec
    }
	else//will run all the time otherwise when a 1 tick or 2 tick task is not running
	{
		wdt_reset();
        if (programPacketReceived == 1)  //packet arrived
		{
		    programPacketReceived = 0; //reset flag

            programPacketCount++;

            if(shortProgramPacketReceived)
            {
                // receivedFwByteCount should already have been set by CAN driver
                shortProgramPacketReceived = 0;
            }
            else
            {
                receivedFwByteCount = 8;
            }

            if(programPacketCount < 6) // Handle the first 5 header packets
            {
                if(programPacketCount == 1)
                {
                    // First packet so pull out the header info
                    receivedChecksum += (DWORD)abyCANRxBuff[4];
                    receivedChecksum += ((DWORD)abyCANRxBuff[5] << 8);
                    receivedChecksum += ((DWORD)abyCANRxBuff[6] << 16);
                    receivedChecksum += ((DWORD)abyCANRxBuff[7] << 24);

                    // Clear the update pending flag...
                    eeprom_busy_wait();
                    eeprom_write_byte((BYTE*)EE_FW_UPDATE_PENDING_LOC, 0);

                }
                // TODO: Compare FW part # here?

            }
            else
            {
			    //point to index in buffer where we need to fill
			    pbyVal = scbyProgPgBuff;	//point to destination array beginning
			    pbyVal += scwIndex;	   //advance in destination array to current index
			    //stuff 8 bytes in the buffer
			    for (byI = 0; byI < receivedFwByteCount; byI++)
			    {
			        *pbyVal++ = abyCANRxBuff[byI];
				    scwIndex++;
                
                    if((scwIndex >= 128) && (pageWriteCount < 96))
				    {
				        _CLI();//disable interrupts
					    boot_page_erase(scdwAddress);
					    while(boot_rww_busy())
					    {
					        boot_rww_enable();
					    }

			            //prepare for the page write
					    //for (wJ = 0; wJ < SPM_PAGESIZE; wJ += 2)
                        for (wJ = 0; wJ < 128; wJ += 2)
					    {
					        boot_page_fill(wJ, *(WORD*)&scbyProgPgBuff[wJ]);
					        scbyChksm2 = scbyChksm2 + scbyProgPgBuff[wJ] + scbyProgPgBuff[wJ + 1];
					    }

			            //finally write to the page
					    boot_page_write(scdwAddress);
					    scdwAddress = scdwAddress + 128;//SPM_PAGESIZE;
					    while(boot_rww_busy())
					    {
					        boot_rww_enable();
					    }
                        pageWriteCount++;
                        if(pageWriteCount > 10)
                        {
                            TOGGLE_LED_1;
                        }
                        if(pageWriteCount > 40)
                        {
                            TOGGLE_LED_2;
                        }
                        if(pageWriteCount == 96)
                        {
                            TOGGLE_LED_4;
                            fwUpdateCompleteFlag = 1;
                        }
					    _SEI();//enable interrupts
					    scwIndex = 0;
		                pbyVal = scbyProgPgBuff;	//point to destination array beginning
				    }//if (scwIndex >= 128)
			    }//for (byI = 0; byI < 8; byI++)
            }
		    sendProgDataAck(); //send Ack
		}//if (programPacketReceived == 1)

		//if (checksumRequestFlag == 1) //request for checksum has come from display
        if(fwUpdateCompleteFlag)
		{
			
            //checksumRequestFlag = 0; //reset the request
            fwUpdateCompleteFlag = 0;
			//now we will calculate the checksum
			scdwAddress = 0;

 			while (scdwAddress < 12288)
			{
				scbyChksm3 = scbyChksm3 + pgm_read_byte_near((WORD)scdwAddress);
				scdwAddress++;
                wdt_reset();
			}

            if((scbyChksm2 == receivedChecksum) && (scbyChksm3 == receivedChecksum))
            {
                // Success!!
                // set the flag in EE as program is valid
                _CLI();
                sendChecksums();
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
                scwIndex = 0;
                scdwAddress = 0;
                //checksumRequestFlag = 0;
                //scbySleep = 0;
                //scflEeUpdateReq = 0;
                //scbyChksm1 = 0;
                scbyChksm2 = 0;
                scbyChksm3 = 0;
                receivedChecksum = 0;
                //scflVerReq = 0;
                sendChecksums();
                sendInvalidProgMsg();
                activeError = 1;
                // Turn off all LEDs to sync flashing
                PORTB &= ~(_BV(PB1));
                PORTC &= ~(_BV(PC6));
                PORTB &= ~(_BV(PB4));
                PORTB &= ~(_BV(PB3));
            }

			//sendChecksumToDisplay(); //send the checksum to the display
		}
	}
     //}//if (scbySleep == 0)
  } //for(;;)
   return (0);
}





/*******************************************************************************
                                End of File
*******************************************************************************/




