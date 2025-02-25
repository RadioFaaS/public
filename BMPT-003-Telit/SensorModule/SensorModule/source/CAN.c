#include "CAN.h"
#include "NValloc.h"
#include "Config.h"
#include "SensorModule.h"
#include "TaskList.h"
#include "WiFiModule.h"
#include "SPI.h"
#include "ADC.h"
#include "LED.h"
#include "BlockageProcessor.h"

#define DLC_MAX            8
#define CH_DISABLE         0x00
#define CH_RxENA           0x80
#define CH_TxENA           0x40

typedef enum
{
	EE_NO_WRITE,
	EE_WRITE_SERIAL,
	EE_WRITE_HW_VER,
    EE_WRITE_HW_PART_NUM_1,
    EE_WRITE_HW_PART_NUM_2
} EE_WriteState_E;

#define CAN_RETRY_LIMIT 1000
static WORD can_retry;
static st_cmd_t scPeriodicCANTx; // this object is used for blockage/flow data transfer
static st_cmd_t scRandomCANTx;	 // used for other message transfers


//8 byte Tx buffers used for CAN message transmission
static U8 txCANMsgSensorData[8];
static U8 txCANMsgNonBlockageBuff[8];

//Rx buffer for CAN Rx interrupt routine
static U8 scabyCANRxBuff[8];

// EE data write buffer
static U8 scabyCANEeBuff[8];
static EE_WriteState_E sceCANEeWriteState = EE_NO_WRITE;

static BYTE scflEePktRecd;//EE data pkt received flag during EE update

static BYTE activeBlockageFlag;
static BYTE activeBlockageRows;

static BYTE CANDataRequestFlags;

static BYTE CANTrafficDetected;

static BYTE activeBlockageStartTimes[4]; // Stores timer readings of when blockage alerts are first activated
static BYTE * serialNumPtr;

static BYTE canRetryFailCount = 0;
static BYTE pingRequestFlag = 0;


void initCAN(void)
{
  BYTE byI;
  BYTE hwType;

  Can_reset();
  //- Set CAN Bit-timing
  can_init((U16)CAN_BAUDRATE);        // c.f. macro in "can_drv.h"
  //- Set CAN Timer Pre-scaler
  CANTCON = CANBT1;

  scPeriodicCANTx.pt_data = &txCANMsgSensorData[0]; //point to buffer
  scPeriodicCANTx.status = STATUS_CLEARED;
  scPeriodicCANTx.cmd = CMD_TX_DATA;
  scPeriodicCANTx.dlc = 6;

  scRandomCANTx.pt_data = &txCANMsgNonBlockageBuff[0]; //point to buffer
  scRandomCANTx.status = STATUS_CLEARED;
  scRandomCANTx.cmd = CMD_TX_DATA;
  scRandomCANTx.dlc = 8;

  serialNumPtr = getSerialNumPtr() + 4;
  hwType = getHwConfig();

  for (byI = 2; byI <= 5; byI++)
  {
	 /* MOB init for reception */
	 CANPAGE = (byI << 4);                        /* select channel */
	 CANSTMOB  = 0x00;                            /* reset channel status */
	 CANCDMOB = CH_DISABLE;                       /* reset control and dlc register */
     /* Reception Channel : mask */
	 ////CANIDM1 = 0x00;
	 ////CANIDM2 &= ~0xE0;
	 ////CANIDM4 = 0;
#ifdef CAN_FILTER_ENABLED
     if(hwType == HW_TYPE_WIRED) // Filter out sensor data from other modules
     {
         // ID MASK => 0x780, ID TAG => 0x000
         CANIDT4 = 0x00;
         CANIDT2 = 0x00;
         CANIDT1 = 0x00;
         CANIDM4 = 0x00;
         CANIDM2 = 0x00;
         CANIDM1 = 0xF0;
     }
     else
     {
         CANIDT4 = 0x00;
         CANIDT2 = 0x00;
         CANIDT1 = 0x00;
         CANIDM4 = 0x00;
         CANIDM2 = 0x00;
         CANIDM1 = 0x00;
     }
#else
     /* Reception Channel : mask */
     CANIDM1 = 0x00;
     CANIDM2 &= ~0xE0;
     CANIDM4 = 0;

     CANIDT4 &= ~0x04;      /* clear bit rtr in CANIDT4. */
#endif
	 /* Reception Channel configuration */
	 ////CANIDT4 &= ~0x04;                             /* clear bit rtr in CANIDT4. */
	 CANCDMOB |= DLC_MAX;                         /* Reception 8 bytes.*/
	 CANCDMOB |= CH_RxENA;                        /* Reception enabled without buffer.*/


  }
  	 /* interrupt configuration - enable interrupts at reception channel*/
     CANIE2 |= 0x3C; //turn on IEMOB4x bits to enable interrupts on MOB 2, 3, 4, 5
	 CANGIE = ((1<<ENRX) | (1<<ENIT));       /* Can_Rx & IT enable */

	 scflEePktRecd = 0;

     activeBlockageFlag = 0;
     activeBlockageRows = 0x0F;

     CANDataRequestFlags = 0;
     CANTrafficDetected = 0;

}

// Interrupt handler for CAN module
ISR(CAN_INT_vect)
{
   unsigned int id;       /* can_data index */
   BYTE byI;
   BYTE byCtr;

   BYTE pageSave = CANPAGE;

   for (byI = 2; byI <= 5; byI++)
   {
	   CANPAGE = (byI << 4);    /*  select channel 2/3/4/5 to see if it is Rx interrupt */
	   if((CANSTMOB & MOB_RX_COMPLETED) == MOB_RX_COMPLETED)
	   {
	      CANTrafficDetected = 1;

          id = (((int)(CANIDT2)) >> 5) + (((int)(CANIDT1)) << 3);       // V2.0 part A

            //Now take appropriate action based on id and data received
			switch (id)
			{
			   case CANID_SENSOR_DATA: // Ignore sensor data from all modules
                    break;

               case CANID_ACTIVE_BLOCKAGE:
                // Check module ID and handle active blockage here...
		            for (byCtr = 0; byCtr < 5; byCtr++)
		            {
    		            scabyCANRxBuff[byCtr] = CANMSG;
		            }
                    // Check if the blockage is on this module
                    if(compareSerialNums(scabyCANRxBuff, serialNumPtr))
                    {
                        storeActiveBlockageData(scabyCANRxBuff[4]);
                    }
                    break;

               case CANID_EE_DATA:
		           for (byCtr = 0; byCtr < 8; byCtr++)
		           {
				     scabyCANRxBuff[byCtr] = CANMSG;
		           }
			       scflEePktRecd = 1;
			       break;

               case CANID_START_SENSOR_FW_UPDATE:
                    // Check module Serial Num and prep for update here...
                    for (byCtr = 0; byCtr < 4; byCtr++)
                    {
                        scabyCANRxBuff[byCtr] = CANMSG;
                    }

                    if(compareSerialNums(scabyCANRxBuff, serialNumPtr))
                    {
                        _CLI();
                        // jump to bootloader...
						eeprom_busy_wait();
						while(eeprom_read_byte((BYTE *)EE_FW_UPDATE_PENDING_LOC) != 0x01)
						{
							eeprom_busy_wait();
							eeprom_write_byte((BYTE *)EE_FW_UPDATE_PENDING_LOC, 0x01);
							eeprom_busy_wait();
						}						
                        jumpToBootloader();
                    }
                    break;

                case CANID_REQ_SENSOR_VERSIONS:
                    CANDataRequestFlags |= 0x01;
                    break;

                case CANID_TARGETED_DEVICE_INFO_REQ:
                    // Check module Serial Num
                    for (byCtr = 0; byCtr < 4; byCtr++)
                    {
                        scabyCANRxBuff[byCtr] = CANMSG;
                    }
                    if(compareSerialNums(scabyCANRxBuff, serialNumPtr))
                    {
                        CANDataRequestFlags |= 0x01;
                    }
                    break;                    

                case CANID_START_CAL:
                    startSensorCalibration();
                    break;

                case CANID_SENSOR_PING_ALL:
                    pingRequestFlag = 1;
                    break;

                case CANID_WRITE_SERIAL_NUM:
                    for (byCtr = 0; byCtr < 8; byCtr++)
                    {
                        scabyCANEeBuff[byCtr] = CANMSG;
                    }
                    sceCANEeWriteState = EE_WRITE_SERIAL;
                    break;

                case CANID_WRITE_HW_VERSION:
                    for (byCtr = 0; byCtr < 5; byCtr++)
                    {
                        scabyCANEeBuff[byCtr] = CANMSG;
                    }
                    sceCANEeWriteState = EE_WRITE_HW_VER;
                    break;

                case CANID_WRITE_HW_PART_NUM_1:
                    for (byCtr = 0; byCtr < 8; byCtr++)
                    {
                        scabyCANEeBuff[byCtr] = CANMSG;
                    }
                    sceCANEeWriteState = EE_WRITE_HW_PART_NUM_1;
                    break;
                    
                case CANID_WRITE_HW_PART_NUM_2:
                    for (byCtr = 0; byCtr < 5; byCtr++)
                    {
                        scabyCANEeBuff[byCtr] = CANMSG;
                    }
                    sceCANEeWriteState = EE_WRITE_HW_PART_NUM_2;
                    break;
                
                case CANID_DISABLE_SENSOR_DATA:
                    setSensorDataDisableFlag();
                    break;

                case CANID_ENABLE_SENSOR_DATA:
                    clearSensorDataDisableFlag();
                    break;

                case CANID_ENABLE_DEBUG_DATA:
                    for (byCtr = 0; byCtr < 4; byCtr++)
                    {
                        scabyCANRxBuff[byCtr] = CANMSG;
                    }

                    if(compareSerialNums(scabyCANRxBuff, serialNumPtr))
                    {
                        setEnableDebugFlag();
                    }
                    break;

                case CANID_DISABLE_DEBUG_DATA:
                    for (byCtr = 0; byCtr < 4; byCtr++)
                    {
                        scabyCANRxBuff[byCtr] = CANMSG;
                    }

                    if(compareSerialNums(scabyCANRxBuff, serialNumPtr))
                    {
                        clearEnableDebugFlag();
                    }
                break;

               default:
				   break;
		    }
	   }
	   else if (CANSTMOB & MOB_NOT_REACHED) //error in MOB 1
	   {
		  _NOP();
	   }
	   CANSTMOB = 0x00;                             /* reset channel 0 status */
	   CANCDMOB = DLC_MAX;                         /* receive 8 bytes */
	   CANCDMOB |= CH_RxENA;                       /* reception enable */
	   CANGIT = CANGIT;                            /* reset all flags */
   }
   CANPAGE = pageSave;
}


void sendCANSensorDataMsg(BYTE * levels)
{
	can_retry = CAN_RETRY_LIMIT;
	while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED && can_retry > 0)
	{
		can_retry--;
	}
	if(can_retry == 0)
	{
        canRetryFailCount++;
        if(canRetryFailCount > 50)
        {
           canRetryFailCount = 0;
           initCAN();
        }
        return;
	}
    canRetryFailCount = 0;

    // Fill the data bytes
    txCANMsgSensorData[0] = *(serialNumPtr); // Serial Num
    txCANMsgSensorData[1] = *(serialNumPtr + 1); // Serial Num
    txCANMsgSensorData[2] = *(serialNumPtr + 2); // Serial Num
    txCANMsgSensorData[3] = *(serialNumPtr + 3); // Serial Num
    txCANMsgSensorData[4] = ((levels[0] << 4) | (levels[1] & 0x0F)); // Rows 1 & 2
    txCANMsgSensorData[5] = ((levels[2] << 4) | (levels[3] & 0x0F)); // Rows 3 & 4
    scPeriodicCANTx.dlc = 6;
    scPeriodicCANTx.id.std = CANID_SENSOR_DATA;

    if(can_cmd(&scPeriodicCANTx) != CAN_CMD_ACCEPTED)
    {
        // Abort the TX request
        scPeriodicCANTx.cmd = CMD_ABORT;
        can_cmd(&scPeriodicCANTx);
        scPeriodicCANTx.cmd = CMD_TX_DATA;
    }
}

// For debug only...
void sendCANRawADC(WORD * rawPtr)
{
    while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED);
    // Fill the data bytes
    txCANMsgSensorData[0] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[1] = (*(rawPtr++) & 0xFF00) >> 8;
    txCANMsgSensorData[2] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[3] = (*(rawPtr++) & 0xFF00) >> 8;
    txCANMsgSensorData[4] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[5] = (*(rawPtr++) & 0xFF00) >> 8;
    txCANMsgSensorData[6] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[7] = (*(rawPtr++) & 0xFF00) >> 8;
    scPeriodicCANTx.dlc = 8;
    scPeriodicCANTx.id.std = 0x5A;
    while (can_cmd(&scPeriodicCANTx) != CAN_CMD_ACCEPTED);
}

// For debug only...
void sendCANRawADC2(WORD * rawPtr)
{
    while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED);
    // Fill the data bytes
    txCANMsgSensorData[0] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[1] = (*(rawPtr++) & 0xFF00) >> 8;
    txCANMsgSensorData[2] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[3] = (*(rawPtr++) & 0xFF00) >> 8;
    txCANMsgSensorData[4] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[5] = (*(rawPtr++) & 0xFF00) >> 8;
    txCANMsgSensorData[6] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[7] = (*(rawPtr++) & 0xFF00) >> 8;
    scPeriodicCANTx.dlc = 8;
    scPeriodicCANTx.id.std = 0x5B;
    while (can_cmd(&scPeriodicCANTx) != CAN_CMD_ACCEPTED);
}

// For debug only...
void sendCANRawADC3(WORD * rawPtr)
{
    while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED);
    // Fill the data bytes
    txCANMsgSensorData[0] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[1] = (*(rawPtr++) & 0xFF00) >> 8;
    txCANMsgSensorData[2] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[3] = (*(rawPtr++) & 0xFF00) >> 8;
    txCANMsgSensorData[4] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[5] = (*(rawPtr++) & 0xFF00) >> 8;
    txCANMsgSensorData[6] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[7] = (*(rawPtr++) & 0xFF00) >> 8;
    scPeriodicCANTx.dlc = 8;
    scPeriodicCANTx.id.std = 0x5C;
    while (can_cmd(&scPeriodicCANTx) != CAN_CMD_ACCEPTED);
}

// For debug only...
void sendCANRawADC4(WORD * rawPtr)
{
    while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED);
    // Fill the data bytes
    txCANMsgSensorData[0] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[1] = (*(rawPtr++) & 0xFF00) >> 8;
    txCANMsgSensorData[2] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[3] = (*(rawPtr++) & 0xFF00) >> 8;
    txCANMsgSensorData[4] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[5] = (*(rawPtr++) & 0xFF00) >> 8;
    txCANMsgSensorData[6] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[7] = (*(rawPtr++) & 0xFF00) >> 8;
    scPeriodicCANTx.dlc = 8;
    scPeriodicCANTx.id.std = 0x5D;
    while (can_cmd(&scPeriodicCANTx) != CAN_CMD_ACCEPTED);
}

// For debug only...
void sendCANRawADC5(WORD * rawPtr)
{
    while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED);
    // Fill the data bytes
    txCANMsgSensorData[0] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[1] = (*(rawPtr++) & 0xFF00) >> 8;
    txCANMsgSensorData[2] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[3] = (*(rawPtr++) & 0xFF00) >> 8;
    txCANMsgSensorData[4] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[5] = (*(rawPtr++) & 0xFF00) >> 8;
    txCANMsgSensorData[6] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[7] = (*(rawPtr++) & 0xFF00) >> 8;
    scPeriodicCANTx.dlc = 8;
    scPeriodicCANTx.id.std = 0x5E;
    while (can_cmd(&scPeriodicCANTx) != CAN_CMD_ACCEPTED);
}

// For debug only...
void sendCANRawADC6(WORD * rawPtr)
{
    while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED);
    // Fill the data bytes
    txCANMsgSensorData[0] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[1] = (*(rawPtr++) & 0xFF00) >> 8;
    txCANMsgSensorData[2] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[3] = (*(rawPtr++) & 0xFF00) >> 8;
    txCANMsgSensorData[4] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[5] = (*(rawPtr++) & 0xFF00) >> 8;
    txCANMsgSensorData[6] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[7] = (*(rawPtr++) & 0xFF00) >> 8;
    scPeriodicCANTx.dlc = 8;
    scPeriodicCANTx.id.std = 0x5F;
    while (can_cmd(&scPeriodicCANTx) != CAN_CMD_ACCEPTED);
}

// For debug only...
void sendCANRawADC7(WORD * rawPtr)
{
    while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED);
    // Fill the data bytes
    txCANMsgSensorData[0] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[1] = (*(rawPtr++) & 0xFF00) >> 8;
    txCANMsgSensorData[2] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[3] = (*(rawPtr++) & 0xFF00) >> 8;
    txCANMsgSensorData[4] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[5] = (*(rawPtr++) & 0xFF00) >> 8;
    txCANMsgSensorData[6] = *(rawPtr) & 0xFF;
    txCANMsgSensorData[7] = (*(rawPtr++) & 0xFF00) >> 8;
    scPeriodicCANTx.dlc = 8;
    scPeriodicCANTx.id.std = 0x60;
    while (can_cmd(&scPeriodicCANTx) != CAN_CMD_ACCEPTED);
}

void sendCalStatusMsg(BYTE status, BYTE row, BYTE sensorNum)
{
    while (can_get_status(&scRandomCANTx) == CAN_STATUS_NOT_COMPLETED);
    // Fill the data bytes
    txCANMsgNonBlockageBuff[0] = status;
    txCANMsgNonBlockageBuff[1] = row;
    txCANMsgNonBlockageBuff[2] = sensorNum;
    txCANMsgNonBlockageBuff[3] = *(serialNumPtr); // Serial Num;
    txCANMsgNonBlockageBuff[4] = *(serialNumPtr + 1); // Serial Num;
    txCANMsgNonBlockageBuff[5] = *(serialNumPtr + 2); // Serial Num;
    txCANMsgNonBlockageBuff[6] = *(serialNumPtr + 3); // Serial Num;

    scRandomCANTx.dlc = 7;
    scRandomCANTx.id.std = CANID_CAL_STATUS;
    while (can_cmd(&scRandomCANTx) != CAN_CMD_ACCEPTED);
}


void sendDeviceInfoData(BYTE * data)
{
    while (can_get_status(&scRandomCANTx) == CAN_STATUS_NOT_COMPLETED);
    // Fill the data bytes
    txCANMsgNonBlockageBuff[0] = *(data++); 
    txCANMsgNonBlockageBuff[1] = *(data++); 
    txCANMsgNonBlockageBuff[2] = *(data++);
    txCANMsgNonBlockageBuff[3] = *(data++);
    txCANMsgNonBlockageBuff[4] = *(data++);
    txCANMsgNonBlockageBuff[5] = *(data++);
    txCANMsgNonBlockageBuff[6] = *(data++);
    txCANMsgNonBlockageBuff[7] = *data;

    scRandomCANTx.dlc = 8;
    scRandomCANTx.id.std = CANID_DEVICE_INFO;
    while (can_cmd(&scRandomCANTx) != CAN_CMD_ACCEPTED);
}

void sendCANBitResultsMsg(BYTE bitVal)
{

    while (can_get_status(&scRandomCANTx) == CAN_STATUS_NOT_COMPLETED);

    txCANMsgNonBlockageBuff[0] = bitVal; // BIT status indicator

    scRandomCANTx.dlc = 1;
    scRandomCANTx.id.std = CANID_SENSOR_BIT_RESULT;
    while (can_cmd(&scRandomCANTx) != CAN_CMD_ACCEPTED);

}

void sendCANPingResponseMsg(void)
{
    while (can_get_status(&scRandomCANTx) == CAN_STATUS_NOT_COMPLETED);

    txCANMsgNonBlockageBuff[0] = *(serialNumPtr); // Serial Num
    txCANMsgNonBlockageBuff[1] = *(serialNumPtr + 1); // Serial Num
    txCANMsgNonBlockageBuff[2] = *(serialNumPtr + 2); // Serial Num
    txCANMsgNonBlockageBuff[3] = *(serialNumPtr + 3); // Serial Num

    scRandomCANTx.dlc = 4;
    scRandomCANTx.id.std = CANID_PING_RESPONSE;
    while (can_cmd(&scRandomCANTx) != CAN_CMD_ACCEPTED);
}

BYTE getActiveBlockageFlag(void)
{
    return activeBlockageFlag;
}

void setActiveBlockageFlag(void)
{
    activeBlockageFlag = 1;
}

void clearActiveBlockageFlag(void)
{
    activeBlockageFlag = 0;
}

BYTE getActiveBlockageRows(void)
{
    return activeBlockageRows;
}

void setActiveBlockageRows(BYTE val)
{
    activeBlockageRows = val;
}

BYTE * getActiveBlockageStartTimes(void)
{
    return activeBlockageStartTimes;
}

BYTE getCANDataRequestFlags(void)
{
    return CANDataRequestFlags;
}

void setCANDataRequestFlags(BYTE val)
{
    CANDataRequestFlags = val;
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

void storeActiveBlockageData(BYTE blockageMask)
{
    BYTE timerVal = 0;
    static BYTE blockageStateChangeMask = 0;

    if(blockageMask)
    {
        activeBlockageFlag = 1;
    }
    blockageStateChangeMask ^= blockageMask;
    activeBlockageRows = blockageMask; // Assume index 4 is the row bitmask

    timerVal = get4HzTimer();

    if((activeBlockageRows & 0x01) && (blockageStateChangeMask & 0x01))
    {
        activeBlockageStartTimes[0] = timerVal;
    }

    if((activeBlockageRows & 0x02) && (blockageStateChangeMask & 0x02))
    {
        activeBlockageStartTimes[1] = timerVal;
    }

    if((activeBlockageRows & 0x04) && (blockageStateChangeMask & 0x04))
    {
        activeBlockageStartTimes[2] = timerVal;
    }

    if((activeBlockageRows & 0x08) && (blockageStateChangeMask & 0x08))
    {
        activeBlockageStartTimes[3] = timerVal;
    }
}

BYTE getCANTrafficDetected(void)
{
    return CANTrafficDetected;
}

BYTE getPingRequestFlag(void)
{
    return pingRequestFlag;
}

void clearPingRequestFlag(void)
{
    pingRequestFlag = 0;
}


void stepEEUpdateTask(void)
{
	
	switch(sceCANEeWriteState)
	{
		case EE_NO_WRITE:
			break;
			
		case EE_WRITE_SERIAL:
			writeSerialNumToEeprom(&scabyCANEeBuff[0]);
			sceCANEeWriteState = EE_NO_WRITE;	// Reset flag
			break;
			
		case EE_WRITE_HW_VER:	
			writeHwVersionToEeprom(&scabyCANEeBuff[0]);
			sceCANEeWriteState = EE_NO_WRITE; 	// Reset flag
			break;

        case EE_WRITE_HW_PART_NUM_1:
			writeHwPartNumToEeprom(&scabyCANEeBuff[0], 0);
			sceCANEeWriteState = EE_NO_WRITE; 	// Reset flag            
            break;

        case EE_WRITE_HW_PART_NUM_2:
            writeHwPartNumToEeprom(&scabyCANEeBuff[0], 1);
            sceCANEeWriteState = EE_NO_WRITE; 	// Reset flag
            break;
	}	
}


