#include "CAN.h"
#include "NValloc.h"
#include "Config.h"
#include "SensorModule.h"
#include "TaskList.h"
#include "WiFiModule.h"
#include "BlockageProcessor.h"
#include "SPI.h"
#include "ADC.h"
#include "LED.h"
#include "BIT.h"

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

#define BLOCKAGE_DATA_BUFF_SIZE     256

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

static BYTE blockageDataBuffer[BLOCKAGE_DATA_BUFF_SIZE];
static BYTE blockageDataBufferHead;
static BYTE blockageDataBufferTail;

static BYTE canRetryFailCount = 0;

static BYTE connectedSensorSerialNums[92]; // 23 sensors * 4 bytes
static BYTE connectedSensorCount = 0;
static BYTE connectedSensorIndex = 0;
static BYTE secondUdpHeaderSent = 0;

static BYTE calStatusResponseCount = 0;
static BYTE calStatusBufferIndex = 1; // 1st byte reserved for results count
static BYTE* calStatusBufferPtr;

static BYTE popBlockageDataBuffer(void);
static void storeConnectedSensor(BYTE * serialNumPtr);
#ifdef _ENABLE_WIFI_DEBUG_DATA
// Raw ADC values
static BYTE scabyRawADC[64];
static WORD scRawADCCount = 0;
static BYTE scfRawPacketPieces = 0x00;
static void copyRawCANMessage(void* buf, BYTE n);
#endif

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
    CANIDT4 = 0x00;
    CANIDT2 = 0x00;
    CANIDT1 = 0x00;
    CANIDM4 = 0x00;
    CANIDM2 = 0x00;
    CANIDM1 = 0x00;
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

     blockageDataBufferHead = 0;
     blockageDataBufferTail = 0;

     calStatusBufferPtr = getFwDataPtr(); // Re-use the static FW update buffer for cal data
}

void fillBlockageDataBuffer(BYTE * data, BYTE size)
{
    BYTE i;
    
    for(i = 0; i < size; i++)
    {
        blockageDataBuffer[blockageDataBufferTail++] = *(data + i);
        
        if(blockageDataBufferTail >= BLOCKAGE_DATA_BUFF_SIZE)
        {
            blockageDataBufferTail = 0;
        }
    }
}

void retransmitBlockageData(void)
{
    //cli();
    if(blockageDataBufferTail != blockageDataBufferHead)
    {
        if(compareSerialNums(&blockageDataBuffer[blockageDataBufferHead], serialNumPtr)) // SN matches this module
        {
			cli(); // Prevent head/tail from being disrupted during pop operation
            (void) popBlockageDataBuffer();
			(void) popBlockageDataBuffer();
			(void) popBlockageDataBuffer();
			(void) popBlockageDataBuffer();
			storeActiveBlockageData(popBlockageDataBuffer());
            sei();
        }
        else
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
            cli();
            txCANMsgSensorData[0] = popBlockageDataBuffer(); // Serial Num 
            txCANMsgSensorData[1] = popBlockageDataBuffer(); // Serial Num
            txCANMsgSensorData[2] = popBlockageDataBuffer(); // Serial Num 
            txCANMsgSensorData[3] = popBlockageDataBuffer(); // Serial Num 
            txCANMsgSensorData[4] = popBlockageDataBuffer();// Blockage data
            sei();
            scPeriodicCANTx.dlc = 5;
            scPeriodicCANTx.id.std = CANID_ACTIVE_BLOCKAGE;

            if(can_cmd(&scPeriodicCANTx) != CAN_CMD_ACCEPTED)
            {
                // Abort the TX request
                scPeriodicCANTx.cmd = CMD_ABORT;
                can_cmd(&scPeriodicCANTx);
                scPeriodicCANTx.cmd = CMD_TX_DATA;
            }
        }
    }
    //sei();
}

static BYTE popBlockageDataBuffer(void)
{
	BYTE ret = 0x00;
	
	if(blockageDataBufferTail != blockageDataBufferHead)
	{
		ret = blockageDataBuffer[blockageDataBufferHead++];
		if(blockageDataBufferHead >= BLOCKAGE_DATA_BUFF_SIZE)
		{
			blockageDataBufferHead = 0;
		}
	}
	
	return ret;
}

// Interrupt handler for CAN module
ISR(CAN_INT_vect)
{
   unsigned int id;       /* can_data index */
   BYTE byI;
   BYTE byCtr;
   BYTE flags;

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
			   case CANID_SENSOR_DATA:
                    // Collect sensor data and buffer for WiFi Tx
		            for (byCtr = 0; byCtr < 6; byCtr++)
		            {
    		            scabyCANRxBuff[byCtr] = CANMSG;
		            }                       
					addFlowRateMessage(scabyCANRxBuff);
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
                    
               case CANID_DISABLE_WIFI:
                    // Shut off wifi module
                    SET_WIFI_RESET_LOW;
                    break;

                case CANID_ENABLE_WIFI:
                    // Turn on wifi module      
                    SET_WIFI_RESET_HIGH;                 
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

                case CANID_START_CAL:
                    startSensorCalibration();
                    break;

                case CANID_CAL_STATUS:
                    // Store all status data to retransmit to display 
                    for (byCtr = 0; byCtr < 7; byCtr++)
                    {
                        scabyCANRxBuff[byCtr] = CANMSG;
                    }
                    copyCalStatusDataToTxBuffer(scabyCANRxBuff);
                    calStatusResponseCount++;
                    break;

                case CANID_WRITE_SERIAL_NUM:
                    for (byCtr = 0; byCtr < 8; byCtr++)
                    {
	                    scabyCANEeBuff[byCtr] = CANMSG;
                    }
                    sceCANEeWriteState = EE_WRITE_SERIAL;
                    break;

                case CANID_DEVICE_INFO:
                    for (byCtr = 0; byCtr < 8; byCtr++)
                    {
                        scabyCANRxBuff[byCtr] = CANMSG;
                    }                    
                    storeDeviceInfoData(&scabyCANRxBuff[0]);
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
                 
                 case CANID_PROG_DATA_ACK:
                    // Re-transmit to the display
                    CANDataRequestFlags |= 0x02;
                    TOGGLE_ROW3_LED;
                    break;

                 case CANID_DISABLE_SENSOR_DATA:
                    setSensorDataDisableFlag();
                    break;

                 case CANID_ENABLE_SENSOR_DATA:
                    clearSensorDataDisableFlag();
                    break;   

                 case CANID_READY_FOR_FW_UPDATE:
                    // Set flag to Tx ack to display
                    flags = getCANDataRequestFlags();
                    setCANDataRequestFlags(flags | START_FW_UPDATE_RESP_BITMASK);
                    break;

                 case CANID_PING_RESPONSE:
                    for (byCtr = 0; byCtr < 4; byCtr++)
                    {
                        scabyCANRxBuff[byCtr] = CANMSG;
                    }                    
                    storeConnectedSensor(&scabyCANRxBuff[0]);
                    break;

                 case CANID_SENSOR_BIT_RESULT:
                    scabyCANRxBuff[0] = CANMSG;
                    flags = getCANDataRequestFlags();					
                    setCANDataRequestFlags(flags | BIT_DATA_RX_BITMASK);
                    setLastActiveBIT(scabyCANRxBuff[0]);
                    break;

                 case CANID_SENSOR_INVALID_PROG:
                    flags = getCANDataRequestFlags();				 
                    setCANDataRequestFlags(flags | BIT_DATA_RX_BITMASK);
                    setLastActiveBIT(2); // BIT id '2'
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

#ifdef _ENABLE_WIFI_DEBUG_DATA					
				 case CANID_RAW_ADC1:
					copyRawCANMessage(scabyRawADC + 1, 8);
					scfRawPacketPieces |= (1 << 0);
					break;
				 case CANID_RAW_ADC2:
					copyRawCANMessage(scabyRawADC + 9, 8);
					scfRawPacketPieces |= (1 << 1);
					break;				 
				 case CANID_RAW_ADC3:
					copyRawCANMessage(scabyRawADC + 17, 8);
					scfRawPacketPieces |= (1 << 2);
					break;				 
				 case CANID_RAW_ADC4:
					copyRawCANMessage(scabyRawADC + 25, 8);
					scfRawPacketPieces |= (1 << 3);
					break;			
				 case CANID_RAW_ADC5:
					copyRawCANMessage(scabyRawADC + 33, 8);
					scfRawPacketPieces |= (1 << 4);
					break;				 
				 case CANID_RAW_ADC6:
					copyRawCANMessage(scabyRawADC + 41, 8);
					scfRawPacketPieces |= (1 << 5);
					break;				 
				 case CANID_RAW_ADC7:
				    copyRawCANMessage(scabyRawADC + 49, 8);
				    scfRawPacketPieces |= (1 << 6);
					break;
#endif				 
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

#ifdef _ENABLE_WIFI_DEBUG_DATA
static void copyRawCANMessage(void* buf, BYTE n)
{
	while(n > 0)
	{
		*((BYTE*)(buf++)) = CANMSG;
		n--;		
	}	
}

void sendUdpRawADC(void)
{
	if(scfRawPacketPieces == 0x7F)
	{
		scabyRawADC[0] = scRawADCCount++;
		buildAsipTransmitMessage(0x00,0x00, 57, scabyRawADC);
		transmitAsipMessage();
		scfRawPacketPieces = 0x00;
	}	
}
#endif

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
    txCANMsgSensorData[4] = ((levels[0] << 4) | (levels[1] & 0x0F));//*levels++; // Rows 1 & 2
    txCANMsgSensorData[5] = ((levels[2] << 4) | (levels[3] & 0x0F));//*levels++; // Rows 3 & 4
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
	can_retry = CAN_RETRY_LIMIT;
	while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED && can_retry > 0)
	{
		can_retry--;
	}
	if(can_retry == 0)
	{
		return;
	}
	
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

	can_cmd(&scPeriodicCANTx);
}

// For debug only...
void sendCANRawADC2(WORD * rawPtr)
{
    ////while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED);
	can_retry = CAN_RETRY_LIMIT;
	while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED && can_retry > 0)
	{
		can_retry--;
	}
	if(can_retry == 0)
	{
		return;
	}
	
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
    ////while (can_cmd(&scPeriodicCANTx) != CAN_CMD_ACCEPTED);
	can_cmd(&scPeriodicCANTx);
}

// For debug only...
void sendCANRawADC3(WORD * rawPtr)
{
    ////while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED);
	can_retry = CAN_RETRY_LIMIT;
	while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED && can_retry > 0)
	{
		can_retry--;
	}
	if(can_retry == 0)
	{
		return;
	}
	
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
    ////while (can_cmd(&scPeriodicCANTx) != CAN_CMD_ACCEPTED);
	can_cmd(&scPeriodicCANTx);
}

// For debug only...
void sendCANRawADC4(WORD * rawPtr)
{
    ////while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED);
	can_retry = CAN_RETRY_LIMIT;
	while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED && can_retry > 0)
	{
		can_retry--;
	}
	if(can_retry == 0)
	{
		return;
	}
	
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
    ////while (can_cmd(&scPeriodicCANTx) != CAN_CMD_ACCEPTED);
	can_cmd(&scPeriodicCANTx);
}

// For debug only...
void sendCANRawADC5(WORD * rawPtr)
{
    ////while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED);
	can_retry = CAN_RETRY_LIMIT;
	while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED && can_retry > 0)
	{
		can_retry--;
	}
	if(can_retry == 0)
	{
		return;
	}
	
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
    ////while (can_cmd(&scPeriodicCANTx) != CAN_CMD_ACCEPTED);
	can_cmd(&scPeriodicCANTx);
}

// For debug only...
void sendCANRawADC6(WORD * rawPtr)
{
    ////while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED);
	can_retry = CAN_RETRY_LIMIT;
	while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED && can_retry > 0)
	{
		can_retry--;
	}
	if(can_retry == 0)
	{
		return;
	}
	
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
    ////while (can_cmd(&scPeriodicCANTx) != CAN_CMD_ACCEPTED);
	can_cmd(&scPeriodicCANTx);
}

// For debug only...
void sendCANRawADC7(WORD * rawPtr)
{
    ////while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED);
	can_retry = CAN_RETRY_LIMIT;
	while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED && can_retry > 0)
	{
		can_retry--;
	}
	if(can_retry == 0)
	{
		return;
	}
	
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
    ////while (can_cmd(&scPeriodicCANTx) != CAN_CMD_ACCEPTED);
	can_cmd(&scPeriodicCANTx);
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

void sendPingAllSensors(void)
{    
    while (can_get_status(&scRandomCANTx) == CAN_STATUS_NOT_COMPLETED);

    scRandomCANTx.dlc = 0;
    scRandomCANTx.id.std = CANID_SENSOR_PING_ALL;
    while (can_cmd(&scRandomCANTx) != CAN_CMD_ACCEPTED);
}

void sendStartCalCommand(void)
{    
    while (can_get_status(&scRandomCANTx) == CAN_STATUS_NOT_COMPLETED);

    scRandomCANTx.dlc = 0;
    scRandomCANTx.id.std = CANID_START_CAL;
    while (can_cmd(&scRandomCANTx) != CAN_CMD_ACCEPTED);
}

void sendCANActiveBlockageMsg(BYTE * serialNumPtr, BYTE rowMask)
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
    txCANMsgNonBlockageBuff[0] = *(serialNumPtr++); // Serial Num 
    txCANMsgNonBlockageBuff[1] = *(serialNumPtr++); // Serial Num
    txCANMsgNonBlockageBuff[2] = *(serialNumPtr++); // Serial Num 
    txCANMsgNonBlockageBuff[3] = *serialNumPtr; // Serial Num 
    txCANMsgNonBlockageBuff[4] = rowMask;

    scRandomCANTx.dlc = 5;
    scRandomCANTx.id.std = CANID_ACTIVE_BLOCKAGE;

    if(can_cmd(&scPeriodicCANTx) != CAN_CMD_ACCEPTED)
    {
        // Abort the TX request
        scPeriodicCANTx.cmd = CMD_ABORT;
        can_cmd(&scPeriodicCANTx);
        scPeriodicCANTx.cmd = CMD_TX_DATA;
    }
}

void sendCANStartFwUpdateMsg(BYTE * serialNumPtr)
{
    while (can_get_status(&scRandomCANTx) == CAN_STATUS_NOT_COMPLETED);

    // Fill the data bytes
    txCANMsgNonBlockageBuff[0] = *(serialNumPtr++); // Serial Num
    txCANMsgNonBlockageBuff[1] = *(serialNumPtr++); // Serial Num
    txCANMsgNonBlockageBuff[2] = *(serialNumPtr++); // Serial Num
    txCANMsgNonBlockageBuff[3] = *serialNumPtr; // Serial Num

    scRandomCANTx.dlc = 4;
    scRandomCANTx.id.std = CANID_START_SENSOR_FW_UPDATE;
    while (can_cmd(&scRandomCANTx) != CAN_CMD_ACCEPTED);
}

void sendCANRequestVersionsMsg(void)
{
	can_retry = CAN_RETRY_LIMIT;
	while (can_get_status(&scPeriodicCANTx) == CAN_STATUS_NOT_COMPLETED && can_retry > 0)
	{
		can_retry--;
	}
	if(can_retry == 0)
	{
		return;
	}

    scRandomCANTx.dlc = 0;
    scRandomCANTx.id.std = CANID_REQ_SENSOR_VERSIONS;

	can_cmd(&scPeriodicCANTx);
}

void sendCANGetDeviceInfoBySerial(BYTE * serialNumPtr)
{
    while (can_get_status(&scRandomCANTx) == CAN_STATUS_NOT_COMPLETED);
    // Fill the data bytes
    txCANMsgNonBlockageBuff[0] = *(serialNumPtr++);
    txCANMsgNonBlockageBuff[1] = *(serialNumPtr++);
    txCANMsgNonBlockageBuff[2] = *(serialNumPtr++);
    txCANMsgNonBlockageBuff[3] = *(serialNumPtr);

    scRandomCANTx.dlc = 4;
    scRandomCANTx.id.std = CANID_TARGETED_DEVICE_INFO_REQ;
    while (can_cmd(&scRandomCANTx) != CAN_CMD_ACCEPTED);
}

void sendCANBitResultsMsg(BYTE bitVal)
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
	
    txCANMsgNonBlockageBuff[0] = bitVal; // BIT status indicator

    scRandomCANTx.dlc = 1;
    scRandomCANTx.id.std = CANID_SENSOR_BIT_RESULT;

    if(can_cmd(&scPeriodicCANTx) != CAN_CMD_ACCEPTED)
    {
        // Abort the TX request
        scPeriodicCANTx.cmd = CMD_ABORT;
        can_cmd(&scPeriodicCANTx);
        scPeriodicCANTx.cmd = CMD_TX_DATA;
    }
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

void sendCANFwUpdateDataPacket(BYTE * dataPtr)
{
    while (can_get_status(&scRandomCANTx) == CAN_STATUS_NOT_COMPLETED);

    // Fill the data bytes
    txCANMsgNonBlockageBuff[0] = *(dataPtr++); 
    txCANMsgNonBlockageBuff[1] = *(dataPtr++);
    txCANMsgNonBlockageBuff[2] = *(dataPtr++); 
    txCANMsgNonBlockageBuff[3] = *(dataPtr++); 
    txCANMsgNonBlockageBuff[4] = *(dataPtr++);
    txCANMsgNonBlockageBuff[5] = *(dataPtr++); 
    txCANMsgNonBlockageBuff[6] = *(dataPtr++); 
    txCANMsgNonBlockageBuff[7] = *dataPtr; 

    scRandomCANTx.dlc = 8;
    scRandomCANTx.id.std = CANID_SENSOR_PROG_DATA_CMD;
    while (can_cmd(&scRandomCANTx) != CAN_CMD_ACCEPTED);
}

void sendCANFwUpdateShortDataPacket(BYTE * dataPtr, BYTE count)
{
    BYTE i;
    
    while (can_get_status(&scRandomCANTx) == CAN_STATUS_NOT_COMPLETED);

    // Fill the data bytes
    for(i = 0; i < count; i++)
    {
        txCANMsgNonBlockageBuff[i] = *(dataPtr + i);
    }

    scRandomCANTx.dlc = count;
    scRandomCANTx.id.std = CANID_SENSOR_PROG_DATA_CMD;
    while (can_cmd(&scRandomCANTx) != CAN_CMD_ACCEPTED);
}

void sendCANSensorDataDisableMsg(void)
{
    while (can_get_status(&scRandomCANTx) == CAN_STATUS_NOT_COMPLETED);

    // Fill the data bytes
    scRandomCANTx.dlc = 0;
    scRandomCANTx.id.std = CANID_DISABLE_SENSOR_DATA;
    while (can_cmd(&scRandomCANTx) != CAN_CMD_ACCEPTED);
}

void sendCANSensorDataEnableMsg(void)
{
    while (can_get_status(&scRandomCANTx) == CAN_STATUS_NOT_COMPLETED);

    // Fill the data bytes
    scRandomCANTx.dlc = 0;
    scRandomCANTx.id.std = CANID_ENABLE_SENSOR_DATA;
    while (can_cmd(&scRandomCANTx) != CAN_CMD_ACCEPTED);
}

void sendCANFwUpdateComplete(void)
{
    while (can_get_status(&scRandomCANTx) == CAN_STATUS_NOT_COMPLETED);

    scRandomCANTx.dlc = 0;
    scRandomCANTx.id.std = CANID_FW_UPDATE_COMPLETE;
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

// Returns flag that indicates whether any CAN
//  packets have been received since power-up.
BYTE getCANTrafficDetected(void)
{
    return CANTrafficDetected;
}

// Tracks the serial numbers of all connected wired sensors
//  as reported after a Ping All request
static void storeConnectedSensor(BYTE * serialNumPtr)
{
    BYTE i;
    BYTE matchFound = 0;
    BYTE nextIndex;

    for(i = 0; i < connectedSensorCount; i++)
    {
        if(compareSerialNums(serialNumPtr, &connectedSensorSerialNums[i * 4]))
        {
            matchFound = 1;
            break;
        }
    }
    if(!matchFound)
    {
        nextIndex = connectedSensorCount * 4;
        connectedSensorSerialNums[nextIndex] = *serialNumPtr;
        connectedSensorSerialNums[nextIndex + 1] = *(serialNumPtr + 1);
        connectedSensorSerialNums[nextIndex + 2] = *(serialNumPtr + 2);
        connectedSensorSerialNums[nextIndex + 3] = *(serialNumPtr + 3);

        connectedSensorCount++;
        //connectedSensorIndex
    }
}

// Clears counters associated with tracking
//  CAN bus-connected sensors.
void resetConnectedSensors(void)
{
    connectedSensorCount = 0;
    connectedSensorIndex = 0;
    secondUdpHeaderSent = 0;
}

// Sends a device info request to the next detected wired sensor
BYTE sendNextTargetedDeviceInfoReq(void)
{
    BYTE txCount = 0;
	
	if(connectedSensorIndex < connectedSensorCount)
    {
        // Note that wireless sensor's info is sent separately and is not included 
		//   in the connectedSensorIndex or connectedSensorCount

        for(txCount = 0; txCount < 2; txCount++)
		{
			sendCANGetDeviceInfoBySerial(&connectedSensorSerialNums[connectedSensorIndex * 4]);
		}        
		connectedSensorIndex++;
    }

    if(connectedSensorIndex >= connectedSensorCount)
    {
		return 1;
    }
    else
    {
        return 0;
    }
}

void resendLastTargetedDeviceInfoReq(void)
{
	BYTE txCount = 0;
	
	if(connectedSensorIndex > 0)
	{
        for(txCount = 0; txCount < 2; txCount++)
        {
	        sendCANGetDeviceInfoBySerial(&connectedSensorSerialNums[(connectedSensorIndex - 1) * 4]);
        }		
	}
}

// Returns the number of CAN bus-connected
//  sensor modules as determined by a previous
//  ping request.
BYTE getConnectedSensorCount(void)
{
    return connectedSensorCount;
}

// Returns the number of calibration results
//  data packets received from the CAN bus. 
BYTE getCalStatusResponseCount(void)
{
    return calStatusResponseCount;
}

// Resets the calibration results counter and
//  corresponding results buffer index.
void clearCalStatusResponseCount(void)
{
    calStatusResponseCount = 0;
    calStatusBufferIndex = 1; // Reserve the 1st byte for sensor count
}

// Copies received calibration results data into the
//  predefined WiFi transmit buffer and updates the
//  buffer index values.
void copyCalStatusDataToTxBuffer(BYTE* newDataPtr)
{
    BYTE i;
    BYTE* txBufferPtr;

    txBufferPtr = (calStatusBufferPtr + calStatusBufferIndex);

    for(i = 0; i < 7; i++)
    {
        *(txBufferPtr + i) = *(newDataPtr + i);
    }

    calStatusBufferIndex += 7;
}

void sendSensorCount(BYTE count)
{
	while (can_get_status(&scRandomCANTx) == CAN_STATUS_NOT_COMPLETED);

	// Fill the data bytes
	txCANMsgNonBlockageBuff[0] = count;

	scRandomCANTx.dlc = 1;
	scRandomCANTx.id.std = 0x50;
	while (can_cmd(&scRandomCANTx) != CAN_CMD_ACCEPTED);
}

// Checks if a CAN request has been received
//  to write EEPROM data and executes the
//  write based on the state/data type.
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




