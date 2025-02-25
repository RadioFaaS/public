

#include "WiFiModule.h"
#include "SensorModuleWirelessBoot.h"
#include <avr/pgmspace.h>
#include <string.h>
//#include <util/delay_basic.h>
#include "UART.h"
#include "ASIP.h"
#include "NValloc.h"

//const char broadcastAddress[] PROGMEM = "192.168.1.255\0";
//const char moduleIP[] PROGMEM = "192.168.1.1\0";
//const char netmask[] PROGMEM = "255.255.255.0\0";

// Device setup
//const char getVersionCommand[] PROGMEM = "AT+VER=?\r\n";
const char setRegulatoryDomainCommand[] PROGMEM = "AT+WREGDOMAIN=0\r\n"; // 0 => FCC
const char setOpModeCommand[] PROGMEM = "AT+WM=2\r\n"; // 2 => Limited AP
const char setSecurityCommand[] PROGMEM = "AT+WSEC=1\r\n"; // 1 => Open
const char setRadioOpEnableCommand[] PROGMEM = "AT+WRXACTIVE=1\r\n"; // 1 => Enable, 0 => Disable
//const char setRadioOpDisableCommand[] PROGMEM = "AT+WRXACTIVE=0\r\n\0"; // 1 => Enable, 0 => Disable
const char setTransmitPowerCommand[] PROGMEM = "AT+WP=0\r\n"; // 0 => Max Power, 7 => Min Power
const char setKeepAliveCommand[] PROGMEM = "AT+PSPOLLINTRL=0\r\n"; // 0 => Disable the timer

const char setATSyncCommand[] PROGMEM = "\r\n\r\nAT\r\n";			// Simple AT command with sync chars
const char setVerboseEnableCommand[] PROGMEM = "ATV1\r\n";	// Verbose mode enable
const char setEchoEnableCommand[] PROGMEM = "ATE1\r\n";		// Echo mode enable
const char setEchoDisableCommand[] PROGMEM = "ATE0\r\n";		// Echo mode disable
//const char setSoftResetCommand[] PROGMEM = "AT+RESET\r\n";		// Soft reset the GS module
const char setSSIDCommand[] PROGMEM = "AT+WA=";					// Set SSID - Note, don't set \r\n yet, SSID to follow
//const char setSSID_HACK_Command[] PROGMEM = "AT+WA=CDSJB-MOD\r\n";
const char startTcpServerCommand[] PROGMEM = "AT+NSTCP=41509\r\n";

// Network setup
const char setDHCPCommand[] PROGMEM = "AT+NDHCP=1\r\n";
const char setNetParamsCommand[] PROGMEM = "AT+NSET=192.168.1.1,255.255.255.0,192.168.1.1\r\n";
const char setDHCPServerCommand[] PROGMEM = "AT+DHCPSRVR=1\r\n";
//const char * associateNetworkCommand = "";

// Connections
const char setBulkTransfer[] PROGMEM = "AT+BDATA=1\r\n"; // Enables bulk transfer
const char udpOutConnectCommand[] PROGMEM = "AT+NCUDP=192.168.1.255,41501\r\n";
//const char updInConnectCommand[] PROGMEM = "AT+NCUDP=10.25.35.255,41500\r\n";

const char closeAllConnectionsCommand[] PROGMEM = "AT+NCLOSEALL\r\n";
const char startUdpServerCommand[] PROGMEM = "AT+NSUDP=41500\r\n"; // On 'ASIP In' port?

//const char commandSuccessful[] PROGMEM = "OK\r\0";
//const char connectResponse[] PROGMEM = "CONNECT \0";
//const char transferSuccessful[] PROGMEM = "\x1BO\0";
////const char udpBulkTransferSeq[] PROGMEM = "\x1BY";
const char udpBulkTransferSeq[] PROGMEM = "\x1BZ";
//const char udpBulkTransferReceived[] PROGMEM = "\x1By\0";

const char udpBulkTransferEnd[] PROGMEM = "192.168.1.255:41500:";//"address:port:length\0";

const char setBaudRateCommand[] PROGMEM = "ATB=115200,8,n,1\r\n"; // Change the baud rate
const char setBaudRateCommand2[] PROGMEM = "ATB=38400,8,n,1\r\n"; // Change the baud rate 38400
const char setFlowControlCommand[] PROGMEM = "AT&R1\r\n"; // Enable hardware flow control

/** New TELIT Module implementation */

/** @brief: It will switch to legacy mode and after that old commands will work from GAINSPAN module 
	AT+YLC=<Mode>  
	Mode: 1 = Enables legacy mode 
	Mode: 0 = Enables new AT command 
*/
const char switchToLegacyMode[] PROGMEM = "AT+YLC=1\r\n"; 


typedef enum
{
	INIT_POWERUP,
	INIT_AT_SYNC,
	INIT_AT_SYNC_1,
	INIT_AT_SYNC_2,
	INIT_AT_SYNC_3_SSID,
	INIT_SWITCH_TO_LEGACY_MODE,
	INIT_VERBOSE,
	INIT_ECHO,
	INIT_FLOW_CNTL,
	INIT_BAUD_RATE,
	INIT_BULK_DATA,
	INIT_REG_DOMAIN,
	INIT_OP_MODE,
	INIT_SECURITY,
	INIT_RADIO_ENABLE,
	INIT_TX_POWER,
	INIT_KEEP_ALIVE,
	INIT_DHCP,
	INIT_NET_PARAMS,
	INIT_DHCP_SERVER,
	INIT_SSID,
	INIT_CLOSE_CONNECTIONS,
    INIT_OPEN_UDP_OUT,
    INIT_OPEN_UDP_IN,
    INIT_OPEN_TCP_PORT,
	INIT_IDLE,
    //INIT_TEST_TX,
	INIT_NEXT_STATE_WAIT_SWITCH_LEGACY,
	INIT_NEXT_STATE_WAIT,
} wifiInitState;



//static BYTE wifiCommReady;
static BYTE wifiDataReady;

static BYTE asipCID = 0;
static BYTE asipServerCID = 0;

static char expectedResponseString[10];

static BYTE expectingWifiResponse;
static char * expectedWifiResponsePtr;

//static BYTE * rxDataBuffPtr;

PGM_P p; // Pointer used to locate const strings stored in PROGMEM

static void sendAssociateNetworkCommand(void);
static void buildSSID(char * ssid);


void stepInitWiFiModule(void)
{
	static wifiInitState InitState = INIT_POWERUP;
	static wifiInitState nextState;
    static WORD tickCount = 0;
	static BOOL fBaudChanged = FALSE;
	
	switch(InitState)
	{
		// set the external reset line high and wait for it to settle
		case INIT_POWERUP:
			ENABLE_UART_SHIFTERS;
			setWifiResetHigh();
			
			tickCount++;
			if(tickCount == 1)
			{
				TURN_ON_ROW4_LED;
			}
			else if(tickCount == 100)
			{
				TURN_OFF_ROW4_LED;
			}
			else if(tickCount > 1000)
			{
				// Power up done, jump to the commands now
				//clearUART_RX_Buffer();
				InitState = INIT_AT_SYNC;
				tickCount = 0;
			}
			break;
				
		// Send out the AT command and some sync bytes
		case INIT_AT_SYNC:
			sendWifiCommand(setATSyncCommand);
			nextState = INIT_AT_SYNC_1;
			InitState = INIT_NEXT_STATE_WAIT;	
			break;
		
		// Send out the AT command and some sync bytes
		case INIT_AT_SYNC_1:
			sendWifiCommand(setATSyncCommand);
			nextState = INIT_AT_SYNC_2;
			InitState = INIT_NEXT_STATE_WAIT;
			break;
		
		// Send out the AT command and some sync bytes
		case INIT_AT_SYNC_2:
			sendWifiCommand(setATSyncCommand);
			nextState = INIT_SWITCH_TO_LEGACY_MODE;
			InitState= INIT_NEXT_STATE_WAIT;
			break;
		
		// Send out the AT command and some sync bytes
		case INIT_SWITCH_TO_LEGACY_MODE:
			sendWifiCommand(switchToLegacyMode);
			nextState = INIT_VERBOSE;
			InitState= INIT_NEXT_STATE_WAIT_SWITCH_LEGACY;
			break;
			

		// Enable echo and verbose mode
		case INIT_VERBOSE: 
			sendWifiCommand(setVerboseEnableCommand);
			nextState = INIT_ECHO;
			InitState = INIT_NEXT_STATE_WAIT;
			break;

		// Disable echo mode
		case INIT_ECHO:
			sendWifiCommand(setEchoDisableCommand);//setEchoEnableCommand);
			nextState = INIT_FLOW_CNTL;
			InitState = INIT_NEXT_STATE_WAIT;
		    break;
		
		// Enable hardware flow control
		case INIT_FLOW_CNTL:
			sendWifiCommand(setFlowControlCommand);
			nextState = INIT_BAUD_RATE;//INIT_BULK_DATA;//INIT_BAUD_RATE;
			InitState = INIT_NEXT_STATE_WAIT;
		    break;

		// Change the baud rate (step it up to 115200)
		case INIT_BAUD_RATE:
			tickCount++;
			if(tickCount == 1)
			{
				////setExpectedWifiResponse(1);
				//sendWifiCommand(setFlowControlCommand);
				//sendWifiCommand(setBaudRateCommand2);
			}
			else if(tickCount > 1000 || getResponseType() == 1)
			{
				tickCount = 0;
				InitState = INIT_BULK_DATA;
			}
			else
			{
				if(getIsTX_Idle() && !fBaudChanged)
				{
					// We have nothing left to transmit, change the baud rate of the MCU
					//setUART_Baud(BAUD_RATE_38400_16MHZ);//BAUD_RATE_115200_16MHZ); BAUD_RATE_38400_16MHZ
					fBaudChanged = TRUE;
				}
			}
			break;
			
		// Enable bulk data transfer
		case INIT_BULK_DATA:
			//setUART_Baud(BAUD_RATE_115200_16MHZ);
            sendWifiCommand(setBulkTransfer);
			nextState = INIT_REG_DOMAIN;
			InitState = INIT_NEXT_STATE_WAIT;
			break;
			
		case INIT_REG_DOMAIN:
			sendWifiCommand(setRegulatoryDomainCommand);
			nextState = INIT_OP_MODE;
			InitState = INIT_NEXT_STATE_WAIT;
			break;
			
		case INIT_OP_MODE:
			sendWifiCommand(setOpModeCommand);
			nextState = INIT_SECURITY;
			InitState = INIT_NEXT_STATE_WAIT;
			break;
			
		case INIT_SECURITY:
			sendWifiCommand(setSecurityCommand);
			nextState = INIT_RADIO_ENABLE;
			InitState = INIT_NEXT_STATE_WAIT;
			break;
			
		case INIT_RADIO_ENABLE:
			sendWifiCommand(setRadioOpEnableCommand);
			nextState = INIT_TX_POWER;
			InitState = INIT_NEXT_STATE_WAIT;
			break;
			
		case INIT_TX_POWER:
			sendWifiCommand(setTransmitPowerCommand);
			nextState = INIT_DHCP; // INIT_KEEP_ALIVE @attention: Keep Alive is skipped as not supported as per Telit Communication
			InitState = INIT_NEXT_STATE_WAIT;
			break;
			
		case INIT_KEEP_ALIVE:
			sendWifiCommand(setKeepAliveCommand);
			nextState = INIT_DHCP;
			InitState = INIT_NEXT_STATE_WAIT;
			break;
			
		case INIT_DHCP:
			sendWifiCommand(setDHCPCommand);
			nextState = INIT_NET_PARAMS;
			InitState = INIT_NEXT_STATE_WAIT;
			break;
			
		case INIT_NET_PARAMS:
			sendWifiCommand(setNetParamsCommand);
			nextState = INIT_DHCP_SERVER;
			InitState = INIT_NEXT_STATE_WAIT;
			break;
			
		case INIT_DHCP_SERVER:
			sendWifiCommand(setDHCPServerCommand);
			nextState = INIT_SSID;
			InitState = INIT_NEXT_STATE_WAIT;
			break;
			
		case INIT_SSID:
			tickCount++;
			if(tickCount == 1)
			{
				//setExpectedWifiResponse(WIFI_RESPONSE_OK);
                ////sendWifiCommand(setSSID_HACK_Command);
				sendAssociateNetworkCommand();
			}
			else if((tickCount > 1000) || (getResponseType() == WIFI_RESPONSE_OK))
			{
				tickCount = 0;
				InitState = INIT_CLOSE_CONNECTIONS;
			}
			break;
			
		case INIT_CLOSE_CONNECTIONS:
			sendWifiCommand(closeAllConnectionsCommand);
			nextState = INIT_OPEN_UDP_OUT;
			InitState = INIT_NEXT_STATE_WAIT;
			break;

        case INIT_OPEN_UDP_OUT:
            tickCount++;
            if(tickCount == 1)
            {
                ////setExpectedWifiResponse(WIFI_RESPONSE_CONNECT_OUT);
                sendWifiCommand(udpOutConnectCommand);
            }
            else if((tickCount > 250) || (getResponseType() == WIFI_RESPONSE_CONNECT_OUT))
            {
                tickCount = 0;

                eeprom_busy_wait();
                eeprom_write_byte((BYTE *)EE_WIFI_OUT_CID_LOC, asipCID);

                InitState = INIT_OPEN_UDP_IN;
            }
            break;

        case INIT_OPEN_UDP_IN:
			tickCount++;
			if(tickCount == 1)
			{
    			////setExpectedWifiResponse(WIFI_RESPONSE_CONNECT_IN);
                sendWifiCommand(startUdpServerCommand);
			}
			else if((tickCount > 250) || (getResponseType() == WIFI_RESPONSE_CONNECT_IN))
			{
    			tickCount = 0;

                eeprom_busy_wait();
                eeprom_write_byte((BYTE *)EE_WIFI_IN_CID_LOC, asipServerCID);

    			InitState = INIT_IDLE;//INIT_OPEN_TCP_PORT;//INIT_IDLE;
			}
            break;

		case INIT_IDLE:
            ////setExpectedWifiResponse(WIFI_RESPONSE_NONE);
			TURN_ON_ROW4_LED;
            // Write the flag indicating that the Wifi module is properly configured
            eeprom_busy_wait();
            eeprom_write_byte((BYTE *)EE_WIFI_CONFIG_STATE_LOC, 0x00);

            setWifiDoneFlag();

			break;
			
		case INIT_NEXT_STATE_WAIT:
			tickCount++;
			if((tickCount > 250) || (getResponseType() == WIFI_RESPONSE_OK))
			{
				tickCount = 0;
				InitState = nextState;
			}
			break;
		
		case INIT_NEXT_STATE_WAIT_SWITCH_LEGACY:
			tickCount++;
			if((tickCount > 1000) || (getResponseType() == WIFI_RESPONSE_OK))
			{
				tickCount = 0;
				InitState = nextState;
			}
			break;
			

        default:
            break;
	}
	
}

void setWifiResetHigh(void)
{
    //PORTD |= _BV(PD1);
    PORTD &= ~(_BV(PD1)); // logic inverted by hardware
}

// Set up the AP SSID and transmit channel...
// Assumes default WiFi channel 6
static void sendAssociateNetworkCommand(void)
{
    char buff[20];
    
    buildSSID(buff);
	
	/* Old code now \r\n moved inside the build SSID */
    //strcpy(&buff[10], "\r\n");
    
	sendWifiCommand(setSSIDCommand);
    addBytesToWifiTxBuffer((BYTE *)&buff[0], 12);  //Total 12 bytes of data from build SSID function 
}

void sendWifiCommand(PGM_P command)
{
    char wifiConfigTxBuffer[50]; // Tx buffer for WiFi config messages

    // Retrieve the command string from Flash and then add to Tx buffer
	strcpy_P(wifiConfigTxBuffer, command);
    //TURN_ON_ROW2_LED;s
    // Send via UART...
    addBytesToWifiTxBuffer((BYTE *)&wifiConfigTxBuffer[0], __strlen_P(command));
}


// Build the WiFi SSID from the serial number and product name
static void buildSSID(char * ssid)
{
    //BYTE * serialPtr;
    
    //serialPtr = getSerialNumPtr();
    //serialPtr += 4;
    // {'L','B','W','-','L','M','1','0','\0','\0'};
    ssid[0] = 'C';
    ssid[1] = 'D';
    ssid[2] = 'S';
    ssid[3] = 'J';
    ssid[4] = 'B';
    ssid[5] = '_';
    ssid[6] = eeprom_read_byte((BYTE*)(EE_SERIAL_NUM_LOC + 4));//*serialPtr++;
    ssid[7] = eeprom_read_byte((BYTE*)(EE_SERIAL_NUM_LOC + 5));//*serialPtr++;
    ssid[8] = eeprom_read_byte((BYTE*)(EE_SERIAL_NUM_LOC + 6));//*serialPtr++;
    ssid[9] = eeprom_read_byte((BYTE*)(EE_SERIAL_NUM_LOC + 7));//*serialPtr++;
    ssid[10] = '\r';
	ssid[11] = '\n';
}



BYTE getWifiReadyFlag(void)
{
    return wifiDataReady;
}

void setAsipCID(BYTE connection, BYTE val)
{
    if(connection == CID_TYPE_ASIP_OUT)
    {
        asipCID = val;
    }
    else if(connection == CID_TYPE_ASIP_IN)
    {
        asipServerCID = val;
    }
}

BYTE getAsipServerCID(void)
{
    return asipServerCID;
}

void setupFwUpdateTcpPort(void)
{
    sendWifiCommand(startTcpServerCommand);
}


void sendUdpPacket(AsipMsgStruct * pMessage)
{
    BYTE cmdBuffer[10];
    BYTE remainder;
    BYTE dataLen = pMessage->PayloadLength + 7; // Payload plus ASIP header/checksum
    
    // ========== WiFi Command ==========
    // Connection ID
    cmdBuffer[0] = asipCID + '0';
    
    // Convert the data length into a 4 digit string
    cmdBuffer[1] = '0'; // Thousands
    cmdBuffer[2] = (dataLen / 100) + '0'; // Hundreds
    remainder = dataLen % 100;
    cmdBuffer[3] = (remainder / 10) + '0'; // Tens
    remainder = remainder % 10;
    cmdBuffer[4] = remainder + '0'; // Ones
    
    // ========== ASIP Message ==========
    // Start of ASIP Frame
    cmdBuffer[5] = 0xC2;
    cmdBuffer[6] = 0x53;
    cmdBuffer[7] = pMessage->MsgClass;
    cmdBuffer[8] = pMessage->MsgId;
    cmdBuffer[9] = pMessage->PayloadLength;
    
    sendWifiCommand(udpBulkTransferSeq);
    addBytesToWifiTxBuffer(cmdBuffer, 10);
    // Add in payload
    addBytesToWifiTxBuffer(pMessage->PayloadPtr, pMessage->PayloadLength);
    // Add in checksum bytes
    cmdBuffer[0] = pMessage->Checksum1;
    cmdBuffer[1] = pMessage->Checksum2;
    addBytesToWifiTxBuffer(cmdBuffer, 2);
}


void sendTcpPacket(AsipMsgStruct * pMessage)
{
    BYTE cmdBuffer[10];
    BYTE remainder;
    BYTE dataLen = pMessage->PayloadLength + 7; // Payload plus ASIP header/checksum
    
    // ========== WiFi Command ==========
    // Connection ID
    cmdBuffer[0] = '2';//asipCID + '0';
    
    // Convert the data length into a 4 digit string
    cmdBuffer[1] = '0'; // Thousands
    cmdBuffer[2] = (dataLen / 100) + '0'; // Hundreds
    remainder = dataLen % 100;
    cmdBuffer[3] = (remainder / 10) + '0'; // Tens
    remainder = remainder % 10;
    cmdBuffer[4] = remainder + '0'; // Ones
    
    // ========== ASIP Message ==========
    // Start of ASIP Frame
    cmdBuffer[5] = 0xC2;
    cmdBuffer[6] = 0x53;
    cmdBuffer[7] = pMessage->MsgClass;
    cmdBuffer[8] = pMessage->MsgId;
    cmdBuffer[9] = pMessage->PayloadLength;
    
    sendWifiCommand(udpBulkTransferSeq);
    addBytesToWifiTxBuffer(cmdBuffer, 10);
    // Add in payload
    addBytesToWifiTxBuffer(pMessage->PayloadPtr, pMessage->PayloadLength);
    // Add in checksum bytes
    cmdBuffer[0] = pMessage->Checksum1;
    cmdBuffer[1] = pMessage->Checksum2;
    addBytesToWifiTxBuffer(cmdBuffer, 2);
}


