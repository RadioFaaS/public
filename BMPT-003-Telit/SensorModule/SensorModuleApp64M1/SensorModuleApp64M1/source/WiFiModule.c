

#include "WiFiModule.h"
#include "SensorModule.h"
#include "UART.h"
#include "NValloc.h"
#include <avr/pgmspace.h>
#include <string.h>
#include <util/delay_basic.h>


// Connections
//const char setBulkTransfer[] PROGMEM = "AT+BDATA=1\r\n"; // Enables bulk transfer
const char udpOutConnectCommand[] PROGMEM = "AT+NCUDP=192.168.1.255,41501\r\n";
//const char updInConnectCommand[] PROGMEM = "AT+NCUDP=10.25.35.255,41500\r\n";

const char closeAllConnectionsCommand[] PROGMEM = "AT+NCLOSEALL\r\n";
const char startUdpServerCommand[] PROGMEM = "AT+NSUDP=41500\r\n"; // On 'ASIP In' port?

// TODO: I'm not entirely convinced that this is the right command for sending UDP.
const char udpBulkTransferSeq[] PROGMEM = "\x1BZ";

//const char udpBulkTransferReceived[] PROGMEM = "\x1By\0";

const char udpBulkTransferEnd[] PROGMEM = "192.168.1.255:41500:";//"address:port:length\0";

const char setBaudRateCommand[] PROGMEM = "ATB=115200,8,n,1\r\n"; // Change the baud rate
const char setFlowControlCommand[] PROGMEM = "AT&R1\r\n"; // Enable hardware flow control

static BYTE asipCID = 0;


void sendWifiCommand(PGM_P command)
{
	char wifiConfigTxBuffer[50]; // Tx buffer for WiFi config messages

	// Retrieve the command string from Flash and then add to Tx buffer
	strcpy_P(wifiConfigTxBuffer, command);
	// Send via UART...
	addBytesToWifiTxBuffer((BYTE *)&wifiConfigTxBuffer[0], __strlen_P(command));
}

void readAsipOutCID(void)
{
    eeprom_busy_wait();
    asipCID = 0;
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
    
    sendWifiCommand(udpBulkTransferSeq); // This should also work for TCP connections
    addBytesToWifiTxBuffer(cmdBuffer, 10);
    // Add in payload
    addBytesToWifiTxBuffer(pMessage->PayloadPtr, pMessage->PayloadLength);
    // Add in checksum bytes
    cmdBuffer[0] = pMessage->Checksum1;
    cmdBuffer[1] = pMessage->Checksum2;
    addBytesToWifiTxBuffer(cmdBuffer, 2);
}

void startUdpPacketTransmit(WORD size)
{
	BYTE cmdBuffer[10];
	BYTE remainder;
	//BYTE dataLen = pMessage->PayloadLength + 7; // Payload plus ASIP header/checksum
	
	// ========== WiFi Command ==========
	// Connection ID
	cmdBuffer[0] = asipCID + '0';
	
	// Convert the data length into a 4 digit string
	cmdBuffer[1] = (size >= 1000)?'1':'0';//(size / 1000) + '0'; // Thousands
	cmdBuffer[2] = (size / 100) + '0'; // Hundreds
	remainder = size % 100;
	cmdBuffer[3] = (remainder / 10) + '0'; // Tens
	remainder = remainder % 10;
	cmdBuffer[4] = remainder + '0'; // Ones
	
	sendWifiCommand(udpBulkTransferSeq);
	addBytesToWifiTxBuffer(cmdBuffer, 5);
}






