

#include "WiFiModule.h"
#include "SensorModule.h"
#include <avr/pgmspace.h>
#include <string.h>
#include "SPI.h"
#include <util/delay_basic.h>



const char broadcastAddress[] PROGMEM = "10.25.35.255\0";
const char moduleIP[] PROGMEM = "10.25.35.1\0";
const char netmask[] PROGMEM = "255.255.255.0\0";

// SPI setup
const char setSPIConfigCommand[] PROGMEM = "AT+SPICONF=0,0\0"; // Need reset/restart after sending this?

// Device setup
//const char getVersionCommand[] PROGMEM = "AT+VER=?\r\n";
const char setRegulatoryDomainCommand[] PROGMEM = "AT+WREGDOMAIN=0\r\n\0"; // 0 => FCC
const char setOpModeCommand[] PROGMEM = "AT+WM=2\r\n\0"; // 2 => Limited AP
const char setSecurityCommand[] PROGMEM = "AT+WSEC=1\r\n\0"; // 1 => Open
const char setRadioOpEnableCommand[] PROGMEM = "AT+WRXACTIVE=1\r\n\0"; // 1 => Enable, 0 => Disable
const char setRadioOpDisableCommand[] PROGMEM = "AT+WRXACTIVE=0\r\n\0"; // 1 => Enable, 0 => Disable
const char setTransmitPowerCommand[] PROGMEM = "AT+WP=0\0"; // 0 => Max Power
const char setKeepAliveCommand[] PROGMEM = "AT+PSPOLLINTRL=0\r\n\0"; // 0 => Disable the timer

// Network setup
const char setDHCPCommand[] PROGMEM = "AT+NDHCP=1\r\n\0";
const char setNetParamsCommand[] PROGMEM = "10.25.35.1,255.255.255.0,10.25.35.1\r\n\0";
const char setDHCPServerCommand[] PROGMEM = "AT+DHCPSRVR=1\r\n\0";
//const char * associateNetworkCommand = "";

// Connections
const char setBulkTransfer[] PROGMEM = "AT+BDATA=1\r\n\0"; // Enables bulk transfer
const char udpOutConnectCommand[] PROGMEM = "AT+NCUDP=10.25.35.255,41501\r\n\0";
//const char updInConnectCommand[] PROGMEM = "AT+NCUDP=10.25.35.255,41500\r\n";

const char closeAllConnectionsCommand[] PROGMEM = "AT+NCLOSEALL\r\n\0";
const char startUdpServerCommand[] PROGMEM = "AT+NSUDP=41500\r\n\0"; // On 'ASIP In' port?

const char commandSuccessful[] PROGMEM = "OK\r\0";
const char connectResponse[] PROGMEM = "CONNECT \0";
const char transferSuccessful[] PROGMEM = "\x1BO\0";
const char udpBulkTransferSeq[] PROGMEM = "\x1BY";
const char udpBulkTransferReceived[] PROGMEM = "\x1By\0";

const char udpBulkTransferEnd[] PROGMEM = "10.25.35.255:41500:";//"address:port:length\0";

PGM_P const wifiCmdArray[] PROGMEM = {
    setSPIConfigCommand,
    setRegulatoryDomainCommand,
    setOpModeCommand,
    setSecurityCommand,
    setRadioOpEnableCommand,
    setRadioOpDisableCommand,
    setTransmitPowerCommand,
    setKeepAliveCommand,   
    setDHCPCommand,
    setNetParamsCommand,
    setDHCPServerCommand,
    setBulkTransfer,
    udpOutConnectCommand,
    //updInConnectCommand,
    closeAllConnectionsCommand,
    startUdpServerCommand,
    udpBulkTransferSeq,
    udpBulkTransferEnd
    
};

PGM_P const wifiResponseStrings[] PROGMEM = {
    commandSuccessful,
    connectResponse,
    transferSuccessful,
    udpBulkTransferReceived
    
};

//const WORD asipInPort  = 41500;
//const WORD asipOutPort = 41501;
//const WORD gpsPort     = 41502;
//const WORD uatPort     = 41503;


static BYTE wifiCommReady;
static BYTE wifiDataReady;

static BYTE asipCID;
static BYTE asipServerCID;

static char expectedResponseString[10];

static BYTE expectingWifiResponse;
static char * expectedWifiResponsePtr;

//static BYTE * rxDataBuffPtr;

PGM_P p; // Pointer used to locate const strings stored in PROGMEM

static void sendAssociateNetworkCommand(void);
static void buildSSID(char * ssid);


void initWiFiModule(void)
{
    //char wifiTxBuffer[40];
    
    setWifiResetHigh();
    
    expectingWifiResponse = 0;
    expectedWifiResponsePtr = NULL;
    
    //rxDataBuffPtr = getSPIRxBufferPtr();
    
    //wifiDataReady = 0;
    
    //wifiTxBufferHead = 0;
    //wifiTxBufferTail = 0;
    
    // reset?
    // check connection?   
    // config SPI?
    //expectingWifiResponse = 1;
    sendWifiCommand(0); // SPI config   

    // Reset to apply SPI clock settings
    setWifiResetLow();
    _delay_loop_2(50000); // ~25 ms with 8MHz clock
    setWifiResetHigh();
    _delay_loop_2(50000);

    sendWifiCommand(11); // Enable bulk transfer     
    
    // config device settings
    sendWifiCommand(1); // Reg Domain
    sendWifiCommand(2); // Op Mode
    sendWifiCommand(3); // Security
    sendWifiCommand(4); // Radio Enable
    sendWifiCommand(6); // Transmit Power
    sendWifiCommand(7); // Keep Alive                     
    
    // config network settings
    sendWifiCommand(8); // DHCP
    sendWifiCommand(9); // Net Params
    sendWifiCommand(10); // DHCP Server
    sendAssociateNetworkCommand(); // Associate Network    
    
    // config connections
    sendWifiCommand(13); // Close All Connections

    // open comm port
    // Retrieve the response string from Flash and then copy to the buffer
    memcpy_P(&p, &wifiResponseStrings[1], sizeof(PGM_P)); // "CONNECT "
    strcpy_P(expectedResponseString, p);
    expectedWifiResponsePtr = expectedResponseString;
    expectingWifiResponse = 1;
    do
    {
        sendWifiCommand(12); // Open UDP Out Port (Get CID)
        _delay_loop_2(50000); // ~25 ms with 8MHz clock
    } while(!getSpiResponseReceivedFlag());

    memcpy_P(&p, &wifiResponseStrings[1], sizeof(PGM_P)); // "CONNECT "
    strcpy_P(expectedResponseString, p);
    expectedWifiResponsePtr = expectedResponseString;
    expectingWifiResponse = 2;
    do 
    {
        sendWifiCommand(14); // Start UDP Server (Get CID)    
        _delay_loop_2(50000); // ~25 ms with 8MHz clock    
    } while(!getSpiResponseReceivedFlag());


    expectingWifiResponse = 0;
}

void setWifiResetLow(void)
{
    PORTD &= ~(_BV(PD1));
}

void setWifiResetHigh(void)
{
    PORTD |= _BV(PD1);
}

// Set up the AP SSID and transmit channel...
// Assumes default WiFi channel 6
static void sendAssociateNetworkCommand(void)
{
    char buff[20];
    
    buildSSID(buff);
    strcpy(&buff[10], ",,6\r\n\0");
    
    // TODO??...Send via SPI...
    addBytesToWifiTxBuffer((BYTE *)buff, 16);
}

void sendWifiCommand(BYTE arrayIndex)
{
    char wifiConfigTxBuffer[40]; // Tx buffer for WiFi config messages

    // Retrieve the command string from Flash and then add to Tx buffer
    memcpy_P(&p, &wifiCmdArray[arrayIndex], sizeof(PGM_P));
    strcpy_P(wifiConfigTxBuffer, p);

    // Send via SPI...
    addBytesToWifiTxBuffer((BYTE *)wifiConfigTxBuffer, __strlen_P(p));
}



// Build the WiFi SSID from the serial number and product name
static void buildSSID(char * ssid)
{
    BYTE * serialPtr;
    
    serialPtr = getSerialNumPtr();
    
    ssid[0] = 'C';
    ssid[1] = 'D';
    ssid[2] = 'S';
    ssid[3] = 'J';
    ssid[4] = 'B';
    ssid[5] = '_';
    ssid[6] = *serialPtr++;
    ssid[7] = *serialPtr++;
    ssid[8] = *serialPtr++;
    ssid[9] = *serialPtr++;
    ssid[10] = '\0';                           
}

void initWifiWakeupInterrupt(void)
{
    // Enable Pin Change Interrupt on PD7
    PCMSK2 |= _BV(PCINT23);
    
    // Activate Pin Change Interrupt group 2
    PCICR |= _BV(PCIE2);
    
    wifiCommReady = 0;
    
}

// ISR for External Pin Change (WiFi Host Wake-Up Signal)
ISR(PCINT2_vect)
{
    // Check if this is a rising or falling edge...
    if(IS_WIFI_WAKEUP_HIGH) // Rising edge
    {
        if(wifiCommReady) // Check for 1-time communications ready flag
        {
            // Set flag to start clocking out SPI data...
            wifiDataReady = 1;
        }
        wifiCommReady = 1;
    }
    else // Falling edge
    {
        // Stop clocking SPI data
        wifiDataReady = 0;
    }
}

BYTE getWifiReadyFlag(void)
{
    return wifiDataReady;
}

BYTE getExpectedWifiResponseType(void)
{
    return expectingWifiResponse;
}

char * getExpectedWifiResponsePtr(void)
{
    return expectedResponseString;
}

void setAsipCID(BYTE val)
{
    asipCID = val;
}

void setAsipServerCID(BYTE val)
{
    asipServerCID = val;
}

BYTE getAsipServerCID(void)
{
    return asipServerCID;
}


