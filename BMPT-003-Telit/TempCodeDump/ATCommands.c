
/** @brief : Network Setup AT commands for Access point */

/** @cite: Found*/
const char setRegulatoryDomainCommand[] PROGMEM = "AT+WREGDOMAIN=0\r\n"; // 0 => FCC

/** @cite: Found */ 
const char setOpModeCommand[] PROGMEM = "AT+WM=2\r\n"; // 2 => Limited AP   


/** @cite: Not needed merged into single command in new module */
const char setSecurityCommand[] PROGMEM = "AT+WSEC=1\r\n"; // 1 => Open
const char setRadioOpEnableCommand[] PROGMEM = "AT+WRXACTIVE=1\r\n"; // 1 => Enable, 0 => Disable



//const char setRadioOpDisableCommand[] PROGMEM = "AT+WRXACTIVE=0\r\n\0"; // 1 => Enable, 0 => Disable
/** @cite: Found */ 
const char setTransmitPowerCommand[] PROGMEM = "AT+WP=0\r\n"; // 0 => Max Power, 7 => Min Power

/** @cite: NOT Found */ 
const char setKeepAliveCommand[] PROGMEM = "AT+PSPOLLINTRL=0\r\n"; // 0 => Disable the timer

/** @cite: Same */ 
const char setATSyncCommand[] PROGMEM = "\r\n\r\nAT\r\n";			// Simple AT command with sync chars

/** @cite: Same */ 
const char setVerboseEnableCommand[] PROGMEM = "ATV1\r\n";	// Verbose mode enable
const char setEchoEnableCommand[] PROGMEM = "ATE1\r\n";		// Echo mode enable
const char setEchoDisableCommand[] PROGMEM = "ATE0\r\n";		// Echo mode disable


//const char setSoftResetCommand[] PROGMEM = "AT+RESET\r\n";		// Soft reset the GS module

/** @cite: Not needed merged into single command in new module */
const char setSSIDCommand[] PROGMEM = "AT+WA=";					// Set SSID - Note, don't set \r\n yet, SSID to follow
//const char setSSID_HACK_Command[] PROGMEM = "AT+WA=CDSJB-MOD\r\n";

/** @cite: NOT USED IN CODE */
const char startTcpServerCommand[] PROGMEM = "AT+NSTCP=41509\r\n";

// Network setup
/** @cite: Not needed merged into single command in new module */
const char setDHCPCommand[] PROGMEM = "AT+NDHCP=1\r\n";
const char setNetParamsCommand[] PROGMEM = "AT+NSET=192.168.1.1,255.255.255.0,192.168.1.1\r\n";
const char setDHCPServerCommand[] PROGMEM = "AT+DHCPSRVR=1\r\n";
//const char * associateNetworkCommand = "";

// Connections
const char setBulkTransfer[] PROGMEM = "AT+BDATA=1\r\n"; // Enables bulk transfer
const char udpOutConnectCommand[] PROGMEM = "AT+NCUDP=192.168.1.255,41501\r\n";
//const char updInConnectCommand[] PROGMEM = "AT+NCUDP=10.25.35.255,41500\r\n";

/** @cite: Found */ 
const char closeAllConnectionsCommand[] PROGMEM = "AT+NCLOSEALL\r\n";

const char startUdpServerCommand[] PROGMEM = "AT+NSUDP=41500\r\n"; // On 'ASIP In' port?

//const char commandSuccessful[] PROGMEM = "OK\r\0";
//const char connectResponse[] PROGMEM = "CONNECT \0";
//const char transferSuccessful[] PROGMEM = "\x1BO\0";
////const char udpBulkTransferSeq[] PROGMEM = "\x1BY";
const char udpBulkTransferSeq[] PROGMEM = "\x1BZ";
//const char udpBulkTransferReceived[] PROGMEM = "\x1By\0";

const char udpBulkTransferEnd[] PROGMEM = "192.168.1.255:41500:";//"address:port:length\0";

/** @cite: Found */ 
const char setBaudRateCommand[] PROGMEM = "ATB=115200,8,n,1\r\n"; // Change the baud rate
const char setBaudRateCommand2[] PROGMEM = "ATB=38400,8,n,1\r\n"; // Change the baud rate
/** @cite: Found */ 
const char setFlowControlCommand[] PROGMEM = "AT&R1\r\n"; // Enable hardware flow control



/*** NEW COMMANDS */

/** @brief: Regulatory Domain Config
 * @attention : This command only works after WNI is created and same handle value should be provided
 * AT+WREGDS=1,"TELEC" 
 * 1 = Network Handle Value Whandle 
 * "TELEC" or "FCC" or "ETSI" can be configured
 */
const char setRegulatoryDomainCommand[] PROGMEM = "AT+WREGDS=1,\"FCC\"\r\n";

/**  @brief : Wifi Network mode config Access point or Station Mode 
 * 1 => Access point mode 
 * Bandwidht Default= 20MHz 
 * Concurrent mode: Disabled
*/
const char setOpModeCommand[] PROGMEM = "AT+WNI=1\r\n"; 

/** @brief : Set the transmission Rate of data  
 * AT+WTXRATES=<WHandle>,<Transmission rate>
*/

const char setCountryCode[] PROGMEM = "AT+WCCS=1,\"US\"\r\n";

/**  @brief : Wifi Network mode config Access point or Station Mode 
 *   1 = Whandle
 *   0 = Max TX Power      4 = Minimum Power 
*/
const char setTransmitPowerCommand[] PROGMEM = "AT+WTXPOWERS=1,0\r\n"; 

/**  @brief : Send a syncing value just basic AT command with LF and CR  */
const char setATSyncCommand[] PROGMEM = "\r\n\r\nAT\r\n";			// Simple AT command with sync chars

/**  @brief : Verbose mode enable 
 *  1 = Enable 
 *  0 = Disable
  */
const char setVerboseEnableCommand[] PROGMEM = "ATV1\r\n";	

/**  @brief : Echo mode enable 
 *  1 = Enable 
 *  0 = Disable
  */
const char setEchoEnableCommand[] PROGMEM = "ATE1\r\n";		// Echo mode enable
const char setEchoDisableCommand[] PROGMEM = "ATE0\r\n";		// Echo mode disable


/** @brief : Close the connection that we try to initialize 
 * 1 = Whandle value
*/
const char closeAllConnectionsCommand[] PROGMEM = "AT+WNDC=1\r\n";

/** @brief : Change baud rate 
 * 0 = High Speed UART    1 = Debug UART (TX1,RX1)
 * 115200 = Baud rate value
 * 8 = Bits per character
 * 0 = Parity mode
 * 1 = Stop bits
*/
const char setBaudRateCommand[] PROGMEM = "ATB=0,115200,8,0,1\r\n"; // Change the baud rate

/** @brief : Set the hardware flow control  
 * 3 = Enables Hardware flow control
 * 0 = Disables HW Flow Control 
*/
const char setFlowControlCommand[] PROGMEM = "AT&K3\r\n"; // Enable hardware flow control