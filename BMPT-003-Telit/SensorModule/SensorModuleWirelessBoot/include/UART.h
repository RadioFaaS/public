
#ifndef UART_H_
#define UART_H_

#include "Config.h"
// 8 MHz FOSC
#define BAUD_RATE_9600_8MHZ      25  
#define BAUD_RATE_19200_8MHz     12
#define BAUD_RATE_115200_8MHZ    1

// 16 MHz FOSC
#define BAUD_RATE_9600_16MHZ      51
#define BAUD_RATE_19200_16MHZ     25
#define BAUD_RATE_115200_16MHZ    3//3.4//3
#define BAUD_RATE_38400_16MHZ     11

#define BAUD_RATE_115200_DIVISOR_16MHZ 34

void initUART(void);
//BYTE getUARTData(void);
//void putUARTData(BYTE data);
BOOL addBytesToWifiTxBuffer(const BYTE * data, BYTE size);
BOOL getUART_IsRXFull(void);
BYTE getUART_TxBytesFree(void);
void clearUART_RX_Buffer(void);
void setUARTBaud(BYTE baud);
BOOL getIsTxIdle(void);

void setExpectedWifiResponse(BYTE val);

void setUART_Baud(BYTE brr_reg);
BOOL getIsTX_Idle(void);

#endif /* UART_H_ */