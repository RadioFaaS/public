
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
#define BAUD_RATE_115200_16MHZ    3//3.4
#define BAUD_RATE_38400_16MHZ     11

#define BAUD_RATE_115200_DIVISOR_16MHZ 34

enum
{
    WIFI_RESPONSE_NONE = 0,
    WIFI_RESPONSE_OK,
    WIFI_RESPONSE_CONNECT_OUT,
    WIFI_RESPONSE_CONNECT_IN
};



void initUART(void);
BOOL addBytesToWifiTxBuffer(const BYTE * data, BYTE size);
BYTE getUART_TxBytesFree(void);
void setUARTBaud(BYTE baud);
BOOL getIsTxIdle(void);
BOOL getUARTTxBuffEmpty(void);

void setExpectedWifiResponse(BYTE val);

#endif /* UART_H_ */