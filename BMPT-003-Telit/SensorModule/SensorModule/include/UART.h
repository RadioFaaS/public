
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
#define BAUD_RATE_115200_16MHZ    3

void initUART(void);
BYTE getUARTData(void);
void putUARTData(BYTE data);

#endif /* UART_H_ */