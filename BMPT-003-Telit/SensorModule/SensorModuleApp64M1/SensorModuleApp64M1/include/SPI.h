

#ifndef SPI_H_
#define SPI_H_

#include "Config.h"


void initSPI(void);
//void transmitSPI(BYTE * txData, BYTE size, BYTE * rxData);
//void processReceivedWifiData(void);
void addBytesToSensorDataBuffer(BYTE * data, BYTE size);
void addBytesToWifiTxBuffer(BYTE * data, BYTE size);
void forceStartSPITxSequence(void);
BYTE parseIncomingGainspanData(BYTE nextChar, char * expectedPtr, BYTE expectedType);
BYTE getSpiResponseReceivedFlag(void);

#endif /* SPI_H_ */