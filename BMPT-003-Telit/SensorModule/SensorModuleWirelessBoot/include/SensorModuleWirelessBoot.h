#ifndef SENSORMODULEWIRELESSBOOT_H_
#define SENSORMODULEWIRELESSBOOT_H_

void setWifiDoneFlag(void);
WORD pushPageBuffer(BYTE * pBuf, WORD nBytes);
void setFwUpdateCompleteFlag(void);
BYTE compareSerialNums(BYTE * num1, BYTE * num2);
char * getSerialNumPtr(void);
void readHwPartNum(BYTE * partNumPtr);
void readHwVersion(BYTE * versionPtr);
void readFwPartNum(BYTE * fwPartNum);
char * getFwVersionPtr(void);
void sendReadyForFwUpdate(void);

#endif /* SENSORMODULEWIRELESSBOOT_H_ */