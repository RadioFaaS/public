

#ifndef BLOCKAGEPROCESSOR_H_
#define BLOCKAGEPROCESSOR_H_

#include "Config.h"

void initBlockageProcessor(void);
void readCalValues(void);
void applyCalibrationToData(void);
void processBlockageData(void);
BYTE getMaxSensorDataIndex(BYTE rowNumber);
BYTE getSensorOORFlag(void);
BYTE getNoMagnetRowMask(void);
BYTE getNoMagDetectionDone(void);
BYTE getFlowDetectedFlag(void);

void setSensorDataDisableFlag(void);
void clearSensorDataDisableFlag(void);

void setEnableDebugFlag(void);
void clearEnableDebugFlag(void);
#endif /* BLOCKAGEPROCESSOR_H_ */