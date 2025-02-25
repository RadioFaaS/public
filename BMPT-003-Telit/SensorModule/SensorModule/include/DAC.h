
#ifndef DAC_H_
#define DAC_H_

#include "Config.h"

void initDAC(void);
void setDAC(WORD vout);
WORD getDAC(void);


#endif /* DAC_H_ */