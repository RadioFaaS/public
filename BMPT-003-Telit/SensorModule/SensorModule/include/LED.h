

#ifndef LED_H_
#define LED_H_

enum
{
    LED_OFF,
    LED_ON,
    LED_FLASH,
    LED_FAST_FLASH    
};

enum
{
    LED_STATE_NORMAL,
    LED_STATE_ERROR,
    LED_STATE_BLOCKAGE,
    LED_STATE_POST,
    LED_STATE_INIT_NO_FLOW        
};

void initLEDStateMachine(void);
void stepLEDStateMachine(void);
void turnOnAllLEDs(void);
void turnOffAllLEDs(void);
void toggleAllLEDs(void);

void toggleRow1LED(void);
void toggleRow2LED(void);
void toggleRow3LED(void);
void toggleRow4LED(void);

void turnOnRow1LED(void);
void turnOnRow2LED(void);
void turnOnRow3LED(void);
void turnOnRow4LED(void);


#endif /* LED_H_ */