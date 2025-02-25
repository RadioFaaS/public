/*
 * LED.c
 *
 * Created: 8/3/2012 4:30:00 PM
 *  Author: sbailey
 */ 
#include "LED.h"
#include "Config.h"
#include "TaskList.h"
#include "CAN.h"
#include "BIT.h"
#include "BlockageProcessor.h"

static BYTE LEDState;
static BYTE LEDBlinkStartTime;

#define POST_DURATION_TICK_COUNT    8  // ~2 sec
#define SLOW_BLINK_TICK_COUNT       4  // ~500 msec
#define FAST_BLINK_TICK_COUNT       1  //
#define BLOCKAGE_TIMEOUT_TICK_COUNT 8  // ~2 sec

static BYTE flashCounter[4];
static BYTE timerPostStart = 0;
static void forceOnAllLEDs(void);
static void clearFlashCounters(void);

// Variable initialization for the 
//  LED state machine
void initLEDStateMachine(void)
{
    LEDState = LED_STATE_POST;
    LEDBlinkStartTime = 0;
    clear4HzTimer();
    clearFlashCounters();

    timerPostStart = get4HzTimer();
}

// Updates the system/LED state machine
// based on latest system status
void stepLEDStateMachine(void)
{
    BYTE blockedRows = 0;
    BYTE timerVal;
    BYTE noMagMask = 0;
	

    timerVal = get4HzTimer();
    noMagMask = getNoMagnetRowMask();

    switch(LEDState)
    {
        case LED_STATE_POST:
            forceOnAllLEDs();
            if(timerVal > (BYTE)(POST_DURATION_TICK_COUNT + timerPostStart))
            {
                LEDState = LED_STATE_INIT_NO_FLOW;
            }       
            break;

        case LED_STATE_INIT_NO_FLOW:
            if(getActiveSystemError())
            {
                LEDState = LED_STATE_ERROR;
                turnOffAllLEDs();
                LEDBlinkStartTime = timerVal;
            }
            else if(getFlowDetectedFlag() == 1)
            {
                forceOnAllLEDs();
                clearFlashCounters();
                LEDState = LED_STATE_NORMAL;
            }
            else // No flow detected yet
            {
                // Use one of the counters to time the LED toggle.
                // All 4 LEDS should flash together so no need to use all 4 counters.
                flashCounter[0]++;

                if(flashCounter[0] > SLOW_BLINK_TICK_COUNT)
                {
                    toggleAllLEDs();
                    flashCounter[0] = 0;
                }
            }

            break;
            
        case LED_STATE_NORMAL:
            if(getActiveSystemError())
            {
                LEDState = LED_STATE_ERROR;
                turnOffAllLEDs();
                LEDBlinkStartTime = timerVal;
            }
            else if(getActiveBlockageFlag()) //blockage message received
            {
                LEDState = LED_STATE_BLOCKAGE; 
                turnOnAllLEDs();
                LEDBlinkStartTime = timerVal;
            }
            else
            {
                turnOnAllLEDs();
            }            
            
            break;
        
        case LED_STATE_BLOCKAGE:
            if(getActiveSystemError())
            {
                LEDState = LED_STATE_ERROR;
                turnOffAllLEDs();
                LEDBlinkStartTime = timerVal;
            }
            else
            {
                blockedRows = getActiveBlockageRows();

                // Check for blockage timeouts                
                if(blockedRows & 0x01)
                {
                    flashCounter[0]++;

                    if(flashCounter[0] > SLOW_BLINK_TICK_COUNT)
                    {
                        toggleRow1LED();
                        flashCounter[0] = 0;
                    }
                }
                else if(noMagMask & 0x01)
                {
                    PORTB &= ~(_BV(PB1));
                }
                else
                {
                    turnOnRow1LED();
                    flashCounter[0] = 0;
                }
                
                if(blockedRows & 0x02)
                {
                    flashCounter[1]++;

                    if(flashCounter[1] > SLOW_BLINK_TICK_COUNT)
                    {
                        toggleRow2LED();
                        flashCounter[1] = 0;
                    }
                }
                else if(noMagMask & 0x02)
                {
                    PORTC &= ~(_BV(PC6));
                }
                else
                {
                    turnOnRow2LED();
                    flashCounter[1] = 0;
                }
                
                if(blockedRows & 0x04)
                {
                    flashCounter[2]++;

                    if(flashCounter[2] > SLOW_BLINK_TICK_COUNT)
                    {
                        toggleRow3LED();
                        flashCounter[2] = 0;
                    }
                }
                else if(noMagMask & 0x04)
                {
                    PORTB &= ~(_BV(PB4));
                }
                else
                {
                    turnOnRow3LED();
                    flashCounter[2] = 0;
                }
                
                if(blockedRows & 0x08)
                {
                    flashCounter[3]++;

                    if(flashCounter[3] > SLOW_BLINK_TICK_COUNT)
                    {
                        toggleRow4LED();
                        flashCounter[3] = 0;
                    }
                }
                else if(noMagMask & 0x08)
                {
                    PORTB &= ~(_BV(PB3));
                }
                else
                {
                    turnOnRow4LED();
                    flashCounter[3] = 0;
                }                
                if(blockedRows == 0)
                {
                    LEDState = LED_STATE_NORMAL;
                    turnOnAllLEDs();
                    clearActiveBlockageFlag();
                }
            }
                       
            break;
        
        case LED_STATE_ERROR:
            if(!getActiveSystemError())
            {
                LEDState = LED_STATE_NORMAL; 
            }
            else
            {
                if(timerVal >= (BYTE)(LEDBlinkStartTime + FAST_BLINK_TICK_COUNT)) // timer count > fast blink count
                {
                    toggleAllLEDs();
                    LEDBlinkStartTime = timerVal;
                }
            }            
 
            break;
        
        default:
            break;
    }
}

// Turns on all row LEDs 
//  and ignores the unused row mask
static void forceOnAllLEDs(void)
{   
    PORTB |= (_BV(PB4));
    PORTB |= (_BV(PB3));
    PORTC |= (_BV(PC6));
    PORTB |= (_BV(PB1));
}

// Turns on all row LEDs except those with no magnets
void turnOnAllLEDs(void)
{
    BYTE rowMask;    
    rowMask = getNoMagnetRowMask();

    rowMask = ~(rowMask); // Find the rows with magnets

    // First turn off all LEDS 
    PORTB &= ~(_BV(PB1) | _BV(PB3) | _BV(PB4));
    PORTC &= ~(_BV(PC6));

    if(rowMask & 0x01)
    {
        PORTB |= (_BV(PB1));
    }

    if(rowMask & 0x02)
    {
        PORTC |= (_BV(PC6));
    }

    if(rowMask & 0x04)
    {
        PORTB |= (_BV(PB4));
    }

    if(rowMask & 0x08)
    {
        PORTB |= (_BV(PB3));
    }

}

// Forces all row LEDs to off
void turnOffAllLEDs(void)
{
    PORTB &= ~(_BV(PB1) | _BV(PB3) | _BV(PB4));
    PORTC &= ~(_BV(PC6));
}

// Toggles all row LEDs
void toggleAllLEDs(void)
{
    PORTB ^= (_BV(PB1) | _BV(PB3) | _BV(PB4));
    PORTC ^= (_BV(PC6));
}

void toggleRow1LED(void)
{
    PORTB ^= (_BV(PB1));
}

void toggleRow2LED(void)
{
    PORTC ^= (_BV(PC6));
}

void toggleRow3LED(void)
{
    PORTB ^= (_BV(PB4));
}

void toggleRow4LED(void)
{
    PORTB ^= (_BV(PB3));
}

void turnOnRow1LED(void)
{
    PORTB |= (_BV(PB1));
}

void turnOnRow2LED(void)
{
    PORTC |= (_BV(PC6));
}

void turnOnRow3LED(void)
{
    PORTB |= (_BV(PB4));
}

void turnOnRow4LED(void)
{
    PORTB |= (_BV(PB3));
}

// Resets the counters for LED toggling
static void clearFlashCounters(void)
{
    flashCounter[0] = 0;
    flashCounter[1] = 0;
    flashCounter[2] = 0;
    flashCounter[3] = 0;
}

