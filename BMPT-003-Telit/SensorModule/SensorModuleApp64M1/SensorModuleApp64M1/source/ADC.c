/*******************************************************************************
 * Copyright 2012 Appareo Systems
 * All rights reserved
 * This software and/or material is the property of Appareo Systems.
 * All use, disclosure, and/or reproduction not specifically authorized in
 * writing by Appareo Systems is prohibited.
 *
 * PROJECT:       Neptune LBMS
 *
 * FILE NAME:     $Workfile:   ADC.c  $
 *
 * DESCRIPTION:   Analog to Digital sampling.
 *
 * REVISION HISTORY:
* $Log:$
*/
/*==============================================================================
   INCLUDE FILES
 *============================================================================*/

#include "ADC.h"
#include "BlockageProcessor.h"
#include "NValloc.h"
#include "DAC.h"
#include "LED.h"
#include "CAN.h"
#include <avr/eeprom.h>
#include <util/delay_basic.h>
/*==============================================================================
   DEFINES AND ENUMERATIONS
 *============================================================================*/


/*==============================================================================
   TYPEDEFS
 *============================================================================*/


/*==============================================================================
   GLOBAL VARIABLES
 *============================================================================*/

 extern WORD sensorBaseline[NUM_SENSOR_ROWS][NUM_SENSORS_PER_ROW];
/*==============================================================================
   STATIC VARIABLES
 *============================================================================*/
//array where the result of conversion is stored

static WORD accumulatedADCData[NUM_SENSOR_ROWS][NUM_SENSORS_PER_ROW];
static WORD avgADCData[NUM_SENSOR_ROWS][NUM_SENSORS_PER_ROW];


//channel to be converted
static BYTE channelIndex;

// Sensor row currently selected
static BYTE activeRow;

// Counter used to do averaging
static BYTE sampleCount;

static BYTE runCalibrationFlag;
static WORD calRowCount;

// Indices for performing a cal
static BYTE calRow = 0;
static BYTE calSensor = 0;

// Cal results data
static BYTE calFail = 0;
static BYTE calFailRow = 0;
static BYTE calFailSensor = 0;

static BYTE checkCalVals(BYTE row, BYTE sensor);

// Table for specifying the utilized ADC channels in the desired sampling order
const static BYTE channelMuxTable[] = {0x09, 0x02, 0x03, 0x08, 0x05, 0x06, 0x07};

extern WORD sensorCalVals[NUM_SENSOR_ROWS][NUM_SENSORS_PER_ROW];
static BYTE sensorSampleCount = 0;
static BYTE sensorsReadyFlag = 0;

static void setNextDAC(void);
static void executeCalibration(void);

/*******************************************************************************
FUNCTION NAME: initADC

DESCRIPTION:   This function initializes ADC registers

ARGUMENTS:     None

RETURN:        None

REVISION HISTORY:

DATE:       WHO:  CHANGE DESCRIPTION:
======================================================================
06/24/2005  VS   Created.
*******************************************************************************/
void initADC(void)
{
   clearADCData();
   
   ADMUX = 0;
   //Set REFSn bits in the ADMUX register to make AVCC voltage as the reference voltage
   //REFS0 =1, REFS1 = 0
   ADMUX |= _BV(REFS0);
   ADMUX &= ~_BV(REFS1);

   //ADLAR = 0 in ADMUX (disables left-adjust of conversion result)
   ADMUX &= ~_BV(ADLAR);
   //ADMUX MUX0-MUX4 bits are already 0 indicating channel 0 selection
   
   //Initial channel selection should be first entry of the table
   ADMUX  |= channelMuxTable[0];

   //Set ADPS bits in ADCSRA to select pre-scaler 128, set ADPS0=ADPS1=ADPS2=1 
   ADCSRA |= (_BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2));
   //ADFR bit is already 0 to select polled mode(not free running mode)

   //Set ADIE bit in ADCSRA to 1 to enable interrupt
   ADCSRA |= _BV(ADIE);
   //Set the ADEN bit in the ADCSRA register to enable ADC
   ADCSRA |= _BV(ADEN);

    // Select channel for the initial conversion
    ADMUX &= ~(_BV(MUX0) | _BV(MUX1) | _BV(MUX2) | _BV(MUX3) | _BV(MUX4));
    ADMUX |= channelMuxTable[0];
 
   // Init index for next conversion
   channelIndex = 0;
   
   sampleCount = 0;

   runCalibrationFlag = 0;
   calRowCount = 0;
   
   activeRow = 1; // First row sampled (after increment) will be row 1
   selectRow1();
}

/*******************************************************************************
FUNCTION NAME: startADCConversion

DESCRIPTION:   This function starts the conversion of ADC channels

ARGUMENTS:     None

RETURN:        None

REVISION HISTORY:

DATE:       WHO:  CHANGE DESCRIPTION:
======================================================================
06/24/2005  VS   Created.
*******************************************************************************/
void startADCConversion(void)
{
    if(runCalibrationFlag)
    {
         executeCalibration();
    }
    else
    {
         // Write ADSC to 1 in ADCSRA to start conversion.
         //This bit will be reset automatically when conversion is complete
         ADCSRA |= _BV(ADSC);  
    }
}

// Performs the sensor calibration by tracking the 
//  ADC setup/ready states and compares the averaged
//  samples to the acceptable voltage window
static void executeCalibration(void)
{
    BYTE calCheck;
    WORD dacVal;

    static BYTE calRetryCount = 0;
    static BYTE calLastStep = 0;
    static BYTE waitCycle = 1;
    static BYTE calInitDone = 0;

    // Set next DAC and row
    // wait for next iteration (setup time)
    // start conversion
    // check cal vals (do average)
    // if ok, set next DAC and row
    // if not ok, repeat same DAC and row
    // start conversion
    if(!calInitDone)
    {
        calInitDone = 1;
        waitCycle = 1;
        selectRow1();
        activeRow = 1;
        setDAC(512);
        calRow = 0;
        calSensor = 0;
        channelIndex = 0;
        clearADCData();
        // Clear the Mux values
        ADMUX &= ~(_BV(MUX0) | _BV(MUX1) | _BV(MUX2) | _BV(MUX3) | _BV(MUX4));            
        // Select the channel for first conversion
        ADMUX |= channelMuxTable[channelIndex];
    }        
    else if(waitCycle)
    {            
        waitCycle = 0;
        ADCSRA |= _BV(ADSC); // Start conversion
    }
    else
    {
        // Set the DAC and row for the next sample sequence
        _CLI();
        dacVal = getDAC();
            
        calCheck = checkCalVals(calRow, calSensor);
        if(calCheck == CAL_AVG_IN_RANGE)
        {
            calFail |= 0;
                
            // write DAC val to EEPROM
            // Write the values as Little Endian
            eeprom_busy_wait();
            eeprom_write_byte((BYTE *)(EE_SENSOR_CAL_LOC + (calRow * 14) + (calSensor * 2)), (BYTE)(dacVal & 0xFF));
            eeprom_busy_wait();
            eeprom_write_byte((BYTE *)(EE_SENSOR_CAL_LOC + (calRow * 14) + (calSensor * 2) + 1), (BYTE)((dacVal & 0xFF00) >> 8));

            eeprom_busy_wait();
            eeprom_write_byte((BYTE *)(EE_SENSOR_BASE_LOC + (calRow * 14) + (calSensor * 2)), (BYTE)(sensorBaseline[calRow][calSensor] & 0xFF));
            eeprom_busy_wait();
            eeprom_write_byte((BYTE *)(EE_SENSOR_BASE_LOC + (calRow * 14) + (calSensor * 2) + 1), (BYTE)((sensorBaseline[calRow][calSensor] & 0xFF00) >> 8));

            calSensor++;
            setDAC(512);
            ADMUX &= ~(_BV(MUX0) | _BV(MUX1) | _BV(MUX2) | _BV(MUX3) | _BV(MUX4));
            if(calSensor < NUM_SENSORS_PER_ROW)
            {
                ADMUX |= channelMuxTable[calSensor];
            }
            else
            {
                ADMUX |= channelMuxTable[0];
            }
            toggleAllLEDs();
        }
        else if(calRetryCount > 8)
        {
            calFail |= 0;
                
            eeprom_busy_wait();
            eeprom_write_byte((BYTE *)(EE_SENSOR_CAL_LOC + (calRow * 14) + (calSensor * 2)), (BYTE)(dacVal & 0xFF));
            eeprom_busy_wait();
            eeprom_write_byte((BYTE *)(EE_SENSOR_CAL_LOC + (calRow * 14) + (calSensor * 2) + 1), (BYTE)((dacVal & 0xFF00) >> 8));

            eeprom_busy_wait();
            eeprom_write_byte((BYTE *)(EE_SENSOR_BASE_LOC + (calRow * 14) + (calSensor * 2)), (BYTE)(sensorBaseline[calRow][calSensor] & 0xFF));
            eeprom_busy_wait();
            eeprom_write_byte((BYTE *)(EE_SENSOR_BASE_LOC + (calRow * 14) + (calSensor * 2) + 1), (BYTE)((sensorBaseline[calRow][calSensor] & 0xFF00) >> 8));

            calSensor++;
            setDAC(512);
            ADMUX &= ~(_BV(MUX0) | _BV(MUX1) | _BV(MUX2) | _BV(MUX3) | _BV(MUX4));
            if(calSensor < NUM_SENSORS_PER_ROW)
            {
                ADMUX |= channelMuxTable[calSensor];
            }
            else
            {
                ADMUX |= channelMuxTable[0];
            }
            toggleRow3LED();
            calRetryCount = 0;
        }
            
        else if(calCheck == CAL_AVG_TOO_LOW) // Avg too low
        {
            if(dacVal > 412)
            {
                dacVal--;
                setDAC(dacVal);
                toggleRow1LED();
                if(calLastStep != 0)
                {
                    calLastStep = 0;
                    calRetryCount++;
                }
                else
                {
                    calRetryCount = 0;
                }
            }
            else
            {
                // give up...
                calFail = 1;
                calFailRow = calRow;
                calFailSensor = calSensor;
                    
                calSensor++;
                setDAC(512);
                ADMUX &= ~(_BV(MUX0) | _BV(MUX1) | _BV(MUX2) | _BV(MUX3) | _BV(MUX4));
                if(calSensor < NUM_SENSORS_PER_ROW)
                {
                    ADMUX |= channelMuxTable[calSensor];
                }
                else
                {
                    ADMUX |= channelMuxTable[0];
                }
                toggleAllLEDs();

            }
        }
        else if(calCheck == CAL_AVG_TOO_HIGH) // Avg too high
        {
            dacVal = getDAC();
            if(dacVal < 612)
            {
                dacVal++;
                setDAC(dacVal);
                toggleRow2LED();
                if(calLastStep != 1)
                {
                    calLastStep = 1;
                    calRetryCount++;
                }
                else
                {
                    calRetryCount = 0;
                }
            }
            else
            {
                // give up...
                calFail = 1;
                calFailRow = calRow;
                calFailSensor = calSensor;

                calSensor++;
                setDAC(512);
                ADMUX &= ~(_BV(MUX0) | _BV(MUX1) | _BV(MUX2) | _BV(MUX3) | _BV(MUX4));
                if(calSensor < NUM_SENSORS_PER_ROW)
                {
                    ADMUX |= channelMuxTable[calSensor];
                }
                else
                {
                    ADMUX |= channelMuxTable[0];
                }
                toggleAllLEDs();
            }
        }
        clearADCData();

        if(calSensor >= NUM_SENSORS_PER_ROW)
        {
            calSensor = 0;
            calRow++;
            setNextSensorRow();

            if(calRow >= NUM_SENSOR_ROWS)
            {
                runCalibrationFlag = 0;
                calInitDone = 0;
                turnOffAllLEDs();

                // Send cal status message here...
                sendCalStatusMsg(calFail, calFailRow, calFailSensor);
                    
                // Reload the cal data from EEPROM so it is immediately active
                readCalValues();
            }
        }
        channelIndex = calSensor;
        sensorSampleCount = 0;
        calRowCount = 0;

        waitCycle = 1;

        _SEI();
     
    }
}

// Scans the averaged sensor data and determines
//   if the measured voltage falls within the specified window.
static BYTE checkCalVals(BYTE row, BYTE sensor)
{
    WORD sensorAvg;
    BYTE successFlag = 0;

    sensorAvg = (accumulatedADCData[row][sensor]) >> 4;

    if(sensorAvg > 508)
    {
        successFlag = 1; // Too high
        
        if(sensorAvg < 516)
        {
            successFlag = 2; // In range
        }        
    }
    else
    {
        successFlag = 0; //Too low
    }
    sensorBaseline[row][sensor] = sensorAvg;
    
    return successFlag;
}


// Interrupt routine called when ADC conversion is complete
ISR(ADC_vect)
{
    WORD adcValue = 0;
    
    //read the ADC (ADCL and ADCH in that order) in appropriate array element 
    adcValue = (WORD)ADCL;
    adcValue |= ((WORD)ADCH)<<8; 
        
    // activeRow will be in range 1-4...store new data in the array
    accumulatedADCData[activeRow - 1][channelIndex] += adcValue;           

    sensorSampleCount++;

    if(runCalibrationFlag)
    {
        if(sensorSampleCount < 16)
        {
            // do another sample of this sensor
            ADCSRA |= _BV(ADSC);
        }
    }
    else
    {
        if(sensorSampleCount > 3)
        {
            channelIndex++;

            if (channelIndex < ADC_LAST_CHANNEL)
            {
                // Clear the Mux values
                ADMUX &= ~(_BV(MUX0) | _BV(MUX1) | _BV(MUX2) | _BV(MUX3) | _BV(MUX4));
        
                // Select the next channel for conversion
                ADMUX |= channelMuxTable[channelIndex];
        
                sampleCount++;
            }
            else
            {
                ADMUX &= ~(_BV(MUX0) | _BV(MUX1) | _BV(MUX2) | _BV(MUX3) | _BV(MUX4));
                ADMUX |= channelMuxTable[0];
                channelIndex = 0;

                if(activeRow == 4) // All sensors are ready for averaging
                {
                    sensorsReadyFlag = 1;
                }
                setNextSensorRow();
            }

            sensorSampleCount = 0;
            // set dac
            setNextDAC();			
        }
        else // do another sample of this sensor
        {
            ADCSRA |= _BV(ADSC);
        }
    }	
}


// Clears the summed and averaged ADC data
void clearADCData(void)
{
    BYTE i, j;
    for(i = 0; i < NUM_SENSOR_ROWS; i++)
    {    
        for (j = 0; j < ADC_LAST_CHANNEL; j++)
        {
            //rawADCRow1[i] = 0;
            accumulatedADCData[i][j] = 0;
            avgADCData[i][j] = 0;
                      
        }   
    }
    
    sampleCount = 0;
}

// Calculates the average of a 4-sample burst for each sensor
void averageADCData(void)
{
    BYTE i, j;
    if((sampleCount != 0) && (!runCalibrationFlag))
    {
        for(i = 0; i < NUM_SENSOR_ROWS; i++)
        {
            for (j = 0; j < NUM_SENSORS_PER_ROW; j++)
            {
                avgADCData[i][j] = (accumulatedADCData[i][j] >> 2);
                accumulatedADCData[i][j] = 0;                                   
            }
        }
        
        sampleCount = 0;        
    }    
    
}

// Sets the DAC output to the calibration value for the next sensor per the sampling sequence.
// Assumes 'activeRow' is within the range 1-4.
static void setNextDAC(void)
{
    BYTE tempSensor;

    tempSensor = channelIndex;
    
    if(channelIndex >= ADC_LAST_CHANNEL)
    {
        tempSensor = 0;
    }
    
    
    setDAC(sensorCalVals[activeRow - 1][tempSensor]);
}


// Changes the switching I/O to select the next sensor row for sampling
void setNextSensorRow(void)
{
    activeRow++;
    
    // Roll over the row count if necessary
    if(activeRow > 4)
    {
        activeRow = 1;
    }
    
    // Set GPIO to activate the appropriate row
    if(activeRow == 1)
    {
        selectRow1();
    }
    else if(activeRow == 2)
    {
        selectRow2();
    }
    else if(activeRow == 3)
    {
        selectRow3();
    }
    else if(activeRow == 4)
    {
        selectRow4();   
    }    
}

// Retrieve a pointer to the averaged ADC data
WORD * getAveragedADCDataPtr(void)
{
    return &avgADCData[0][0];
}

// Retrieve a pointer to the accumulated ADC data
WORD * getAccumulatedADCDataPtr(void)
{
    return &accumulatedADCData[0][0];
}

// Set Row1 output high, clear the others
void selectRow1(void)
{
    PORTB &= 0x7E; // Clear Bit0 and Bit7
    PORTC &= ~(_BV(PC0));
    PORTD &= 0xFE; // Clear Bit0
    PORTB |= 0x80; // Set Bit7
}

// Set Row2 output high, clear the others
void selectRow2(void)
{
    PORTB &= 0x7E; // Clear Bit0 and Bit7
    PORTC &= 0xFE; // Clear Bit0
    PORTD |= 0x01; // Set Bit0
  
}

// Set Row3 output high, clear the others
void selectRow3(void)
{
    PORTB &= 0x7E; // Clear Bit0 and Bit7
    PORTD &= 0xFE; // Clear Bit0
    PORTC |= 0x01; // Set Bit0
   
}

// Set Row4 output high, clear the others
void selectRow4(void)
{
    PORTB &= 0x7E; // Clear Bit0 and Bit7
    PORTC &= 0xFE; // Clear Bit0
    PORTD &= 0xFE; // Clear Bit0
    PORTB |= 0x01; // Set Bit0 
}

// Kicks off the calibration process
void startSensorCalibration(void)
{
    runCalibrationFlag = 1;
    calRowCount = 0;
}

BYTE getSensorsReadyFlag(void)
{
    return sensorsReadyFlag;
}

void clearSensorsReadyFlag(void)
{
    sensorsReadyFlag = 0;
}

BYTE getRunCalibrationFlag(void)
{
    return runCalibrationFlag;
}

// Returns the most recent calibration results data
void getCalStatusData(BYTE* status, BYTE* calRow, BYTE* calSensor)
{
    *status = calFail;
    *calRow = calFailRow;
    *calSensor = calFailSensor;
}




