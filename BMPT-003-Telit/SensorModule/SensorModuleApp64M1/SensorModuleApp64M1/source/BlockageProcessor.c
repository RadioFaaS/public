
#include "BlockageProcessor.h"
#include "ADC.h"
#include "CAN.h"
#include "SensorModule.h"
#include "NValloc.h"
#include "TaskList.h"
#include "ASIP.h"
#include <math.h>

#define MAGNET_NULL_DEADBAND    30          // Minimum signal for individual row data reporting
#define ADC_MAX                 0x3FF
#define ADC_OOR_DEADBAND        10
#define NOISE_NO_FLOW_THRESHOLD 1//10       // Noise threshold for sensor-wide zero-stuffing
#define INITIAL_MAGNET_DETECT_THRESHOLD 55  // Minimum signal required to initially activate a row

static BYTE rowLevel[NUM_SENSOR_ROWS]; // Stores the current ball level for each of the 4 rows
WORD sensorCalVals[NUM_SENSOR_ROWS][NUM_SENSORS_PER_ROW]; // Quiescent ADC readings for all 28 sensors [row][sensor]
WORD sensorBaseline[NUM_SENSOR_ROWS][NUM_SENSORS_PER_ROW];


static WORD dataHistory[28];
static int lastNoiseCalc[4];

BYTE nullCount[4];

static BYTE noMagnetRowMask;
static BYTE sensorOORFlag;
static BYTE flowDetectedFlag;

static BYTE noMagDetectionDone = 0;
static BYTE runNoMagnetDetection = 0;

static BYTE noMagPersistenceCount[4] = {0, 0, 0, 0};

static BYTE sensorDataDisableFlag = 0;
static BYTE enableDebugSensorData = 0;
static int calcEuclideanDistance(BYTE rowNumber);


void initBlockageProcessor(void)
{
    BYTE i;
    
    rowLevel[0] = 0;
    rowLevel[1] = 0;
    rowLevel[2] = 0;
    rowLevel[3] = 0;

    noMagnetRowMask = 0x0F;
    sensorOORFlag = 0;
    flowDetectedFlag = 0;

    for(i = 0; i < NUM_SENSOR_ROWS; i++)
    {
        nullCount[i] = 0;
    }

    for(i = 0; i < 28; i++)
    {
        dataHistory[i] = 0;
    }

    for(i = 0; i < 4; i++)
    {
        lastNoiseCalc[i] = 0;
    }
}

// Reads and stores the cal data from EEPROM
void readCalValues(void)
{
    BYTE i;
    BYTE j;
    WORD tempVal;
    
    for(i = 0; i < NUM_SENSOR_ROWS; i++)
    {
        for(j = 0; j < NUM_SENSORS_PER_ROW; j++)
        {
            // Load DAC settings
            eeprom_busy_wait();
            tempVal = ((WORD)eeprom_read_byte((BYTE *)(EE_SENSOR_CAL_LOC + (i * 14) + (j * 2))));
            eeprom_busy_wait();
            tempVal |= (((WORD)eeprom_read_byte((BYTE *)(EE_SENSOR_CAL_LOC + (i * 14) + (j * 2) + 1))) << 8);
            
            // Check for bad cal...
            if(tempVal == 0xFFFF)
            {
                tempVal = 512;
            }
            sensorCalVals[i][j] = tempVal;

            // Load sensor baseline (DC bias) values, as determined with calibrated DAC settings
            eeprom_busy_wait();
            tempVal = ((WORD)eeprom_read_byte((BYTE *)(EE_SENSOR_BASE_LOC + (i * 14) + (j * 2))));
            eeprom_busy_wait();
            tempVal |= (((WORD)eeprom_read_byte((BYTE *)(EE_SENSOR_BASE_LOC + (i * 14) + (j * 2) + 1))) << 8);
            
            // Check for bad cal...
            if(tempVal == 0xFFFF)
            {
                tempVal = 512;
            }
            sensorBaseline[i][j] = tempVal;

        }
    }
}


// Takes the averaged raw ADC samples and determines the absolute difference 
//  from each sensor's calibration baseline.
void applyCalibrationToData(void)
{
    BYTE i, j;
    WORD * dataPtr;
    WORD tempWord;

    dataPtr = getAveragedADCDataPtr();
    for(i = 0; i < NUM_SENSOR_ROWS; i++)
    {
        for (j = 0; j < NUM_SENSORS_PER_ROW; j++)
        {
            tempWord = *(dataPtr + (i * NUM_SENSORS_PER_ROW) + j);
            tempWord -= sensorBaseline[i][j];
            
            // take abs() if result is negative
            if(tempWord & 0x8000)
            {
                tempWord = ~(tempWord) + 1;
            }
           
            *(dataPtr + (i * NUM_SENSORS_PER_ROW) + j) = tempWord;
        }
    }
}

// Determines the sensor with the greatest signal strength in each row
//  and transmits the data to the display.
void processBlockageData(void)
{
    int noiseCalculations[4];
#ifdef _USE_NOISE_CALC
    noiseCalculations[0] = calcEuclideanDistance(0);
    noiseCalculations[1] = calcEuclideanDistance(1);
    noiseCalculations[2] = calcEuclideanDistance(2);
    noiseCalculations[3] = calcEuclideanDistance(3);

    if((noiseCalculations[0] < NOISE_NO_FLOW_THRESHOLD) && (noiseCalculations[1] < NOISE_NO_FLOW_THRESHOLD) 
        && (noiseCalculations[2] < NOISE_NO_FLOW_THRESHOLD) &&(noiseCalculations[3] < NOISE_NO_FLOW_THRESHOLD))
    {
        // Force a position of zero, but check for unused rows
        rowLevel[0] = 0;
        rowLevel[1] = 0;
        rowLevel[2] = 0;
        rowLevel[3] = 0;

        if(noMagnetRowMask & 0x01)
        {
            rowLevel[0] = 0xFF;
        }
        if(noMagnetRowMask & 0x02)
        {
            rowLevel[1] = 0xFF;
        }
        if(noMagnetRowMask & 0x04)
        {
            rowLevel[2] = 0xFF;
        }
        if(noMagnetRowMask & 0x08)
        {
            rowLevel[3] = 0xFF;
        }
    }
    else
    {
#endif
        rowLevel[0] = getMaxSensorDataIndex(0);
        rowLevel[1] = getMaxSensorDataIndex(1);
        rowLevel[2] = getMaxSensorDataIndex(2);
        rowLevel[3] = getMaxSensorDataIndex(3);
#ifdef _USE_NOISE_CALC
    }
#endif

    if(enableDebugSensorData)
    {
        sendCANRawADC(getAveragedADCDataPtr() + 0); // Debug - send adc counts...
        sendCANRawADC2(getAveragedADCDataPtr() + 4);
        sendCANRawADC3(getAveragedADCDataPtr() + 8);
        sendCANRawADC4(getAveragedADCDataPtr() + 12);
        sendCANRawADC5(getAveragedADCDataPtr() + 16);
        sendCANRawADC6(getAveragedADCDataPtr() + 20);
        sendCANRawADC7(getAveragedADCDataPtr() + 24); 
    }   
	
	sendASIPSensorDataMsg(rowLevel); // Add data to WiFi Tx buffer
}


// Checks the most recent averaged sensor data for the max value.
// Returns the index (0-6) indicating which sensor sees the strongest magnetic field,
//   or 0xFF for rows with No Magnet Detected.
BYTE getMaxSensorDataIndex(BYTE rowNumber)
{
    WORD * dataPtr = NULL;
    WORD maxValue = 0;
    BYTE index = 0xFF;
    BYTE i;


    static BYTE noMagPeristence = 0;
    static BYTE noMagPersistenceRow = 0xFF;
    
    dataPtr = getAveragedADCDataPtr() + (rowNumber * NUM_SENSORS_PER_ROW);
              
    for(i = 0; i < NUM_SENSORS_PER_ROW; i++)
    {
            
        // Find the max signal
        if(*(dataPtr + i) > maxValue)
        {
            //Store the highest signal
            maxValue = *(dataPtr + i);
            index = i;
        }

    }
    if(runNoMagnetDetection)
    {
        // For each row check for the given minimum signal strength and minimum flow height (index at 2 or above)
            
        if((noMagnetRowMask & (0x01 << rowNumber)) && (maxValue > INITIAL_MAGNET_DETECT_THRESHOLD) && (index > 1))
        {
            noMagPersistenceCount[rowNumber]++;
                
            if(noMagPersistenceCount[rowNumber] > 2) // Signal must persist for at least 3 consecutive samples
            {
                noMagnetRowMask &= ~(0x01 << rowNumber);

            }
        }
        else
        {
            noMagPersistenceCount[rowNumber] = 0;
        }

        if(noMagnetRowMask == 0)
        {
            runNoMagnetDetection = 0;
            noMagDetectionDone = 1;
        }

    }
    else if((!noMagDetectionDone) && (maxValue > INITIAL_MAGNET_DETECT_THRESHOLD) && (index > 1))
    {
        // No Mag detection triggers when specified minimum flow and signal strength is detected

        // First sample meeting the criteria, remember which row to watch
        if(noMagPersistenceRow == 0xFF)
        {
            noMagPersistenceRow = rowNumber;
            noMagPeristence++;
        }
        else if(noMagPersistenceRow == rowNumber) // Repeated row number meeting criteria
        {
            noMagPeristence++;
        }

        if(noMagPeristence > 3)
        {
            runNoMagnetDetection = 1;
            flowDetectedFlag = 1;
        }
    }
    else if(noMagPersistenceRow == rowNumber) // Reset count if criteria not met
    {
        noMagPeristence = 0;
    }

    if((maxValue < (MAGNET_NULL_DEADBAND)) && !(noMagnetRowMask & (0x01 << rowNumber)))
    {
        // Repeat the last valid value
        nullCount[rowNumber]++;

        if(nullCount[rowNumber] > 12) // 3 seconds assuming function calls at 4Hz 
        {
            nullCount[rowNumber] = 12;
            return 0;
        }
        else
        {
            return rowLevel[rowNumber];
        }
    }
    else
    {
        nullCount[rowNumber] = 0;
    }

    if(noMagnetRowMask & (0x01 << rowNumber))
    {
        index = 0xFF;
    }

    return index;
}

// This function evaluates the overall variation between consecutive samples among the 4 rows of the sensor module.
// If this calculated value falls below the defined threshold the system will assume the pump is off and will force position data to index zero.
// Must be called once per complete sample cycle(??)
int calcEuclideanDistance(BYTE rowNumber)
{
    WORD * dataPtr = NULL;
    BYTE i;
    BYTE startingSensorNum;
    WORD dataPoint1;
    WORD dataPoint2;
    int sum = 0;
    signed short diff = 0;
    int noiseCalc;

    dataPtr = getAveragedADCDataPtr() + (rowNumber * NUM_SENSORS_PER_ROW);
    startingSensorNum = (rowNumber * NUM_SENSORS_PER_ROW);
            
    for(i = 0; i < NUM_SENSORS_PER_ROW; i++)
    {
        dataPoint1 = *(dataPtr + i);
        dataPoint2 = dataHistory[startingSensorNum + i];
        
        if(dataPoint2 > dataPoint1)
        {
            diff = dataPoint2 - dataPoint1;
        }
        else
        {
            diff = dataPoint1 - dataPoint2;
        }
        sum += (diff * diff);

    }
    
    // Copy the data for next iteration
    for(i = 0; i < NUM_SENSORS_PER_ROW; i++)
    {
        dataHistory[startingSensorNum + i] = *(dataPtr + i);
    }

    noiseCalc = sqrt(sum);

    if(noiseCalc == 0)
    {
        noiseCalc = lastNoiseCalc[rowNumber];
    }

    lastNoiseCalc[rowNumber] = noiseCalc;

    return noiseCalc;

}

BYTE getSensorOORFlag(void)
{
    return sensorOORFlag;
}

BYTE getNoMagnetRowMask(void)
{
    return noMagnetRowMask;
}


BYTE getNoMagDetectionDone(void)
{
    return noMagDetectionDone;
}

// Returns the flag indicating whether liquid flow 
//  has been detected since power-up
BYTE getFlowDetectedFlag(void)
{
    return flowDetectedFlag;
}

void setSensorDataDisableFlag(void)
{
    sensorDataDisableFlag = 1;
}

void clearSensorDataDisableFlag(void)
{
    sensorDataDisableFlag = 0;
}

void setEnableDebugFlag(void)
{
    enableDebugSensorData = 1;
}

void clearEnableDebugFlag(void)
{
    enableDebugSensorData = 0;
}

