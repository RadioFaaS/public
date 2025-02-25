//******************************************************************************
//! @file file: config.h
//!
//******************************************************************************

#ifndef _CONFIG_H_
#define _CONFIG_H_

//_____ I N C L U D E S ________________________________________________________


#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <compat/ina90.h>
#include <avr/wdt.h>
#include "ports.h"
#include "Types.h"



//_____ M A C R O S ____________________________________________________________

//_____ D E F I N I T I O N S _______________________________________________

// -------------- MCU LIB CONFIGURATION
#define FOSC            16000        // 16 MHz External crystal
#define F_CPU          (FOSC*1000)   // Need for AVR GCC

// -------------- CAN LIB CONFIGURATION
#define CAN_BAUDRATE   125           // in kBit

//#define _MULE_BOARD
#define _REV_B_HW

#define CAN_FILTER_ENABLED
#define MCU_64M1
//#define CPU_FREQ_16MHZ

//#define _USE_NOISE_CALC
//#define _ENABLE_WIFI_DEBUG_DATA

enum
{
    HW_TYPE_WIRED = 0,
    HW_TYPE_WIRELESS    
};

#define NUM_SENSOR_ROWS         4
#define NUM_SENSORS_PER_ROW     7



//______________________________________________________________________________

#endif  // _CONFIG_H


