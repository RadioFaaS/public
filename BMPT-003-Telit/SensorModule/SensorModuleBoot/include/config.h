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
//#include <avr/signal.h>
#include <compat/ina90.h>
#include <avr/wdt.h>
#include "ports.h"
#include "Types.h"



//_____ M A C R O S ____________________________________________________________

//_____ D E F I N I T I O N S _______________________________________________

    // -------------- MCU LIB CONFIGURATION
#define FOSC           16000        // 16 MHz External crystal
#define F_CPU          (FOSC*1000) // Need for AVR GCC

    // -------------- CAN LIB CONFIGURATION
#define CAN_BAUDRATE   125        // in kBit
//#define CAN_BAUDRATE   CAN_AUTOBAUD

#define CAN_FILTER_ENABLED

//#define _MULE_BOARD

//_____ D E C L A R A T I O N S ________________________________________________

//______________________________________________________________________________

#endif  // _CONFIG_H


