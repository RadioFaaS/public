/*******************************************************************************
 * Copyright 2012 Appareo Systems
 * All rights reserved
 * This software and/or material is the property of Appareo Systems.
 * All use, disclosure, and/or reproduction not specifically authorized in
 * writing by Appareo Systems is prohibited.
 *
 * PROJECT:       Navigation Suite
 *
 * FILE NAME:    Ports.h
 *
 * DESCRIPTION:   Micro Port pins configuration.
 *
 * REVISION HISTORY:
 *
 02/06/2006   VS  1. Added macro to know if set reset pin is High.
 05/30/2006   VS  1. Updated macros for card detection and write protect on SD card
 * $Log:$
*******************************************************************************/

/*==============================================================================
   COMPILER DIRECTIVES
 *============================================================================*/
#ifndef _PORTS_CONFIG_H_
#define _PORTS_CONFIG_H_

/*==============================================================================
   INCLUDE FILES
 *============================================================================*/

#include <avr/io.h>

/*==============================================================================
   DEFINES AND ENUMERATIONS
 *============================================================================*/

// For all ports DDR logic 1 => output, logic 0 => input

//PORT B DATA DIRECTION:
//Input/Output PB0: PCINT0/MISO/PSCOUT2A                (Row4Ctrl)
//Input/Output PB1: PCINT1/MOSI/PSCOUT2B                (LED_ROW4)
//Input/Output PB2: ADC5/INT1/ACMPN0/PCINT2             (AIN5)
//Input/Output PB3: AMP0-/PCINT3                        (LED_ROW2)
//Input/Output PB4: AMP0+/PCINT4                        (LED_ROW1)
//Input/Output PB5: ADC6/INT2/ACMPN1/AMP2-/PCINT5       (AIN6)
//Input/Output PB6: ADC7/PSCOUT1B/PCINT6                (AIN7)
//Input/Output PB7: ADC4/PSCOUT0B/SCK/PCINT7            (Row1Ctrl)

#define CONFIG_PORTB_DDR  DDRB |= (_BV(PB0) | _BV(PB1) | _BV(PB3) | _BV(PB4) | _BV(PB7))

#define TOGGLE_LED	    PORTB ^= (_BV(PB4))

#define TURN_LED_ON     PORTB |= (_BV(PB4))
#define TURN_LED_OFF    PORTB &= ~(_BV(PB4))

#define TOGGLE_LED_1    PORTB ^= (_BV(PB1)) 
#define TOGGLE_LED_2    PORTC ^= (_BV(PC6))
#define TOGGLE_LED_3    PORTB ^= (_BV(PB4))
#define TOGGLE_LED_4    PORTB ^= (_BV(PB3))


//PORT B pin access macros:


//PORT C DATA DIRECTION:
//Input/Output PC0: PCINT8/INT3/PSCOUT1A        (Row3Ctrl)
//Input/Output PC1: PCINT9/PSCIN1/OC1B/SS_A     (WiFi Slave Select)
//Input/Output PC2: PCINT10/T0/TXCAN            (TXCAN)
//Input/Output PC3: PCINT11/T1/RXCAN/ICP1B      (RXCAN)
//Input/Output PC4: ADC8/ACMPN3/AMP1-/PCINT12   (AIN4)
//Input/Output PC5: ADC9/ACMP3/AMP1+/PCINT13    (AIN1)
//Input/Output PC6: ADC10/ACMP1/PCINT14         (LED_ROW3)
//Input/Output PC7: D2A/AMP2+/PCINT15           (2_5V_OUT)

#define CONFIG_PORTC_DDR DDRC |=  (_BV(PC0) | _BV(PC1) | _BV(PC2) | _BV(PC3) | _BV(PC6) | _BV(PC7))
          
                           
//Port C pin access macros
#define SET_WIFI_CS_LOW     PORTC &= ~(_BV(PC1))
#define SET_WIFI_CS_HIGH    PORTC |= (_BV(PC1))


//PORT D DATA DIRECTION:
//Input/Output PD0: PCINT16/PSCOUT0A                    (Row2Ctrl)
//Input/Output PD1: PCINT17/PSCIN0/CLK0                 (ResetWiFi)
//Input/Output PD2: PCINT18/PSCIN2/OC1A/MISO_A          (WiFi MISO)
//Input/Output PD3: PCINT19/TXD/TXLIN/OC0A/SS/MOSI_A    (WiFi MOSI)
//Input/Output PD4: PCINT20/ADC1/RXD/RXLIN/ICP1A/SCK_A  (WiFi SPI Clock)
//Input/Output PD5: ADC2/ACMP2/PCINT21                  (AIN2)
//Input/Output PD6: ADC3/ACMPN2/INT0/PCINT22            (AIN3)
//Input/Output PD7: ACMP0/PCINT23                       (WiFi Host Wake-Up)

#define CONFIG_PORTD_DDR DDRD |= (_BV(PD0) | _BV(PD1) | _BV(PD3) | _BV(PD4))

//Port D pin access macros
#define IS_WIFI_WAKEUP_LOW      ((PORTD & 0x80) == 0)
#define IS_WIFI_WAKEUP_HIGH     ((PORTD & 0x80) != 0)


//PORT E DATA DIRECTION:
//Input/Output PE0: PCINT24/RESET/OCD
//Input/Output PE1: PCINT25/OC0B/XTAL1
//Input/Output PE2: PCINT26/ADC0/XTAL2

#define CONFIG_PORTE_DDR                DDRE = 0 

//Port E pin access macros



 /*==============================================================================
   TYPEDEFS
 *============================================================================*/

/*==============================================================================
   EXTERN GLOBAL VARIABLES
 *============================================================================*/


/*==============================================================================
   GLOBAL FUNCTIONS
 *============================================================================*/
#endif  // _PORTS_CONFIG_H_

/*******************************************************************************
                                End of File
*******************************************************************************/