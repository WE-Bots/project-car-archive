/************************************************************************
*                                                                       *
*   Filename:       setup.h                                             *
*   Date:           02/06/2015                                          *
*   File Version:   1.0                                                 *
*                                                                       *
*   Author:         Andrew Cullen                                       *
*                                                                       *
*************************************************************************
*                                                                       *
*   Architecture:   Mid-range PIC                                       *
*   Processor:      PIC16F917                                           *
*   Compiler:       MPLAB XC8 v1.00 (Free mode)                         *
*                                                                       *
*************************************************************************
*                                                                       *
*   Files required: none                                                *
*                                                                       *
*************************************************************************
*                                                                       *
*   Description:    This file contains declarations, definitions and    *
*                   seting up registers for the WEBots Project Car      *
*                   battery management program                          *
*                                                                       *
*************************************************************************
*                                                                       *
*   Pin assignments:                                                    *
*                   C1_MEAS_UF      - AN0                               *
*                   C2_MEAS_UF      - AN1                               *
*                   1.982V_REF_UF   - AN2                               *
*                   C5_MEAS_UF      - AN3                               *
*                   TEMP_TH         - RA4                               *
*                   C6_MEAS_UF      - AN4                               *
*                   C3_MEAS_UF      - AN5                               *
*                   C4_MEAS_UF      - AN6                               *
*                   CURRENT_MEAS    - AN7                               *
*                   CG_SEL1         - RA7                               *
*                   CG_SEL0         - RA6                               *
*                   MOTOR_CONTROL   - RC0                               *
*                   R_LED           - RC1                               *
*                   OPAMP_CGND      - RC2                               *
*                   TP_CLS_EN       - RC3                               *
*                   BT_CLS_EN       - RD0                               *
*                   REF_EN          - RD1                               *
*                   LCD_DB7         - RB5                               *
*                   LCD_DB6         - RB4                               *
*                   LCD_DB5         - RB3                               *
*                   LCD_DB4         - RB2                               *
*                   LCD_EN          - RB1                               *
*                   LCD_RS          - RB0                               *
*                   LCD_RW          - RD7                               *
*                   LCD_PWR_SW      - RD6                               *
*                   LCD_PB          - RD5                               *
*                   G_LED           - RD4                               *
*                   5V_REG1_EN      - RC5                               *
*                   5V_REG2_EN      - RC4                               *
*                   12V_REG_EN      - RD3                               *
*                   UC_CGND         - RD2                               *
*                                                                       *
************************************************************************/

#ifndef SETUP_H
#define	SETUP_H

/***** CONFIGURATION *****/
/*
WDTE =	Watchdog Timer Enable bit
    ON      WDT enabled
    OFF     WDT disabled and can be enabled by SWDTEN bit of the WDTCON register
PWRTE =	Power Up Timer Enable bit
    OFF     PWRT disabled
    ON      PWRT enabled
CP =	Code Protection bit
    OFF     Program memory code protection is disabled
    ON      Program memory code protection is enabled
BOREN =	Brown-out Reset Selection bits
    ON      BOR enabled
    OFF     BOR disabled
    NSLEEP  BOR enabled during operation and disabled in Sleep
    SBODEN  BOR controlled by SBOREN bit of the PCON register
DEBUG =	In-Circuit Debugger Mode bit
    OFF     In-Circuit Debugger disabled, RB6/ISCPCLK and RB7/ICSPDAT are general purpose I/O pins
    ON      In-Circuit Debugger enabled, RB6/ICSPCLK and RB7/ICSPDAT are dedicated to the debugger
FCMEN =	Fail-Safe Clock Monitor Enabled bit
    ON      Fail-Safe Clock Monitor is enabled
    OFF     Fail-Safe Clock Monitor is disabled
MCLRE =	RE3/MCLR pin function select bit
    ON      RE3/MCLR pin function is MCLR
    OFF     RE3/MCLR pin function is digital input, MCLR internally tied to VDD
CPD =	Data Code Protection bit
    OFF     Data memory code protection is disabled
    ON      Data memory code protection is enabled
IESO =	Internal External Switchover bit
    ON      Internal/External Switchover mode is enabled
    OFF     Internal/External Switchover mode is disabled
FOSC =	Oscillator Selection bits
    HS          HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT/T1OSO and RA7/OSC1/CLKIN/T1OSI
    INTOSCIO	INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT/T1OSO pin, I/O function on RA7/OSC1/CLKIN/T1OSI
    INTOSCCLK	INTOSC oscillator: CLKOUT function on RA6/OSC2/CLKOUT/T1OSO pin, I/O function on RA7/OSC1/CLKIN/T1OSI
    LP          LP oscillator: Low-power crystal on RA6/OSC2/CLKOUT/T1OSO and RA7/OSC1/CLKIN/T1OSI
    EXTRCIO	RCIO oscillator: I/O function on RA6/OSC2/CLKOUT/T1OSO pin, RC on RA7/OSC1/CLKIN/T1OSI
    EC          EC: I/O function on RA6/OSC2/CLKOUT/T1OSO pin, CLKIN on RA7/OSC1/CLKIN/T1OSI
    XT          XT oscillator: Crystal/resonator on RA6/OSC2/CLKOUT/T1OSO and RA7/OSC1/CLKIN/T1OSI
    EXTRCCLK	RC oscillator: CLKOUT function on RA6/OSC2/CLKOUT/T1OSO pin, RC on RA7/OSC1/CLKIN/T1OSI
 */

#pragma config WDTE = OFF, PWRTE = OFF, CP = OFF, BOREN = OFF, DEBUG = OFF, FCMEN = OFF, MCLRE = ON, CPD = OFF, IESO = ON, FOSC = INTOSCIO

#include <xc.h>
#include <stdint.h> // allows access to more intuiative interger classes such as uint8_t, int16_t

/***** LCD Definitions *****/

#define D7 RB5
#define D6 RB4
#define D5 RB3
#define D4 RB2
#define EN RB1
#define RS RB0
#define RW RD7
#define LCD_PWR_SW RD6
#define LCD_PB RD5

#define _XTAL_FREQ 4000000  // used for _delay function, specifies a clock frequency of 4MHz

#include "lcd.h" // header file to interface with 16x2 LCD screen

/***** Cell Measurement Definitions *****/

#define REF AN2
#define C1_MEAS AN0
#define C2_MEAS AN1
#define C3_MEAS AN5
#define C4_MEAS AN6
#define C5_MEAS AN3
#define C6_MEAS AN4
#define REF_EN RD1
#define TP_CLS_EN RC3
#define BT_CLS_EN RD0
#define OPAMP_CGND RC2 

/***** Current Measurement Definitions *****/

#define CURRENT_MEAS AN7
#define CG_SEL1 RA7
#define CG_SEL0 RA6

/***** Shutoff/Enable Definitions *****/

#define UC_CGND RD2
#define MOTOR_CONTROL RC0
#define REG1_5V_EN RC5
#define REG2_5V_EN RC4
#define REG_12V_EN RD3
#define TEMP_TH RA4

/***** LED Definitions *****/

#define R_LED RC1
#define G_LED RD4

/***** Variables *****/

float cellVolt[6]; // array to hold the voltages of each cell
float supVolt = 4.2; // supply voltage to be set by the sampleReference function

uint16_t refValue = 0; // ADC value of the reference voltage

float current = 0; // current flowing out of the battery
float shuntRes = 0.01; // the resistance of the current shunt in ohms
uint8_t currentGain = 200; // gain for the current sense module

/***** Calibration Variables *****/

const float refVolt = 1.982; // calibration value for the reference voltage

const float cell1RDT = 2200; // the value of the resistor at the top of the cell 1 resistor divider (OHM)
const float cell1RDB = 2200; // the value of the resistor at the bottom of the cell 1 resistor divider (OHM)

const float cell2RDT = 4700; // the value of the resistor at the top of the cell 2 resistor divider (OHM)
const float cell2RDB = 2200; // the value of the resistor at the bottom of the cell 2 resistor divider (OHM)

const float cell3RDT = 10000; // the value of the resistor at the top of the cell 3 resistor divider (OHM)
const float cell3RDB = 2200; // the value of the resistor at the bottom of the cell 3 resistor divider (OHM)

const float cell4RDT = 14700; // the value of the resistor at the top of the cell 4 resistor divider (OHM)
const float cell4RDB = 2200; // the value of the resistor at the bottom of the cell 4 resistor divider (OHM)

const float cell5RDT = 20000; // the value of the resistor at the top of the cell 5 resistor divider (OHM)
const float cell5RDB = 2200; // the value of the resistor at the bottom of the cell 5 resistor divider (OHM)

const float cell6RDT = 24700; // the value of the resistor at the top of the cell 6 resistor divider (OHM)
const float cell6RDB = 2200; // the value of the resistor at the bottom of the cell 6 resistor divider (OHM)


/***** Functions *****/

float sampleADC ( uint8_t );
void sampleReference();
void sampleBatteryCells ();
void displayLCD ( uint8_t );
float sampleCurrent ();
void currentInit ( uint8_t );




#endif	/* SETUP_H */