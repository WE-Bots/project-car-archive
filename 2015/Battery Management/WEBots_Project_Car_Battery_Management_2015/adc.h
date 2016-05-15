/************************************************************************
*                                                                       *
*   Filename:       adc.h                                               *
*   Date:           2015/06/24                                          *
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
*   Description:    This file contains functions to interface with the  *
*                   ADC module                                          *
*                                                                       *
*                   Ensure that the appropriate TRIS and ANSEL          *
*                   resiters have been set                              *
*                                                                       *
*************************************************************************
*                                                                       *
*   Pin assignments:                                                    *
*                                                                       *
************************************************************************/

#ifndef ADC_H
#define	ADC_H

typedef enum
{
    CELL1 = 0b000,
    CELL2 = 0b001,
    REFV = 0b010,
    CELL5 = 0b011,
    CELL6 = 0b100,
    CELL3 = 0b101,
    CELL4 = 0b110,
    CURRENT = 0b111,

}ADCChannel;

// Function Definition
void initADC();
unsigned int analogRead( uint8_t chan );
unsigned int analogReadFVR();


// Function Declaration
void initADC()
{
    ADCON0 = 0b10000001;
    //         1_______ - data right justified, LSB in ADRESL.0
    //         _0______ - negative reference is VSS
    //         __0_____ - positive reference is VDD
    //         ___xxx__ - channel select bits
    //         _______1 - enable ADC

    ADCON1 = 0b01110000;
    //         _111____ - set the clock source to be the dedicated Frc clock

    ADRESH = 0x00;
    ADRESL = 0x00;

}

unsigned int analogRead( uint8_t  chan )
{
    ADCON0bits.CHS = chan;

    ADCON0bits.ADON = 1; // turn on the ADC module

    __delay_us(10); // wait for the ADC to get ready

    ADCON0bits.GO = 1; // start the conversion

    unsigned int returnVal = 0; // value to be returned after the confersion has been complete

    while (ADCON0bits.GO == 1) {} // wait for the conversion to complete, flag will be cleared when finished

    // conversion has been completed, transfer the data to returnVal

    returnVal = ADRESH; // put the high order byted into the low order bits of the return value
    returnVal = returnVal << 8; // push the high order bits into the high order bits of the return value
    returnVal = returnVal | ADRESL; // put the low order data into the low order bytes of return value

    return returnVal;
}


#endif	/* ADC_H */

