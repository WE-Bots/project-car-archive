/************************************************************************
*                                                                       *
*   Filename:       timer0.h                                            *
*   Date:           27/06/2015                                          *
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
*   Description:    This file contains functions to interface with      *
*                   timer 0 to be used as a time reference              *
*************************************************************************
*                                                                       *
*   Pin assignments:                                                    *
*                   Ensure that nothing else is using timer0            *
*                                                                       *
************************************************************************/

#ifndef TIMER0_H
#define	TIMER0_H

#include <stdbool.h>

unsigned long time = 0;
// ensure that this value is accurate for the clock being used
//NOTE: convConst = 1000 * 256 * 4 / Fosc
const float convConst = 0.128; // constant to convert from ticks to seconds

// Function Declarations

void timeSetup();
void isrTime();
unsigned long currentTime();
unsigned long stopWatch (bool);

// Function Definitions

void timeSetup()
{
    OPTION_REGbits.T0CS = 0; // Clock source i2 the internal instruction clock Fosc/4
    OPTION_REGbits.T0SE = 1; // Increment from high to low
    OPTION_REGbits.PSA = 0 ;// assign a prescaller to Timer 0
    OPTION_REGbits.PS = 0b111; // use 1:256 prescaller

    // setup the overflow interrupt
    INTCONbits.T0IE = 1; // enable overflow interrupt
    INTCONbits.T0IF = 0; // reset flag bit
    INTCONbits.PEIE = 1; // enable peripherial interrrupts
    INTCONbits.GIE = 1; // global interrupt enable

}

void isrTimer0 ()
{
    if (INTCONbits.T0IF == 1 && INTCONbits.T0IE == 1)
    {
        // *** Service Timer0 overflow interrupt

        time = time + 255; // update the time

        INTCONbits.T0IF = 0; // reset the interrupt flag
    }
}

unsigned long currentTime ()
{
    return convConst * ( time + TMR0 );// return the updated time plus the the time currently running in the timer 0 register
}

unsigned long stopWatch (bool mode)
{
    static unsigned long startTime = 0;

    unsigned long temp = 0;

    switch (mode)
    {
        case(0): // start / reset stopwatch
        {
            startTime = time + TMR0; // update the previous time to the current time

            return 1;
        }

        case (1): // peek at the time on the stopwatch
        {
            temp =  time + TMR0 - startTime;

            temp = convConst * temp;

            return temp;
        }
    }

    return 0;
}

#endif	/* TIMER0_H */

