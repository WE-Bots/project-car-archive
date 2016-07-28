/*
 * File:   externInterrupt.c
 * Author: Jacob
 *
 * Created on July 16, 2016, 4:01 AM
 */


#include "externInterrupt.h"
#include <p33Exxxx.h>

void externInterrupt_init(void)
{
    //remap pins
    TRISDbits.TRISD8 = 1;
    RPINR0bits.INT1R = 0b1001000;
    
    INTCON2bits.INT1EP = 0;    //triggers on rising edge 
    IPC5bits.INT1IP = 5;
    IFS1bits.INT1IF = 0;
    IEC1bits.INT1IE = 0;
}

void __attribute__ ((__interrupt__, no_auto_psv)) _INT1Interrupt(void)
{
   IFS1bits.INT1IF = 0; 
}