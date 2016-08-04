
/**
  TMR2 Generated Driver API Source File 

  @Company
    Microchip Technology Inc.

  @File Name
    tmr2.c

  @Summary
    This is the generated source file for the TMR2 driver using MPLAB� Code Configurator

  @Description
    This source file provides APIs for driver for TMR2. 
    Generation Information : 
        Product Revision  :  MPLAB� Code Configurator - v2.25.2
        Device            :  PIC24FV32KA304
        Driver Version    :  0.5
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.24
        MPLAB 	          :  MPLAB X v2.35 or v3.00
 */

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */

/**
  Section: Included Files
 */

#include <xc.h>
#include "tmr2.h"

/**
  Section: Data Type Definitions
 */

/** TMR Driver Hardware Instance Object

  @Summary
    Defines the object required for the maintainence of the hardware instance.

  @Description
    This defines the object required for the maintainence of the hardware
    instance. This object exists once per hardware instance of the peripheral.

  Remarks:
    None.
 */

typedef struct _TMR_OBJ_STRUCT {
    /* Timer Elapsed */
    bool timerElapsed;
    /*Software Counter value*/
    uint8_t count;

} TMR_OBJ;

static TMR_OBJ tmr2_obj;

/**
  Section: Driver Interface
 */


void TMR2_Initialize(void) {
    //TSIDL disabled; TGATE disabled; TCS FOSC/2; TCKPS 1:64; T32 disabled; TON enabled; 
    T2CON = 0x8020;
    //TMR2 0; 
    TMR2 = 0x0000;
    //Period Value = 1.000 s; PR2 31250; 
    PR2 = 0x7A12;


    tmr2_obj.timerElapsed = false;

}

/**
    void DRV_TMR2_Initialize (void)
 */
void DRV_TMR2_Initialize(void) {
    TMR2_Initialize();
}

void TMR2_Tasks_16BitOperation(void) {
    /* Check if the Timer Interrupt/Status is set */
    if (IFS0bits.T2IF) {
        tmr2_obj.count++;
        tmr2_obj.timerElapsed = true;
        IFS0bits.T2IF = false;
    }
}

/**
    void DRV_TMR2_Tasks_16BitOperation (void)
 */
void DRV_TMR2_Tasks_16BitOperation(void) {
    TMR2_Tasks_16BitOperation();
}

void TMR2_Period16BitSet(uint16_t value) {
    /* Update the counter values */
    PR2 = value;
    /* Reset the status information */
    tmr2_obj.timerElapsed = false;
}

/**
    void DRV_TMR2_Period16BitSet (uint16_t value)
 */
void DRV_TMR2_Period16BitSet(uint16_t value) {
    TMR2_Period16BitSet(value);
}

uint16_t TMR2_Period16BitGet(void) {
    return ( PR2);
}

/**
    uint16_t DRV_TMR2_Period16BitGet (void)
 */
uint16_t DRV_TMR2_Period16BitGet(void) {
    return (TMR2_Period16BitGet());
}

void TMR2_Counter16BitSet(uint16_t value) {
    /* Update the counter values */
    TMR2 = value;
    /* Reset the status information */
    tmr2_obj.timerElapsed = false;
}

/**
    void DRV_TMR2_Counter16BitSet (uint16_t value)
 */
void DRV_TMR2_Counter16BitSet(uint16_t value) {
    TMR2_Counter16BitSet(value);
}

uint16_t TMR2_Counter16BitGet(void) {
    return ( TMR2);
}

/**
    uint16_t DRV_TMR2_Counter16BitGet (void)
 */
uint16_t DRV_TMR2_Counter16BitGet(void) {
    return (TMR2_Counter16BitGet());
}

void TMR2_Start(void) {
    /* Reset the status information */
    tmr2_obj.timerElapsed = false;


    /* Start the Timer */
    T2CONbits.TON = 1;
}

/**
    void DRV_TMR2_Start (void)
 */
void DRV_TMR2_Start(void) {
    TMR2_Start();
}

void TMR2_Stop(void) {
    /* Stop the Timer */
    T2CONbits.TON = false;

}

/**
    void DRV_TMR2_Stop (void)
 */
void DRV_TMR2_Stop(void) {
    TMR2_Stop();
}

bool TMR2_GetElapsedThenClear(void) {
    bool status;

    status = tmr2_obj.timerElapsed;

    if (status == true) {
        tmr2_obj.timerElapsed = false;
    }
    return status;
}

/**
    bool DRV_TMR2_GetElapsedThenClear (void)
 */
bool DRV_TMR2_GetElapsedThenClear(void) {
    return (TMR2_GetElapsedThenClear());
}

uint8_t TMR2_SoftwareCounterGet(void) {
    return tmr2_obj.count;
}

/**
    uint8_t DRV_TMR2_SoftwareCounterGet (void)
 */
uint8_t DRV_TMR2_SoftwareCounterGet(void) {
    return (TMR2_SoftwareCounterGet());
}

void TMR2_SoftwareCounterClear(void) {
    tmr2_obj.count = 0;
}

/**
    void DRV_TMR2_SoftwareCounterClear (void)
 */
void DRV_TMR2_SoftwareCounterClear(void) {
    TMR2_SoftwareCounterClear();
}

/**
 End of File
 */
