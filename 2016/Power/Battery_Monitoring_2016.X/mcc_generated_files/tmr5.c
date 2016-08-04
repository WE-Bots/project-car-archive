
/**
  TMR5 Generated Driver API Source File 

  @Company
    Microchip Technology Inc.

  @File Name
    tmr5.c

  @Summary
    This is the generated source file for the TMR5 driver using MPLAB® Code Configurator

  @Description
    This source file provides APIs for driver for TMR5. 
    Generation Information : 
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
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
#include "tmr5.h"
#include "debug_LED.h"

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

static TMR_OBJ tmr5_obj;

/**
  Section: Driver Interface
 */


void TMR5_Initialize(void) {
    //TSIDL disabled; TGATE disabled; TCS FOSC/2; TCKPS 1:1; TON enabled; 
    T5CON = 0x8000;
    //TMR5 0; 
    TMR5 = 0x0000;
    //Period Value = 100.000 ms; PR5 25000; 
    PR5 = 0x61A8;

    IFS1bits.T5IF = false;
    IEC1bits.T5IE = true;

    tmr5_obj.timerElapsed = false;

}

/**
    void DRV_TMR5_Initialize (void)
 */
void DRV_TMR5_Initialize(void) {
    TMR5_Initialize();
}

void __attribute__((interrupt, no_auto_psv)) _T5Interrupt() {
    /* Check if the Timer Interrupt/Status is set */

    //***User Area Begin
    static volatile unsigned int CountCallBack = 0;

    // callback function - called every 2th pass
    if (++CountCallBack >= TMR5_INTERRUPT_TICKER_FACTOR) {
        // ticker function call
        TMR5_CallBack();

        // reset ticker counter
        CountCallBack = 0;
    }

    //***User Area End

    tmr5_obj.count++;
    tmr5_obj.timerElapsed = true;
    IFS1bits.T5IF = false;
}

void TMR5_Period16BitSet(uint16_t value) {
    /* Update the counter values */
    PR5 = value;
    /* Reset the status information */
    tmr5_obj.timerElapsed = false;
}

/**
    void DRV_TMR5_Period16BitSet (uint16_t value)
 */
void DRV_TMR5_Period16BitSet(uint16_t value) {
    TMR5_Period16BitSet(value);
}

uint16_t TMR5_Period16BitGet(void) {
    return ( PR5);
}

/**
    uint16_t DRV_TMR5_Period16BitGet (void)
 */
uint16_t DRV_TMR5_Period16BitGet(void) {
    return (TMR5_Period16BitGet());
}

void TMR5_Counter16BitSet(uint16_t value) {
    /* Update the counter values */
    TMR5 = value;
    /* Reset the status information */
    tmr5_obj.timerElapsed = false;
}

/**
    void DRV_TMR5_Counter16BitSet (uint16_t value)
 */
void DRV_TMR5_Counter16BitSet(uint16_t value) {
    TMR5_Counter16BitSet(value);
}

uint16_t TMR5_Counter16BitGet(void) {
    return ( TMR5);
}

/**
    uint16_t DRV_TMR5_Counter16BitGet (void)
 */
uint16_t DRV_TMR5_Counter16BitGet(void) {
    return (TMR5_Counter16BitGet());
}

void TMR5_CallBack(void) {
    // Add your custom callback code heres
    Debug_Message_Update();
}

void TMR5_Start(void) {
    /* Reset the status information */
    tmr5_obj.timerElapsed = false;

    /*Enable the interrupt*/
    IEC1bits.T5IE = true;

    /* Start the Timer */
    T5CONbits.TON = 1;
}

/**
    void DRV_TMR5_Start (void)
 */
void DRV_TMR5_Start(void) {
    TMR5_Start();
}

void TMR5_Stop(void) {
    /* Stop the Timer */
    T5CONbits.TON = false;

    /*Disable the interrupt*/
    IEC1bits.T5IE = false;
}

/**
    void DRV_TMR5_Stop (void)
 */
void DRV_TMR5_Stop(void) {
    TMR5_Stop();
}

bool TMR5_GetElapsedThenClear(void) {
    bool status;

    status = tmr5_obj.timerElapsed;

    if (status == true) {
        tmr5_obj.timerElapsed = false;
    }
    return status;
}

/**
    bool DRV_TMR5_GetElapsedThenClear (void)
 */
bool DRV_TMR5_GetElapsedThenClear(void) {
    return (TMR5_GetElapsedThenClear());
}

uint8_t TMR5_SoftwareCounterGet(void) {
    return tmr5_obj.count;
}

/**
    uint8_t DRV_TMR5_SoftwareCounterGet (void)
 */
uint8_t DRV_TMR5_SoftwareCounterGet(void) {
    return (TMR5_SoftwareCounterGet());
}

void TMR5_SoftwareCounterClear(void) {
    tmr5_obj.count = 0;
}

/**
    void DRV_TMR5_SoftwareCounterClear (void)
 */
void DRV_TMR5_SoftwareCounterClear(void) {
    TMR5_SoftwareCounterClear();
}

/**
 End of File
 */
