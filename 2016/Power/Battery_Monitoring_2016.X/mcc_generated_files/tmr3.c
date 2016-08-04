
/**
  TMR3 Generated Driver API Source File 

  @Company
    Microchip Technology Inc.

  @File Name
    tmr3.c

  @Summary
    This is the generated source file for the TMR3 driver using MPLAB® Code Configurator

  @Description
    This source file provides APIs for driver for TMR3. 
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
#include "tmr3.h"

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

static TMR_OBJ tmr3_obj;

volatile static uint32_t ticks_ms;

/**
  Section: Driver Interface
 */


void TMR3_Initialize(void) {
    //TSIDL disabled; TGATE disabled; TCS FOSC/2; TCKPS 1:1; TON enabled; 
    T3CON = 0x8000;
    //TMR3 0; 
    TMR3 = 0x0000;
    //Period Value = 1.000 ms; PR3 250; 
    PR3 = 0x00FA;

    IFS0bits.T3IF = false;
    IEC0bits.T3IE = true;

    tmr3_obj.timerElapsed = false;

    ticks_ms = 0;
}

/**
    void DRV_TMR3_Initialize (void)
 */
void DRV_TMR3_Initialize(void) {
    TMR3_Initialize();
}

uint32_t TMR3_ms_Timer(void)
{
    return ticks_ms;
}

void __attribute__((interrupt, no_auto_psv)) _T3Interrupt() {
    /* Check if the Timer Interrupt/Status is set */

    //***User Area Begin

    // ticker function call;
    // ticker is 1 -> Callback function gets called everytime this ISR executes
    TMR3_CallBack();

    //***User Area End

    tmr3_obj.count++;
    tmr3_obj.timerElapsed = true;
    IFS0bits.T3IF = false;
}

void TMR3_Period16BitSet(uint16_t value) {
    /* Update the counter values */
    PR3 = value;
    /* Reset the status information */
    tmr3_obj.timerElapsed = false;
}

/**
    void DRV_TMR3_Period16BitSet (uint16_t value)
 */
void DRV_TMR3_Period16BitSet(uint16_t value) {
    TMR3_Period16BitSet(value);
}

uint16_t TMR3_Period16BitGet(void) {
    return ( PR3);
}

/**
    uint16_t DRV_TMR3_Period16BitGet (void)
 */
uint16_t DRV_TMR3_Period16BitGet(void) {
    return (TMR3_Period16BitGet());
}

void TMR3_Counter16BitSet(uint16_t value) {
    /* Update the counter values */
    TMR3 = value;
    /* Reset the status information */
    tmr3_obj.timerElapsed = false;
}

/**
    void DRV_TMR3_Counter16BitSet (uint16_t value)
 */
void DRV_TMR3_Counter16BitSet(uint16_t value) {
    TMR3_Counter16BitSet(value);
}

uint16_t TMR3_Counter16BitGet(void) {
    return ( TMR3);
}

/**
    uint16_t DRV_TMR3_Counter16BitGet (void)
 */
uint16_t DRV_TMR3_Counter16BitGet(void) {
    return (TMR3_Counter16BitGet());
}

void TMR3_CallBack(void) {
    // Add your custom callback code here
    ticks_ms++;
}

void TMR3_Start(void) {
    /* Reset the status information */
    tmr3_obj.timerElapsed = false;

    /*Enable the interrupt*/
    IEC0bits.T3IE = true;

    /* Start the Timer */
    T3CONbits.TON = 1;
}

/**
    void DRV_TMR3_Start (void)
 */
void DRV_TMR3_Start(void) {
    TMR3_Start();
}

void TMR3_Stop(void) {
    /* Stop the Timer */
    T3CONbits.TON = false;

    /*Disable the interrupt*/
    IEC0bits.T3IE = false;
}

/**
    void DRV_TMR3_Stop (void)
 */
void DRV_TMR3_Stop(void) {
    TMR3_Stop();
}

bool TMR3_GetElapsedThenClear(void) {
    bool status;

    status = tmr3_obj.timerElapsed;

    if (status == true) {
        tmr3_obj.timerElapsed = false;
    }
    return status;
}

/**
    bool DRV_TMR3_GetElapsedThenClear (void)
 */
bool DRV_TMR3_GetElapsedThenClear(void) {
    return (TMR3_GetElapsedThenClear());
}

uint8_t TMR3_SoftwareCounterGet(void) {
    return tmr3_obj.count;
}

/**
    uint8_t DRV_TMR3_SoftwareCounterGet (void)
 */
uint8_t DRV_TMR3_SoftwareCounterGet(void) {
    return (TMR3_SoftwareCounterGet());
}

void TMR3_SoftwareCounterClear(void) {
    tmr3_obj.count = 0;
}

/**
    void DRV_TMR3_SoftwareCounterClear (void)
 */
void DRV_TMR3_SoftwareCounterClear(void) {
    TMR3_SoftwareCounterClear();
}

/**
 End of File
 */
