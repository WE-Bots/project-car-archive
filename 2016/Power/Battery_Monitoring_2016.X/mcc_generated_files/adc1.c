
/**
  ADC1 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    adc1.c

  @Summary
    This is the generated header file for the ADC1 driver using MPLAB® Code Configurator

  @Description
    This header file provides APIs for driver for ADC1.
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
#include "adc1.h"

/**
  Section: Data Type Definitions
 */

/* ADC Driver Hardware Instance Object

  @Summary
    Defines the object required for the maintenance of the hardware instance.

  @Description
    This defines the object required for the maintenance of the hardware
    instance. This object exists once per hardware instance of the peripheral.

 */
typedef struct {
    uint8_t intSample;
}

ADC_OBJECT;

static ADC_OBJECT adc1_obj;

/**
  Section: Driver Interface
 */


void ADC1_Initialize(void) {
    // ADSIDL disabled; ASAM disabled; FORM Absolute decimal result, unsigned, right-justified; MODE12 enabled; ADON enabled; DONE disabled; SAMP disabled; SSRC Clearing sample bit ends sampling and starts conversion; 

    AD1CON1 = 0x8400;

    // OFFCAL disabled; ALTS disabled; PVCFG AVDD; BUFM disabled; BUFREGEN disabled; SMPI 1; CSCNA disabled; NVCFG AVSS; 

    AD1CON2 = 0x0000;

    // ADRC RC clock; SAMC 14; EXTSAM disabled; ADCS 0; 

    AD1CON3 = 0x8E00;

    // CH0SA AN0; CH0SB AN0; CH0NB AVSS; CH0NA AVSS; 

    AD1CHS = 0x0000;

    // CSS3 disabled; CSS4 disabled; CSS5 disabled; CSS6 disabled; CSS7 disabled; CSS8 disabled; CSS9 disabled; CSS10 disabled; CSS11 disabled; CSS0 disabled; CSS12 disabled; CSS13 disabled; CSS2 disabled; CSS14 disabled; CSS1 disabled; CSS15 disabled; 

    AD1CSSL = 0x0000;

    // CSS16 disabled; CSS29 disabled; CSS17 disabled; CSS27 disabled; CSS28 disabled; CSS30 disabled; CSS26 disabled; 

    AD1CSSH = 0x0000;


    adc1_obj.intSample = AD1CON2bits.SMPI;

}

/**
    void DRV_ADC1_Initialize (void)
 */
void DRV_ADC1_Initialize(void) {
    ADC1_Initialize();
}

void ADC1_Start(void) {
    AD1CON1bits.SAMP = 1;
}

/**
    void DRV_ADC1_Start (void)
 */
void DRV_ADC1_Start(void) {
    ADC1_Start();
}

void ADC1_Stop(void) {
    AD1CON1bits.SAMP = 0;
}

/**
    void DRV_ADC1_Stop (void)
 */
void DRV_ADC1_Stop(void) {
    ADC1_Stop();
}

uint16_t ADC1_ConversionResultBufferGet(uint16_t *buffer) {
    int count;
    uint16_t *ADC16Ptr;

    ADC16Ptr = (uint16_t *)&(ADC1BUF0);

    for (count = 0; count < adc1_obj.intSample; count++) {
        buffer[count] = (uint16_t) * ADC16Ptr;
        ADC16Ptr++;
    }
    return count;
}

/**
    uint16_t DRV_ADC1_ConversionBufferGet (uint16_t *buffer)
 */
uint16_t DRV_ADC1_ConversionBufferGet(uint16_t *buffer) {
    return ADC1_ConversionResultBufferGet(buffer);
}

uint16_t ADC1_ConversionResultGet(void) {
    return ADC1BUF0;
}

/**
    uint16_t DRV_ADC1_ConversionGet (void)
 */
uint16_t DRV_ADC1_ConversionGet(void) {
    return ADC1_ConversionResultGet();
}

bool ADC1_IsConversionComplete(void) {
    return AD1CON1bits.DONE; //Wait for conversion to complete   
}

/**
    bool DRV_ADC1_IsConversionComplete (void)
 */
bool DRV_ADC1_IsConversionComplete(void) {
    return ADC1_IsConversionComplete();
}

void ADC1_ChannelSelect(ADC1_CHANNEL channel) {
    AD1CHS = channel;
}

/**
    void DRV_ADC1_ChannelSelect (DRV_ADC1_CHANNEL channel )
 */
void DRV_ADC1_ChannelSelect(DRV_ADC1_CHANNEL channel) {
    ADC1_ChannelSelect(channel);
}

/**
    void DRV_ADC1_Tasks (void)
 */
void DRV_ADC1_Tasks(void) {
    ADC1_Tasks();
}

void ADC1_Tasks(void) {
    // clear the ADC interrupt flag
    IFS0bits.AD1IF = false;
}


/**
  End of File
 */
