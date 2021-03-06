/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB� Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB� Code Configurator - v2.25.2
        Device            :  PIC24FV32KA304
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.24
        MPLAB             :  MPLAB X v2.35 or v3.00
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

// define the system clock frequency for delay functions
#define FCY _XTAL_FREQ

#include "mcc_generated_files/mcc.h"
#include <libpic30.h>

/*
                         Main application
 */
int main(void)
{
    // initialize the device
    SYSTEM_Initialize();

    /** 
    On circuitry disabling and holding itself on 
    */
    
    // Wait for the power to stabalize 
    __delay_ms(300);
    
    // Turn on power connect to the first cell 
    cell1PowerEn_SetHigh();
    
    // Disable the ON circuitry 
    disableOnCircuit_SetHigh();

    /*
    uint32_t prevTime = TMR3_ms_Timer();
    uint16_t switchPeriod = 5000;
    */
    
    while (1)
    {
        //regulatorSwitch_SetHigh();
        // Handle the E-Stop
        // Check if the normally open port of the E-Stop is pressed
        //      -> The other normally closed connection is inline with the 
        //          motor switch line
        if (EStopSignal_GetValue() == 1)
        {
            // disengage the motor 
            motorSwitch_SetLow(); 
            regulatorSwitch_SetLow();
            Set_Debug_Message(DEBUG_LED_MESSAGE_5);
        }
        // The E-Stop isn't pressed
        else 
        {
            // engage the motor 
            motorSwitch_SetHigh();
            regulatorSwitch_SetHigh();
            Set_Debug_Message(DEBUG_LED_MESSAGE_2);
        }
    }

    return -1;
}
/**
 End of File
 */