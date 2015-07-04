/************************************************************************
*                                                                       *
*   Filename:      PIC16F946 Battery Management                         *
*   Date:          2015/06/15                                           *
*   File Version:  1.0                                                  *
*                                                                       *
*   Author:        Andrew Cullen                                        *
*                                                                       *
*************************************************************************
*                                                                       *
*   Architecture:  Mid-range PIC                                        *
*   Processor:     PIC16F917                                            *
*   Compiler:      MPLAB XC8 v1.00 (Free mode)                          *
*                                                                       *
*************************************************************************
*                                                                       *
*   Files required: none                                                *
*                                                                       *
*************************************************************************
*                                                                       *
*   Description:                                                        *
*                                                                       *
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

#include "setup.h"

/***** MAIN PROGRAM *****/
void main()
{
    initController();

    /***** Initial Check *****/

    G_LED = 1;

    //currentGainInit(200);

    //stopWatch(0); // start the stopwatch

    sprintf(topStr, "                ");
    LCDSetCursor(0x00);
    LCDWriteString (topStr);
    sprintf(btmStr, "                ");
    LCDSetCursor(0x10);
    LCDWriteString (btmStr);

    /***** MAIN LOOP *****/
    while(1)
    {
        sampleReference();
        sampleCurrent();
        sampleBatteryCells();
        
        handlePB();
        
        displayLCD(LCDDisplayMode);
    }
}


// Function samples the battery cell voltages and returns the values to global variables
void sampleBatteryCells ()
{
    // Turn on the unity feedback op amps
    OPAMP_CGND = 1;

    // sample each of the cell voltages
    BT_CLS_EN = 1; // turn on the first three voltage dividers

    __delay_ms(1); // wait for the voltage to level out

    // sample the bottom cells
    cellVolt[0] = ( ( cell1RDT + cellRDB ) / cellRDB ) * sampleVoltage(CELL1); // cell 1
    cellVolt[1] = ( ( cell2RDT + cellRDB ) / cellRDB ) * sampleVoltage(CELL2); // cell 2
    cellVolt[2] = ( ( cell3RDT + cellRDB ) / cellRDB ) * sampleVoltage(CELL3); // cell 3

    BT_CLS_EN = 0; // turn off the first three voltage dividers
    TP_CLS_EN = 1; // turn on the last three voltage dividers

    __delay_ms(1);

    cellVolt[3] = ( ( cell4RDT + cellRDB ) / cellRDB ) * sampleVoltage(CELL4); // cell 4
    cellVolt[4] = ( ( cell5RDT + cellRDB ) / cellRDB ) * sampleVoltage(CELL5); // cell 5
    cellVolt[5] = ( ( cell6RDT + cellRDB ) / cellRDB ) * sampleVoltage(CELL6); // cell 6

    TP_CLS_EN = 0; // turn off the last three voltage dividers

    OPAMP_CGND = 0; // turn off the voltage dividers

}

void sampleReference()
{
    // turn on the reference voltage divider
    REF_EN = 1;
    // Turn on the unity feedback op amps
    OPAMP_CGND = 1;

    __delay_ms(5);

    refValue = analogRead(REFV);

    // turn off the reference voltage divider
    REF_EN = 0;
    // Turn off the unity feedback op amps
    OPAMP_CGND = 0;

    supVolt = (refVolt * 1023)/refValue; // calculate the supply voltage to be used later
}

float sampleVoltage(ADCChannel  chan)
{
    __delay_ms(5);
    // convert the digital value into voltage
    return (analogRead(chan) * supVolt)/1023;
}

void sampleCurrent ()
{
    // convert the digital value into voltage
    // convert an amplified voltage to a current using the shunt resistance
    // divide by the gain to get the actual voltage and divide by the resistance to get the current
    current = ((analogRead(CURRENT) * supVolt)/1023) / (shuntRes * currentGain);

}

// display data to the LCD screen, input interger 0...3 depending on the data desired to be displayed
void displayLCD ( int disp )
{
    switch( disp )
    {
        // display the total voltage of the battery and the estimated percentage left
        // along with the current
        case 0:
        {
            sprintf( topStr, "Voltage:%.3f V      ", batteryVoltage() );

            LCDSetCursor(0x00);
            LCDWriteString(topStr);

            sprintf( btmStr, "Current:%.3fA      ", current);

            LCDSetCursor(0x10);
            LCDWriteString(btmStr);

            break;
        }

        // display the cell voltages of cells 1 and 2
        case 1:
        {
            sprintf( topStr, "Cell 1:%.2f     ", cellVolt[0] );

            LCDSetCursor(0x00);
            LCDWriteString(topStr);

            sprintf( btmStr, "Cell 2:%.2f     ", cellVolt[1] );

            LCDSetCursor(0x10);
            LCDWriteString(btmStr);

            break;
        }

        // display the cell voltages of cells 3 and 4
        case 2:
        {
            sprintf( topStr, "Cell 3:%.2f     ", cellVolt[2] );

            LCDSetCursor(0x00);
            LCDWriteString(topStr);

            sprintf( btmStr, "Cell 4:%.2f     ", cellVolt[3] );

            LCDSetCursor(0x10);
            LCDWriteString(btmStr);

            break;
        }

        // display the cell voltages of cells 5 and 6
        case 3:
        {
            sprintf( topStr, "Cell 5:%.2f     ", cellVolt[4] );

            LCDSetCursor(0x00);
            LCDWriteString(topStr);
           
            sprintf( btmStr, "Cell 6:%.2f     ", cellVolt[5] );

            LCDSetCursor(0x10);
            LCDWriteString(btmStr);

            break;
        }
    }
}

// sets the gain for the current sense module
// input a gain of 25, 50, 100 or 200
void currentGainInit ( uint8_t gain )
{
    if ( gain >= 100 )
    {
        CG_SEL0 = 1;

        if ( gain == 100 )
        {
            CG_SEL1 = 0; // sets gain as 100
            return;
        }
        else
        {
            CG_SEL1 = 1; // sets gain as 200
            return;
        }

    }

    else
    {
        CG_SEL0 = 0;

        if ( gain == 25 )
        {
            CG_SEL1 = 0; // sets gain as 25
            return;
        }
        else
        {
            CG_SEL1 = 1; // sets gain as 50
            return;
        }

    }

}

void handlePB ()
{
    if( LCD_PB == 1 )
        return;

    __delay_ms(50);

    if( LCD_PB == 1 )
        return;

    while( LCD_PB == 0 ) {}

    LCDDisplayMode++;

    if (LCDDisplayMode > 3)
        LCDDisplayMode = 0;
}

float batteryVoltage ()
{
    float sum = 0;

    for(uint8_t i = 0; i < 6; i++)
    {
    sum += cellVolt[i];
    }

    return sum;
}

void checkCurrent ()
{
    if ( current > 100)
    {
        // see if it is just a spike
        __delay_ms(500);

        sampleReference();
        sampleCurrent();

        if (current > 100)
        {
            // over allowable current
            MOTOR_CONTROL = 0;
        }
    }
}