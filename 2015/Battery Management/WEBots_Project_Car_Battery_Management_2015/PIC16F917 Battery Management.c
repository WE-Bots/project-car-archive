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

    currentGainInit(200);

#ifdef CIRCUIT_DEBUG
    REF_EN = 1;
    TP_CLS_EN = 1;
    BT_CLS_EN = 1;
    OPAMP_CGND = 1;

    while(1);
#endif

    /***** MAIN LOOP *****/
    while(1)
    {

        if ( E_STOP == 0 )
        {
            BRCount = 0; // reset the brown out counter

            MOTOR_CONTROL = 0;
            R_LED = 1;

            while( E_STOP == 0 )
            {

            }

            R_LED = 0;
            MOTOR_CONTROL = 1;
        }

        // brownout occured
        if ( PCONbits.nBOR == 0)
        {
            brownOutHandle ();
        }

        // check if the cells and current are within range
        if ( systemCheck() != 0)
        {
            R_LED = 1;

            stopWatch(0); // start a stopwatch

            // wait one second to ensure that it isn't just a surge
            while ( stopWatch(1) < 1000 )
            {
                systemCheck(); // keep updating the variables to remove lag in running average
            }

            uint8_t tempCheck = systemCheck();

            if ( tempCheck != 0 )
            {
                shutDown( tempCheck );
            }

            R_LED = 0;
        }
        
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

    __delay_ms(5); // wait for the voltage to level out

    // sample the bottom cells
    cellVolt[0] = IIRnew * ( cell1RR + 1 ) * sampleVoltage(CELL1) + IIRprev * cellVolt[0]; // cell 1
    cellVolt[1] = IIRnew * ( cell2RR + 1 ) * sampleVoltage(CELL2) + IIRprev * cellVolt[1]; // cell 2
    cellVolt[2] = IIRnew * ( cell3RR + 1 ) * sampleVoltage(CELL3) + IIRprev * cellVolt[2]; // cell 3

    BT_CLS_EN = 0; // turn off the first three voltage dividers
    TP_CLS_EN = 1; // turn on the last three voltage dividers

    __delay_ms(5);

    cellVolt[3] = IIRnew * ( cell4RR + 1 ) * sampleVoltage(CELL4) + IIRprev * cellVolt[3]; // cell 4
    cellVolt[4] = IIRnew * ( cell5RR + 1 ) * sampleVoltage(CELL5) + IIRprev * cellVolt[4]; // cell 5
    cellVolt[5] = IIRnew * ( cell6RR + 1 ) * sampleVoltage(CELL6) + IIRprev * cellVolt[5]; // cell 6

    TP_CLS_EN = 0; // turn off the last three voltage dividers

    OPAMP_CGND = 0; // turn off the voltage dividers

}

void sampleReference()
{
    // turn on the reference voltage divider
    REF_EN = 1;
    // Turn on the unity feedback op amps
    OPAMP_CGND = 1;

    __delay_ms(2); // wait for the opamp to stabalize

    analogRead(REFV); // do a sample and dont record the result, helps to remove residual voltage from the previous sample
    
    refValue = 0;

    for ( int i = 0; i <= sampleNum; i++)
    {
        refValue += analogRead(REFV);
    }

    refValue = refValue / sampleNum;

    // turn off the reference voltage divider
    REF_EN = 0;
    // Turn off the unity feedback op amps
    OPAMP_CGND = 0;

    supVolt = (refVolt * 1023)/refValue; // calculate the supply voltage to be used later
}

float sampleVoltage(ADCChannel  chan)
{
    uint16_t temp = 0;

    analogRead(chan); // do a sample and dont record the result, helps to remove residual voltage from the previous sample 

    for ( int i = 0; i <= sampleNum; i++)
    {
        temp += analogRead(chan);
    }

    temp = temp / sampleNum;

    // convert the digital value into voltage
    return (temp * supVolt)/1023;
}

void sampleCurrent ()
{
    uint16_t temp = 0;
    float tempCurrent = 0;

#ifndef CURRENT_DEBUG
    // check to find an appropriate current gain to measure the current
    for (uint8_t i = 0; i <= 3; i++)
    {
        currentGainInit(currentGain[i]); // update the current gain

        __delay_ms(2); // wait for the current gain output to stabalize

        // sample the voltage and calibrate is to an actual voltage
        for ( int i = 0; i <= sampleNum; i++)
        {
            temp += analogRead(CURRENT);
        }

        // averaged 10 bit number
        temp = temp / sampleNum;

        tempCurrent = (temp * supVolt)/1023;

        if ( tempCurrent < supVolt - 0.2 )
        {
            current = tempCurrent / ( currentGain[i] * shuntRes );
            // convert the digital voltage into an actual voltage then into a current accounting for the currrent gain
            // return as mA value
            return;
        }

    }

    // if the voltage is above all of the gains then just send the value using the lowest gain.
    current = tempCurrent / ( currentGain[3] * shuntRes );
#else
    uint8_t gain = 0;

    currentGainInit( currentGain[gain] ); // update the current gain

    __delay_ms(2); // wait for the current gain output to stabalize

    // sample the voltage and calibrate is to an actual voltage
    for ( int i = 0; i <= sampleNum; i++)
    {
        temp += analogRead(CURRENT);
    }

    // averaged 10 bit number
    temp = temp / sampleNum;

    tempCurrent = (temp * supVolt)/1023;

    current = tempCurrent / ( currentGain[gain] * shuntRes );
#endif

}

// display data to the LCD screen, input interger 0...3 depending on the data desired to be displayed
void displayLCD ( int disp )
{

    char temp1[8] = {0,0,0,0,0,0,0,0};
    char temp2[8] = {0,0,0,0,0,0,0,0};

    switch( disp )
    {
#ifndef SAVE_MEMORY
        // display the total voltage of the battery and the estimated percentage left
        // along with the current
        case 0:
        {
            // Voltage
            floatToASCII( temp1, cellVolt[5], 2);
            LCDSetCursor(0x00);
            LCDWriteString(str1);
            LCDWriteString(temp1);
            LCDWriteString(str9);
            // Current
            floatToASCII( temp2, current, 2);
            LCDSetCursor(0x10);
            LCDWriteString(str2);
            LCDWriteString(temp2);
            LCDWriteString(str10);
            
            break;
        }

        // display the cell voltages of cells 1 and 2
        case 1:
        {
            // Cell 1
            floatToASCII( temp1, cellVolt[0], 2);
            LCDSetCursor(0x00);
            LCDWriteString(str3);
            LCDWriteString(temp1);
            LCDWriteString(str9);

            // Cell 2
#ifdef CELL2CELLVOLT
            floatToASCII( temp2, cellVolt[1] - cellVolt[0], 2);
#else
            floatToASCII( temp2, cellVolt[1], 2);
#endif
            LCDSetCursor(0x10);
            LCDWriteString(str4);
            LCDWriteString(temp2);
            LCDWriteString(str9);

            break;
        }

        // display the cell voltages of cells 3 and 4
        case 2:
        {
            // Cell 3
#ifdef CELL2CELLVOLT
            floatToASCII( temp1, cellVolt[2] - cellVolt [1], 2);
#else
            floatToASCII( temp1, cellVolt[2], 2);
#endif
            LCDSetCursor(0x00);
            LCDWriteString(str5);
            LCDWriteString(temp1);
            LCDWriteString(str9);

            // Cell 4
#ifdef CELL2CELLVOLT
            floatToASCII( temp2, cellVolt[3] - cellVolt[2], 2);
#else
            floatToASCII( temp2, cellVolt[3], 2);
#endif
            LCDSetCursor(0x10);
            LCDWriteString(str6);
            LCDWriteString(temp2);
            LCDWriteString(str9);

            break;
        }

        // display the cell voltages of cells 5 and 6
        case 3:
        {
            // Cell 5
#ifdef CELL2CELLVOLT
            floatToASCII( temp1, cellVolt[4] - cellVolt [3], 2);
#else
            floatToASCII( temp1, cellVolt[4], 2);
#endif
            LCDSetCursor(0x00);
            LCDWriteString(str7);
            LCDWriteString(temp1);
            LCDWriteString(str9);

#ifdef CELL2CELLVOLT
            floatToASCII( temp2, cellVolt[5] - cellVolt[4], 2);
#else
            floatToASCII( temp2, cellVolt[5], 2);
#endif
            LCDSetCursor(0x10);
            LCDWriteString(str8);
            LCDWriteString(temp2);
            LCDWriteString(str9);

            break;
        }
#endif
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

    stopWatch(0);
    while( LCD_PB == 0 ) 
    {
        if(stopWatch(1) >= 1000)
        {
            initLCD();

            R_LED = 1;
            __delay_ms(100);
            R_LED = 0;
            return;
        }
    }

    LCDDisplayMode++;

    if (LCDDisplayMode > 3)
        LCDDisplayMode = 0;
}

// if system check is sucessful the function will return 0
// if a cell is out of range it will return the cell number
// if current is out of range it will return 7
uint8_t systemCheck ()
{
    sampleReference();
    sampleCurrent();
    sampleBatteryCells();
    updateI2CData();

    // check cell 1 voltage
    if ( cellVolt[0] < cellVoltL )
    {
        // cell 1 voltage is out of range
        return 1;
    }

    // check cell 2 voltage
    if ( cellVolt[1] - cellVolt[0] < cellVoltL )
    {
        // cell 2 voltage is out of range
        return 2;
    }

    // check cell 3 voltage
    if ( cellVolt[2] - cellVolt[1] < cellVoltL )
    {
        // cell 3 voltage is out of range
        return 3;
    }

    // check cell 4 voltage
    if ( cellVolt[3] - cellVolt[2] < cellVoltL )
    {
        // cell 4 voltage is out of range
        return 4;
    }

    // check cell 5 voltage
    if ( cellVolt[4] - cellVolt[3] < cellVoltL )
    {
        // cell 5 voltage is out of range
        return 5;
    }

    // check cell 6 voltage
    if ( cellVolt[5] - cellVolt[4] < cellVoltL )
    {
        // cell 6 voltage is out of range
        return 6;
    }

    // check current
    if ( current > 90 )
    {
        // current is oot opf range
        // return 7; // MAKE SURE TO UNCOMMENT IF CHECKING CURRENT
    }

    return 0;
}

// function that is called if a brownout has occured
// most likely when motor control is turned on
//      wait and turn it bacck on again
void brownOutHandle ()
{
    PCONbits.nBOR = 1; // reset brownout module

    BRCount = BRCount + 1;

    if ( BRCount >= 20)
    {
        shutDown(1);
    }
    __delay_ms(50);

    MOTOR_CONTROL = 1;
}

void shutDown( uint8_t error)
{
    #ifdef SHUTOFF_DEBUG

                // something went wrong .. display error message and flash lights
                char tempChar[4];
                uint8ToASCII(tempChar, error);
                LCDSetCursor(0x00);
                LCDWriteString("ERROR WITH: ");
                LCDWriteString(tempChar);

                while(1)
                {
                    G_LED = 0;
                    __delay_ms(200);
                    G_LED = 1;
                    __delay_ms(200);
                }
#else
                // something went wrong, shut everything down and go into low powered sleep
                GIE = 0;

                // for low power consumption set everything as output low
                // also shuts off the battery power
                TRISA = 0b00000000;
                TRISB = 0b00000000;
                TRISC = 0b00000000;
                TRISD = 0b00000000;
                TRISE = 0b00000000;

                PORTA = 0;
                PORTB = 0;
                PORTC = 0;
                PORTD = 0;
                PORTE = 0;

                while(1)
                {
                    SLEEP();
                }
#endif
}