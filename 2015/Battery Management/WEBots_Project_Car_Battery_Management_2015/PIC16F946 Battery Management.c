/************************************************************************
*                                                                       *
*   Filename:      PIC16F946 Battery Management                         *
*   Date:          DD/MM/YYYY                                           *
*   File Version:  1.2                                                  *
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
    /***** INITIALIZATION *****/

    // set to 1 for input, 0 for output
    // set input for the cell measurement, current measurment
    TRISA = 0b11110100;
    TRISB = 0b00000000;
    TRISC = 0b00000000;
    TRISD = 0b00000000;
    TRISE = 0b11100000;

    CMCON0bits.CM = 7; // disable comparitors
    LCDCONbits.LCDEN = 0; // disable LCD Driver

    // ADC Configuration
    ANSEL = 0xFF; // set all of the analog channels as analog inputs

    // Current
    CG_SEL0 = 1; // Set the current sense module to have a gain of 200
    CG_SEL1 = 1;


    // change if using interrupts
    ADCON1bits.ADCS = 0b101; // ADC time of 4us, see table 12-1
    ADCON0bits.ADFM = 1; // right justified, LSB in ADRESL.0
    ADCON0bits.VCFG = 0b00; // use VSS as voltage reference
    ADCON0bits.ADON = 1; // turn on ADC

    LCD_Init();

    //*** Main loop
    while(1)
    {

    }
}

// Function samples the battery cell voltages and returns the values to global variables
void sampleBatteryCells ()
{
    // sample the reference voltage to get a benchmark value
    sampleReference();
    
    // Turn on the unity feedback op amps
    OPAMP_CGND = 1;

    // sample each of the cell voltages
    BT_CLS_EN = 1; // turn on the first three voltage dividers

    // sample the bottom cells
    cellVolt[0] = ( ( cell1RDT + cell1RDB ) / cell1RDB ) * sampleADC( 0 ); // cell 1
    cellVolt[1] = ( ( cell2RDT + cell2RDB ) / cell2RDB ) * sampleADC( 1 ); // cell 2
    cellVolt[2] = ( ( cell3RDT + cell3RDB ) / cell3RDB ) * sampleADC( 5 ); // cell 3

    BT_CLS_EN = 0; // turn off the first three voltage dividers
    TP_CLS_EN = 1; // turn on the last three voltage dividers

    cellVolt[3] = ( ( cell4RDT + cell4RDB ) / cell4RDB ) * sampleADC( 6 ); // cell 4
    cellVolt[4] = ( ( cell5RDT + cell5RDB ) / cell5RDB ) * sampleADC( 3 ); // cell 5
    cellVolt[5] = ( ( cell6RDT + cell6RDB ) / cell6RDB ) * sampleADC( 4 ); // cell 6

    TP_CLS_EN = 1; // turn on the last three voltage dividers

    OPAMP_CGND = 1; // turn off the voltage dividers

}

void sampleReference()
{
    // turn on the reference voltage divider
    REF_EN = 1;
    // Turn on the unity feedback op amps
    OPAMP_CGND = 1;

    ADCON0bits.CHS = 0b010; // configure module to read AN0
    __delay_us(10); // wait for the ADC to get ready
    ADCON0bits.GO = 1; // start a conversion

    while(ADCON0bits.GO) // bit is cleared when the conversion is complete
    {}

    // turn off the reference voltage divider
    REF_EN = 0;
    // Turn off the unity feedback op amps
    OPAMP_CGND = 0;

    refValue = ADRESH; // put the high order acd data in the low order bits of refValue
    refValue = refValue << 8; // shift the data into the high order bits of refValue
    refValue = refValue | ADRESL; // OR with the lower bits of the ADC result

    supVolt = (refVolt*1023)/refValue; // calculate the supply voltage to be used later

}

// function to sample the ADC, input the channel and the result is returned as a
// 16 bit interger. Ensure that the reference is returned
uint16_t sampleADC ( uint8_t cell)
{
    uint16_t temp = 0; // temporary variable to hold the read data

    ADCON0bits.CHS = cell; // configure module to read the specific ADC channel
    __delay_us(10); // wait for the ADC to get ready
    ADCON0bits.GO = 1; // start a conversion

    while(ADCON0bits.GO) // bit is cleared when the conversion is complete
    {}

    temp = ADRESH; // put the high order acd data in the low order bits of temp
    temp = temp << 8; // shift the data into the high order bits of temp
    temp = temp | ADRESL; // OR with the lower bits of the ADC result

    return temp;
}

void sampleCurrent ()
{
    uint16_t temp = 0; // temporary variable to hold the read data

    ADCON0bits.CHS = 0b111; // configure module to read AN7
    __delay_us(10); // wait for the ADC to get ready
    ADCON0bits.GO = 1; // start a conversion

    while(ADCON0bits.GO) // bit is cleared when the conversion is complete
    {}

    temp = ADRESH; // put the high order acd data in the low order bits of temp
    temp = temp << 8; // shift the data into the high order bits of temp
    temp = temp | ADRESL; // OR with the lower bits of the ADC result

    // possibly sample the reference here to get updated supVolt

    // convert the digital value into voltage
    current = (temp * supVolt)/1023;

    // convert an amplified voltage to a current using the shunt resistance
    // divide by the gain to get the actual voltage and divide by the resistance to get the current
    current = current / (shuntRes * currentGain);

}

// display data to the LCD screen, input interger 0...3 depending on the data desired to be displayed
void displayLCD ( uint8_t disp )
{
    char s0[16]; // String for the top line
    char s1[16]; // String for the bottom line

    switch( disp )
    {
        // display the total voltage of the battery and the estimated percentage left
        // along with the current
        case (0):
        {
            uint8_t sum = 0;

            for(uint8_t i = 0; i < 6; i++)
            {
                sum += cellVolt[i];
            }

            sprintf( s0, 'Voltage:%.3fV', sum )

            LCD_Set_Cursor(0x00);
            LCD_Write_String(&s0);

            sprintf( s1, 'Current:%.3fA', current)

            LCD_Set_Cursor(0x10);
            LCD_Write_String(&s1);
        }

        // display the cell voltages of cells 1 and 2
        case (1):
        {
            sprintf( s0, 'Cell 1:%.2f', cellVolt[0] )

            LCD_Set_Cursor(0x00);
            LCD_Write_String(&s0);

            sprintf( s1, 'Cell 2:%.2f', cellVolt[1] )

            LCD_Set_Cursor(0x10);
            LCD_Write_String(&s1);
        }

        // display the cell voltages of cells 3 and 4
        case (2):
        {
            sprintf( s0, 'Cell 3:%.2f', cellVolt[2] )

            LCD_Set_Cursor(0x00);
            LCD_Write_String(&s0);

            sprintf( s1, 'Cell 4:%.2f', cellVolt[3] )

            LCD_Set_Cursor(0x10);
            LCD_Write_String(&s1);
        }

        // display the cell voltages of cells 5 and 6
        case (3):
        {
            sprintf( s0, 'Cell 5:%.2f', cellVolt[4] )

            LCD_Set_Cursor(0x00);
            LCD_Write_String(&s0);
            
            sprintf( s1, 'Cell 6:%.2f', cellVolt[5] )

            LCD_Set_Cursor(0x10);
            LCD_Write_String(&s1);
        }
    }
}

// sets the gain for the current sense module
// input a gain of 25, 50, 100 or 200
void currentInit ( uint8_t gain )
{

}