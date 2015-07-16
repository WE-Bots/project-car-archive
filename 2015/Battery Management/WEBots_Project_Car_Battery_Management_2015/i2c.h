/************************************************************************
*                                                                       *
*   Filename:       adc.h                                               *
*   Date:           2015/06/24                                          *
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
*   Description:    This file contains functions to interface with the  *
*                   SSP module to use I2C Communication                 *
*                                                                       *
*                   Battery Monitoring Communication Protocol           *
*                   - first, 2 bytes - unsigned int for the voltage     *
*                   - second, 1 byte - unsigned int for the current     *
*                   - last, 1 byte - result of the above 3 bytes XORed  *
*                     together                                          *
*                                                                       *
*************************************************************************
*                                                                       *
*   Pin assignments:                                                    *
*                                                                       *
************************************************************************/

#ifndef I2C_H
#define	I2C_H

void initI2C();
void updateI2CData ();
void isrI2C ();

void initI2C()
{
    // ensure that the clock is on RC6
    // ensure that the data is on RC7

    // ensure that these pins are externally pulled up

    // set the clock and data as input
    TRISCbits.TRISC6 = 1;
    TRISCbits.TRISC7 = 1;

    RCSTAbits.SPEN = 0; // ensure that the UART is not operating

    SSPADD = 12; // SSP Address register

    SSPCON = 0b00111000;
    //         0_______ - Write Collision Detect bit - no collision
    //         _0______ - Receive Overflow Indicator bit - no overflow
    //         __1_____ - enable the SSP module
    //         ___1____ - enable the clock
    //         ____0110 - select I2C mode using a 7-bit address

    SSPSTAT = 0b00000000;
    //          00______ - cleared for I2C
    //          __xxxxxx - read only bits

    // SSPBUF - Serial Receive/Transmit Buffer

    // enable interrupts for the SSP module

    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 0;

    PIE1bits.SSPIE = 1;
}

void updateI2CData ()
{
        // initialize the variables
        for ( uint8_t i = 0; i <= 3; i++)
        {
            sendData[i] = 0;
        }

        // set the values of sendData
        // first, 2 bytes - unsigned int for the voltage
        //  - map the voltage from 19.2V to 25.2V in 2 bytes
        //      - return return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        uint16_t sendVolt = (cellVolt[5] - 19.2) * 10922.6666;

        sendData[0] = sendVolt & 0x00FF; // low part of the voltage

        sendData[1] = ( sendVolt & 0xFF00 ) >> 8; // high part of the voltage

        // second, 1 byte - unsigned int for the current
        sendData[2] = ( uint8_t )current; // will provide a 1 A resolution
                                        // potentially in the future mult by 2 to get 0.5 A resolution

        // last, 1 byte - result of the above 3 bytes XORed together
        sendData[3] = ( sendData[0] ^ sendData[1] ) ^ sendData[2];
}

void isrI2C ()
{
    if (PIE1bits.SSPIE == 1 &&  PIR1bits.SSPIF == 1 )
    {
        // *** Service I2C interrupt

        if ( countI2C == 1 )
        {
            SSPBUF = sendData[1];
            countI2C++;
        }

        else if ( countI2C == 2 )
        {
            SSPBUF = sendData[2];
            countI2C++;
        }

        else if ( countI2C == 3 )
        {
            SSPBUF = sendData[3];
            countI2C++;
        }

        else if ( countI2C == 4)
        {
            countI2C = 0;
            // send the first byte of data
            SSPBUF = sendData[0];
            countI2C++;
        }
/*
        // else if the last byte was data
        if ( countI2C == 1 )
        {
            SSPBUF = sendData[countI2C];
            countI2C++;
        }
        // if the last byte was an address
        else if ( countI2C == 4 )
        {
            countI2C = 0;
            // send the first byte of data
            SSPBUF = sendData[countI2C];
            countI2C++;
        }
*/


        PIR1bits.SSPIF = 0; // reset the interrupt flag
    }
}

#endif	/* I2C_H */

