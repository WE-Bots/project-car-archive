
/************************************************************************
*                                                                       *
*   Filename:       lcd.h                                               *
*   Date:           02/06/2015                                          *
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
*   Description:    This file contains functions to interface with a LCD*
*************************************************************************
*                                                                       *
*   Pin assignments:                                                    *
*                   Ensure that that the LCD pins have been defined     *
*                   before including this libary                        *
*                                                                       *
************************************************************************/

#include <xc.h>
#include <stdbool.h>

char topStr[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
char btmStr[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// ***Function Declarations
void initLCD();
void LCDSetBits(char);
void pulse();
void LCDcmd(char);
void LCD4bitCmd(char);
void LCDSetCursor(char);
void LCDWriteChar(char);
void LCDWriteString(const char*);
//void clearLCD();
void uint8ToASCII ( char*, uint8_t );
void uint16ToASCII ( char*, uint16_t );
void floatToASCII ( char* , float , uint8_t );

// *** Function Declarations
void initLCD()
{
    RS = 0;             // RS = 0, send instruction
    RW = 0;             // RW = 0, write
    LCDSetBits(0x00);
    __delay_ms(100);        // wait for at least 15ms after power is applied
    LCDSetBits(0x30);       // wake up
    __delay_ms(30);
    pulse();
    __delay_ms(10);
    pulse();
    __delay_ms(10);
    pulse();
    __delay_ms(10);
    LCDSetBits(0x20);      
    pulse();

    LCDcmd(0x28);       // function set
    __delay_ms(10);
    LCDcmd(0x10);       // function set
    __delay_ms(10);
    LCDcmd(0x0F);       // display on/off control
    __delay_ms(10);        
    LCDcmd(0x06);       // display clear
    __delay_ms(10);
}

void LCDSetBits(char a)
{
	if(a & 16)
		D4 = 1;
	else
		D4 = 0;

	if(a & 32)
		D5 = 1;
	else
		D5 = 0;

	if(a & 64)
		D6 = 1;
	else
		D6 = 0;

	if(a & 128)
		D7 = 1;
	else
		D7 = 0;
}

void pulse()
{
    EN = 1;
    __delay_ms(1);          // enable pule width >= 300ns
    EN = 0;                 // clock enable -> falling edge
}

// first 4 bit number to be sent is in the high order 4 bits and the
// second 4 bit number to be sent is in the low order 4 bits
void LCDcmd(char a)
{
	LCDSetBits(a);        // put the data on the output port
        RS = 0;             // RS = 0, send instruction
        RW = 0;             // RW = 0, write
        pulse();            // send the lower 4 bits
        a = a<<4;           // shift to access the higher order bytes
	LCDSetBits(a);        // put the data on the output port
        pulse();            // send the upper 4 bits
}

// 4 bit instruction to be sent is in the high order bit of a
void LCD4bitCmd(char a)
{
	LCDSetBits(a);        // put the data on the output port
        RS = 0;             // RS = 0, send instruction
        RW = 0;             // RW = 0, write
        pulse();            // send the lower 4 bits
}

void LCDWriteChar(char a)
{
    LCDSetBits(a);        // put the data on the output port
    RS = 1;             // RS = 0, send data
    //RW = 0;             // RW = 0, write
    pulse();            // send the lower 4 bits
    a = a<<4;           // shift to access the higher order bytes
    LCDSetBits(a);        // put the data on the output port
    pulse();            // send the upper 4 bits
}


// for first line specify high order bits of a to 0
// and for second line specify high order bits of a to 1
// the location along the line is specified by a [0 - F]
void LCDSetCursor(char a )
{
    if (a & 16) // second line of LCD
    {
       LCD4bitCmd(0xC0);
       a = a << 4;
       LCD4bitCmd(a);
    }

    else        // first line of LCD
    {
       LCD4bitCmd(0x80);
       a = a << 4;
       LCD4bitCmd(a);
    }
}

void LCDWriteString(const char *a)
{
    for(int i=0;a[i]!='\0';i++)
    {
       LCDWriteChar(a[i]);
    }
}

//void clearLCD()
//{
//    LCDcmd(0x01);
//}

// ensure that the array is at least 4 elements long
void uint8ToASCII ( char* returnVal, uint8_t num )
{
    uint8_t divisor = 100;
    uint8_t prevSum = 0;
    bool leading = 1; // variable for tracking leading zeroes
    uint8_t i = 0; // loop counter

    // zero the array
    for (int j = 0; j <= 2; j++)
    {
        returnVal[j] = 0x00;
    }

    do
    {
        // divide and subtract

        returnVal[i] = ( num - prevSum ) / divisor;

        prevSum += returnVal[i]*divisor;

        divisor = divisor/10;

        if (leading == 1)
        {
            if ( returnVal[i] != 0 )
            {
                // found the first digit of the number, start recording it in the array and stop checking for leading zeroes
                i++;
                leading = 0;
            }
        }
        else
            i++;

    } while ( divisor > 0);

    i--;

    for ( uint8_t k = 0; k <= i ; k++ )
    {
        // for ASCII numbers the MSB is 0x3
        // make the MSB while masking the LSB
        returnVal[k] = returnVal[k] | 0x30;
    }
}

// ensure that the array is at least 7 elements long
void uint16ToASCII ( char* returnVal, uint16_t num )
{
    uint16_t divisor = 10000;
    uint16_t prevSum = 0;

    bool leading = 1; // variable for tracking leading zeroes
    uint8_t i = 0; // loop counter

    // zero the array
    for (int j = 0; j <= 5; j++)
    {
        returnVal[j] = 0x00;
    }

    do
    {
        // divide and subtract

        returnVal[i] = ( num - prevSum ) / divisor;

        prevSum += returnVal[i]*divisor;

        divisor = divisor/10;

        if (leading == 1)
        {
            if ( returnVal[i] != 0 )
            {
                // found the first digit of the number, start recording it in the array and stop checking for leading zeroes
                i++;
                leading = 0;
            }
        }
        else
            i++;

    } while ( divisor > 0);

    i--;

    for ( uint8_t k = 0; k <= i ; k++ )
    {
        // for ASCII numbers the MSB is 0x3
        // make the MSB while masking the LSB
        returnVal[k] = returnVal[k] | 0x30;
    }
}

// ensure that the array is at least 8 elements long
void floatToASCII ( char* returnVal, float num, uint8_t precision )
{
    uint32_t scaler = 100;
    uint32_t prevSum = 0;
    bool leading = 1; // variable for tracking leading zeroes
    uint8_t i = 0; // loop counter
    uint8_t decPoint = 0; // tracks the element in the array to put the decimal point
    
    // zero the array
    for (int j = 0; j <= 6; j++)
    {
        returnVal[j] = 0x00;
    }

    // interger part
    do
    {
        // divide and subtract
        // assuming a max of a 999 interger number
        // leading zeroes are removed

        returnVal[i] = ( num - prevSum ) / scaler;

        prevSum += returnVal[i]*scaler;

        scaler = scaler/10;

        if (leading == 1)
        {
            if ( returnVal[i] != 0 )
            {
                // found the first digit of the number, start recording it in the array and stop checking for leading zeroes
                i++;
                leading = 0;
            }
        }
        else
            i++;

    } while ( scaler > 0 );

    
    decPoint = i; // store the location of the decimal point
    i++;

    // fractional part

    scaler = 10;

    for (uint8_t j = i; i - j <= precision ; i++)
    {
        prevSum = prevSum * 10;
        returnVal[i] = (char)( num * scaler - prevSum );

        scaler = scaler * 10;

        prevSum += returnVal[i];
    }

    i--;

    for ( uint8_t k = 0; k <= i ; k++ )
    {
        // for ASCII numbers the MSB is 0x3
        // make the MSB while masking the LSB
        returnVal[k] = returnVal[k] | 0x30;
    }

    returnVal[decPoint] = 0x2E; // add a decimal point

    // add the NULL terminator
    returnVal[i] = '\0';
}