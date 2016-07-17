/*
 * File:   UART.c
 * Author: Kevin
 *
 * Created on July 13, 2016, 5:35 PM
 */

#include "UART.h"
#include "CAN.h"

/*UART1 ISRs*/
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
    IFS0bits.U1RXIF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0;
}

void UART1Init(unsigned int baud)
{
    //Map UART1 to the proper output pins
    RPINR18bits.U1RXR = 0b1001001;
    RPOR0bits.RP64R = 1;

    //UART configuration
    //8-bit data, no parity, one stop bit, no interruts
    U1BRG = (unsigned int) (230312.5 / baud - 0.5);
    IEC0bits.U1RXIE = 1;
    IEC0bits.U1TXIE = 1;
    U1MODE = 0x8000;
    U1STA = 0xA400;
}

void UART1Enable()
{
    U1MODEbits.UARTEN = 1;
    U1STAbits.UTXEN=1;
}

void UART1Disable()
{
    U1MODEbits.UARTEN = 0;
}

unsigned int UART1Read()
{
    return U1RXREG;
}

unsigned int UART1ReadReady()
{
    return U1STAbits.URXDA;
}

void UART1Write(char data)
{
    U1TXREG = data;
}

unsigned int UART1WriteReady()
{
    return !U1STAbits.UTXBF;
}

void UART1WriteStr(char * data, unsigned int length)
{
    int i = 0;
    for (i = 0; i < length; i++)
    {
        while (!UART1WriteReady())
        {
        }
        UART1Write(data[i]);
    }
}

void UART1WriteStrNT(char * data)
{
    int i = 0;
    while (data[i] != '\0')
    {
        while (!UART1WriteReady())
        {
        }
        UART1Write(data[i++]);
    }
}

void UART1EnableInterrupts()
{
    IEC0bits.U1RXIE = 1;
    IEC0bits.U1TXIE = 1;
}

void UART1DisableInterrupts()
{
    IEC0bits.U1RXIE = 1;
    IEC0bits.U1TXIE = 1;
}

void UART1CheckReceiveBuffer()
{
    static char buffer [6]; //4 byte data, 1 byte checksum, 1 byte buffer to store frame close char
    static int bufferIndex=0;
    static int started=0;

    while (UART1ReadReady())
    {
        buffer[bufferIndex]=UART1Read();
        /*Check for start of frame*/
        if (buffer[bufferIndex]=='<')
        {
            started=1;
            bufferIndex=0;
            continue;
        }
        /*Read data*/
        if (started)
        {
            /*Check for end of frame*/
            if (bufferIndex==5 && buffer[5]=='>')
            {
                /*Check checksum*/
                //if(buffer[0]+buffer[1]+buffer[2]+buffer[3]!=buffer[4])
                //{
                  //  started=0;
                   // continue;
               // }
                /*Transmit over CAN*/
                while(!CAN1IsTransmitComplete())
                {}
                unsigned int temp=0xFFff;
                CAN1Transmit(CANMSG_ESTOP, 1, &temp);
                started=0;
                continue;
            }
            /*check for frame error*/
            else if (bufferIndex==5 || buffer[bufferIndex]=='>')
            {
                started=0;
                continue;
            }
            /*Must be valid data*/
            else
            {
                bufferIndex++;
            }
        }
    }
}