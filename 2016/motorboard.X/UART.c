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

/*UART2 ISRs*/
void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void)
{
    IFS1bits.U2RXIF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _U2TXInterrupt(void)
{
    IFS1bits.U2TXIF = 0;
}

void UART1Init(unsigned int baud)
{
    //Map UART1 to the proper output pins
    RPINR18bits.U1RXR = 0b1001001;
    RPOR6bits.RP85R = 1;

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
    U1STAbits.UTXEN = 1;
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

void UART2Init(unsigned int baud)
{
    //Map UART1 to the proper output pins
    RPINR19bits.U2RXR = 0b1001011;
    RPOR0bits.RP64R = 3;

    //UART configuration
    //8-bit data, no parity, one stop bit, no interruts
    U2BRG = (unsigned int) (230312.5 / baud - 0.5);
    IEC1bits.U2RXIE = 1;
    IEC1bits.U2TXIE = 1;
    U2MODE = 0x8000;
    U2STA = 0xA400;
}

void UART2Enable()
{
    U2MODEbits.UARTEN = 1;
    U2STAbits.UTXEN = 1;
}

void UART2Disable()
{
    U2MODEbits.UARTEN = 0;
}

unsigned int UART2Read()
{
    return U2RXREG;
}

unsigned int UART2ReadReady()
{
    return U2STAbits.URXDA;
}

void UART2Write(char data)
{
    U2TXREG = data;
}

unsigned int UART2WriteReady()
{
    return !U2STAbits.UTXBF;
}

void UART2WriteStr(char * data, unsigned int length)
{
    int i = 0;
    for (i = 0; i < length; i++)
    {
        while (!UART2WriteReady())
        {
        }
        UART2Write(data[i]);
    }
}

void UART2WriteStrNT(char * data)
{
    int i = 0;
    while (data[i] != '\0')
    {
        while (!UART2WriteReady())
        {
        }
        UART2Write(data[i++]);
    }
}

void UART2EnableInterrupts()
{
    IEC1bits.U2RXIE = 1;
    IEC1bits.U2TXIE = 1;
}

void UART2DisableInterrupts()
{
    IEC1bits.U2RXIE = 1;
    IEC1bits.U2TXIE = 1;
}

void UART1CheckReceiveBuffer()
{
    static char buffer [6]; //4 byte data, 1 byte checksum, 1 byte buffer to store frame close char
    static int bufferIndex = 0;
    static int started = 0;

    while (UART1ReadReady())
    {
        buffer[bufferIndex] = UART1Read();
        /*Check for start of frame*/
        if (buffer[bufferIndex] == '<')
        {
            started = 1;
            bufferIndex = 0;
            continue;
        }
        /*Read data*/
        if (started)
        {
            /*Check for end of frame*/
            if (bufferIndex == 5 && buffer[5] == '>')
            {
                /*Check checksum*/
                if (buffer[0] + buffer[1] + buffer[2] + buffer[3] != buffer[4])
                {
                    started = 0;
                    continue;
                }
                /*store values*/
                desMotor = buffer[0];
                desMotor = desMotor << 8;
                desMotor += buffer[1];
                desServo = buffer[2];
                desServo = desServo << 8;
                desServo += buffer[3];
                started = 0;
                continue;
            }
                /*check for frame error*/
            else if (bufferIndex == 5 || buffer[bufferIndex] == '>')
            {
                started = 0;
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

void UART2CheckReceiveBuffer()
{
    if (UART2ReadReady())
    {
        unsigned int tmp = UART2Read();
        if (tmp!=0)
        {
            EStopRemote=tmp;
            TMR4 = 0x0000;
        }
    }
}

void initWatchdog()
{
    T4CONbits.TON = 0;
    T4CONbits.T32 = 0;
    T4CONbits.TSIDL = 0b1;
    T4CONbits.TCKPS = 0b10; //1:64 prescaler
    T4CONbits.TCS = 0b0;
    TMR4 = 0x0000;
    PR4 = 34547; //300ms period: 8.68us per tick
    IFS1bits.T4IF = 0; // Clear Timer 4 Interrupt Flag
    IPC6bits.T4IP = 0x04; // Set Timer 4 Interrupt Priority Level
    IEC1bits.T4IE = 1; // Enable Timer4 interrupt
}

void __attribute__((__interrupt__, no_auto_psv)) _T4Interrupt(void)
{
    EStopRemote=1;
    IFS1bits.T4IF = 0; //clear interrupt flag
}