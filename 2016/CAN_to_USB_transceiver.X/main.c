/*
 * File:   main.c
 * Author: Kevin McLean
 *
 * Description: Main program for the CAN to USB tranciever for the 2016 CAR for WE Bots.
 */

#include "Init.h"
#include "UART.h"
#include "CAN.h"
#include <timer.h>
/* for info on using the peripheral libraries look up
 * xc16/v1.22/docs/periph_libs/16-bit Peripheral Libraries.htm
 * in the xc16 compiler install files
 */

char o[2] = {'0', '\0'};
char hex[3] = {'0', '0', '\0'};
volatile char tick = 0;

/*Timer1 ISR*/
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
    tick = 1;
    WriteTimer1(0);
    IFS0bits.T1IF = 0; /* Clear Timer interrupt flag */
}

int main(void)
{
    UART1Init(9600);
    Timer1Init();
    CAN1Init();
    UART1WriteStrNT("Start\n");
    while (1)
    {
        CAN1CheckReceiveBuffer();
        if (tick)
        {
            tick = 0;
            UART1WriteStrNT("tick\n");
        }
        if (UART1ReadReady())
        {
            o[0] = UART1Read();
            switch (o[0])
            {
                case '1':
                    UART1WriteStrNT("1\n");
                    o[0] = 'a';
                    CAN1Transmit(CANMSG_ESTOP, 1, (unsigned int *) o);
                    break;
                case '0':
                    UART1WriteStrNT("0\n");
                    break;
                default:
                    UART1WriteStrNT("Unknown: ");
                    UART1WriteStrNT(o);
                    UART1WriteStrNT("\n");
                    //sprintf(hex, "%x", o[0]);
                    UART1WriteStrNT("0x");
                    UART1WriteStrNT(hex);
                    UART1WriteStrNT("\n");
                    break;
            }
        }
    }
    return 0;
}

