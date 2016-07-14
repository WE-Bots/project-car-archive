/*
 * File:   main.c
 * Author: Kevin McLean
 *
 * Description: Main program for the CAN to USB tranciever for the 2016 CAR for WE Bots.
 */

#include "Init.h"
#include <uart.h>
#include "CAN.h"
#include <timer.h>
/* for info on using the peripheral libraries look up
 * xc16/v1.22/docs/periph_libs/16-bit Peripheral Libraries.htm
 * in the xc16 compiler install files
 */

unsigned char o[2] = {'0', '\0'};
char hex[3] = {'0', '0', '\0'};
volatile char tick = 0;

/*Timer1 ISR*/
void __attribute__((__interrupt__)) _T1Interrupt(void)
{
    tick = 1;
    WriteTimer1(0);
    IFS0bits.T1IF = 0; /* Clear Timer interrupt flag */
}

int main(void)
{
    /*
    CANInit();
    while (CANIsTransmitComplete());
    CANTransmit(0, 0, 0);
     */

    //TRISDbits.TRISD2 = 0;
    UART1Init();
    Timer1Init();
    CAN1Init();
    putsUART1((unsigned int *) "Start\n");
    while (1)
    {
        CAN1CheckReceiveBuffer();
        if (tick)
        {
            tick = 0;
            putsUART1((unsigned int *) "tick\n");
        }
        if (DataRdyUART1())
        {
            o[0] = ReadUART1();
            switch (o[0])
            {
                case '1':
                    //if (!BusyUART1())
                {
                    putsUART1((unsigned int *) "1\n");
                    o[0] = 'a';
                    CAN1Transmit(CANMSG_ESTOP, 1, (unsigned int *) o);
                    //LATDbits.LATD2 = 1;
                }
                    break;
                case '0':
                    //if (!BusyUART1())
                {
                    putsUART1((unsigned int *) "0\n");
                    //LATDbits.LATD2 = 0;
                }
                    break;
                default:
                    putsUART1((unsigned int *) "Unknown: ");
                    putsUART1((unsigned int *) o);
                    putsUART1((unsigned int *) "\n");
                    //sprintf(hex, "%x", o[0]);
                    putsUART1((unsigned int *) "0x");
                    putsUART1((unsigned int *) hex);
                    putsUART1((unsigned int *) "\n");
                    break;
            }
        }
    }
    return 0;
}

