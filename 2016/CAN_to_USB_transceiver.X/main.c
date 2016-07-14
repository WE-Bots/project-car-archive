/*
 * File:   main.c
 * Author: Kevin McLean
 *
 * Description: Main program for the CAN to USB tranciever for the 2016 CAR for WE Bots.
 */

#include "UART.h"
#include "CAN.h"

char o[2] = {'0', '\0'};
char hex[3] = {'0', '0', '\0'};

int main(void)
{
    UART1Init(9600);
    CAN1Init();
    UART1WriteStrNT("Start\n");
    while (1)
    {
        CAN1CheckReceiveBuffer();

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

