/*
 * File:   main.c
 * Author: Kevin McLean
 *
 * Description: Main program for the CAN to USB tranciever for the 2016 CAR for WE Bots.
 */

#include "UART.h"
#include "CAN.h"

int main(void)
{
    UART1Init(9600);
    CAN1Init();
    UART1WriteStrNT("Start\n");
    while (1)
    {
        CAN1CheckReceiveBuffer();
        UART1CheckReceiveBuffer();
    }
    return 0;
}

