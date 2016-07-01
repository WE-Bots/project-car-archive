#include "Init.h"
#include <uart.h>
//#include "CAN.h"

void main()
{
    /*
    CANInit();
    while (CANIsTransmitComplete());
    CANTransmit(0, 0, 0);
     */
    unsigned long i = 0;
    char o = 0;
    TRISDbits.TRISD2 = 0;
    UARTInit();
    while (1)
    {
        if (DataRdyUART1())
        {
            o = ReadUART1();
            switch (o)
            {
                case '1':
                    if (!BusyUART1())
                    {
                        putsUART1("1\n");
                        LATDbits.LATD2 = 1;
                    }
                    break;
                case '0':
                    if (!BusyUART1())
                    {
                        putsUART1("0\n");
                        LATDbits.LATD2 = 0;
                    }
                    break;
                default:
                    putsUART1("Unknown: ");
                    putsUART1(o);
                    putsUART1("\n");
                    break;
            }
        }
        for (i = 0; i < 0xFFFF; i++)
        {
        }
    }
    return;
}

