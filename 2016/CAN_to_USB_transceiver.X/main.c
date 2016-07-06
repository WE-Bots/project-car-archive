#include "Init.h"
#include <uart.h>
#include <ecan.h>
// for info on using the peripheral libraries look up
// xc16/v1.22/docs/periph_libs/16-bit Peripheral Libraries.htm

int main(void)
{
    /*
    CANInit();
    while (CANIsTransmitComplete());
    CANTransmit(0, 0, 0);
     */

    unsigned char o[2] = {'0', '\0'};
    char hex[3]={ '0', '0', '\0'};

    TRISDbits.TRISD2 = 0;
    UARTInit();
    while (1)
    {
        if (DataRdyUART1())
        {
            o[0] = ReadUART1();
            switch (o[0])
            {
                case '1':
                    if (!BusyUART1())
                    {
                        putsUART1((unsigned int *) "1\n");
                        LATDbits.LATD2 = 1;
                    }
                    break;
                case '0':
                    if (!BusyUART1())
                    {
                        putsUART1((unsigned int *) "0\n");
                        LATDbits.LATD2 = 0;
                    }
                    break;
                default:
                    putsUART1((unsigned int *) "Unknown: ");
                    putsUART1((unsigned int *) o);
                    putsUART1((unsigned int *) "\n");
                    sprintf (hex, "%x", o[0]);
                    putsUART1((unsigned int *) "0x");
                    putsUART1((unsigned int *) hex);
                    putsUART1((unsigned int *) "\n");
                    break;
            }
        }
    }
    return 0;
}

