#include <CAN.h>

void main()
{
    CANInit();
    while (CANIsTransmitComplete());
    CANTransmit(0, 0, 0);

}

