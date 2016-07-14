#include "Init.h"
#include <timer.h>

void Timer1Init()
{
    //Enable Timer1 Interrupt and set priority to "1"
    EnableIntT1;
    SetPriorityIntT1(1);
    WriteTimer1(0);
    OpenTimer1(0x8020, 0xFFFF); //about 1 second period
}
