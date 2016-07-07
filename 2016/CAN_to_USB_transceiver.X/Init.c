#include <uart.h>
#include <timer.h>

void UART1Init()
{
    //Map UART1 to the proper output pins
    RPINR18bits.U1RXR=0b1001001;
    RPOR0bits.RP64R=1;

    //UART configuration
    //9600 baud, 8-bit data, no parity, one stop bit, no interruts
    DisableIntU1RX;
    DisableIntU1TX;
    OpenUART1(0x8000, 0xA400, 23);      //9600 baud
    //OpenUART1(0x8000, 0xA400, 1);       //115200 baud
    
    return;
}

void Timer1Init()
{
    //Enable Timer1 Interrupt and set priority to "1"
    EnableIntT1;
    SetPriorityIntT1(1);
    WriteTimer1(0);
    OpenTimer1(0x8020,0xFFFF);  //about 1 second period
}