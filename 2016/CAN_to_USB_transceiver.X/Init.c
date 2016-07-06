#include <uart.h>

void UARTInit()
{
    //Map UART1 to the proper output pins
    RPINR18bits.U1RXR=0b1001001;
    RPOR0bits.RP64R=1;

    //UART configuration
    //9600 baud, 8-bit data, no parity, one stop bit, no interruts
    DisableIntU1RX;
    DisableIntU1TX;
    OpenUART1(UART_EN & UART_NO_PAR_8BIT & UART_1STOPBIT, UART_TX_ENABLE, 23);
    
    return;
}
