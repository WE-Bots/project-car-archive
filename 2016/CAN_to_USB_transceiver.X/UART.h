/*
 * File:   UART.h
 * Author: Kevin
 *
 * Created on July 13, 2016, 5:35 PM
 */

#ifndef UART_H
#define	UART_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <p33Exxxx.h>

    /*Public Funtions*/
    void UART1Init(unsigned int baud);
    void UART1Enable();
    void UART1Disable();
    unsigned int UART1Read();
    unsigned int UART1ReadReady();
    void UART1Write(char data);
    unsigned int UART1WriteReady();
    void UART1WriteStr(char * data, unsigned int length);
    void UART1WriteStrNT(char * data);
    void UART1EnableInterrupts();
    void UART1DisableInterrupts();

    void UART1CheckReceiveBuffer();

    /*UART1 ISRs*/
    void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void);
    void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void);

#ifdef	__cplusplus
}
#endif

#endif	/* UART_H */

