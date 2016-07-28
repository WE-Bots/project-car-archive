//encoder board

/* 
 * File:   Interrupts.h
 * Author: Jacob
 *
 * Created on June 11, 2015, 7:54 PM
 */

#ifndef INTERRUPTS_H
#define	INTERRUPTS_H

//function definitions for interrupts being used
void __attribute__((interrupt,no_auto_psv)) _CNInterrupt(void);

//function definititons for inturrupts for the fix
void __attribute__((interrupt,no_auto_psv)) _OscillatorFail(void);
void __attribute__((interrupt,no_auto_psv)) _AddressError(void);
void __attribute__((interrupt,no_auto_psv)) _StackError(void);
void __attribute__((interrupt,no_auto_psv)) _MathError(void);

void __attribute__((interrupt,no_auto_psv)) _INT0Interrupt(void);
void __attribute__((interrupt,no_auto_psv)) _OC1Interrupt(void);
void __attribute__((interrupt,no_auto_psv)) _T1Interrupt(void);
void __attribute__((interrupt,no_auto_psv)) _T2Interrupt(void);

void __attribute__((interrupt,no_auto_psv)) _SPI1ErrInterrupt(void);
void __attribute__((interrupt,no_auto_psv)) _SPI1Interrupt(void);

void __attribute__((interrupt,no_auto_psv)) _U1RXInterrupt(void);
void __attribute__((interrupt,no_auto_psv)) _U1TXInterrupt(void);
void __attribute__((interrupt,no_auto_psv)) _ADCInterrupt(void);
void __attribute__((interrupt,no_auto_psv)) _SI2C1Interrupt(void);
void __attribute__((interrupt,no_auto_psv)) _MI2C1Interrupt(void);
void __attribute__((interrupt,no_auto_psv)) _INT1Interrupt(void);
void __attribute__((interrupt,no_auto_psv)) _INT2Interrupt(void);
void __attribute__((interrupt,no_auto_psv)) _PWMSpEventMatchInterrupt(void);
void __attribute__((interrupt,no_auto_psv)) _U1ErrInterrupt(void);
void __attribute__((interrupt,no_auto_psv)) _JTAGInterrupt(void);
void __attribute__((interrupt,no_auto_psv)) _PWM1Interrupt(void);
void __attribute__((interrupt,no_auto_psv)) _PWM4Interrupt(void);
void __attribute__((interrupt,no_auto_psv)) _ADCP0Interrupt(void);
void __attribute__((interrupt,no_auto_psv)) _ADCP1Interrupt(void);
void __attribute__((interrupt,no_auto_psv)) _ADCP3Interrupt(void);
//void __attribute__((interrupt,no_auto_psv)) _ADCP6Interrupt(void);

//variables used in interrupts
volatile char edgeDir = 1;   //keeps track of edge direction (1 = rising, 0 = low)
volatile int pulseCountDist = 0;    //hold the number of pulses

volatile char justSent = 0;
volatile char countNumber = 0;
volatile char countTotalNumber = 10; 


#endif	/* INTERRUPTS_H */

