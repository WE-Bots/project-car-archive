//odometer board

#include <stdio.h>
#include <stdlib.h>
#include <p33FJ06GS101A.h>
#include <xc.h>

#include "Interrupts.h"



//*********ISRs being used*****************
void __attribute__((interrupt,no_auto_psv)) _CNInterrupt(void)
{
    if (edgeDir == 1)    //if on rising edge
    {
        LATBbits.LATB2 = 1; //turn Red LED on
        countNumber++;
        if (countNumber == countTotalNumber)
        {
            pulseCountDist++;
            countNumber = 0; 
        }
        edgeDir = 0;
    }

    else
    {
       LATBbits.LATB2 = 0; //turn Red LED off
       //only count on high pulseCountDist++;
       edgeDir = 1;
    }

    IFS1bits.CNIF = 0; // Clear CN interrupt
}

void __attribute__((interrupt,no_auto_psv)) _SI2C1Interrupt(void)

{   if(justSent)
    {
        IFS1bits.SI2C1IF = 0;
        justSent = 0;
    }
     
    else
    {
            
      I2C1TRN = pulseCountDist;  //sends out current data
      
      justSent = 1;
      IFS1bits.SI2C1IF = 0;
    }

 }


//************Empty ISRs for the fix***************

void __attribute__((interrupt,no_auto_psv)) _OscillatorFail(void)
{
    INTCON1bits.OSCFAIL = 0;
}

void __attribute__((interrupt,no_auto_psv)) _AddressError(void)
{
    INTCON1bits.ADDRERR = 0;
}

void __attribute__((interrupt,no_auto_psv)) _StackError(void)
{
    INTCON1bits.STKERR = 0;
}

void __attribute__((interrupt,no_auto_psv)) _MathError(void)
{
    INTCON1bits.MATHERR = 0;
}

void __attribute__((interrupt,no_auto_psv)) _INT0Interrupt(void)
{
    IFS0bits.INT0IF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _OC1Interrupt(void)
{
    IFS0bits.OC1IF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _T2Interrupt(void)
{
    IFS0bits.T2IF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _SPI1ErrInterrupt (void)
{
    IFS0bits.SPI1EIF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _SPI1Interrupt (void)
{
    IFS0bits.SPI1IF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _U1RXInterrupt(void)
{
    IFS0bits.U1RXIF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _ADCInterrupt(void)
{
    IFS0bits.ADIF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _MI2C1Interrupt(void)
{
    IFS1bits.MI2C1IF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _INT1Interrupt(void)
{
    IFS1bits.INT1IF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _INT2Interrupt(void)
{
    IFS1bits.INT2IF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _PWMSpEventMatchInterrupt(void)
{
    IFS3bits.PSEMIF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _U1ErrInterrupt(void)
{
    IFS4bits.U1EIF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _JTAGInterrupt(void)
{
    IFS5bits.JTAGIF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _PWM1Interrupt(void)
{
    IFS5bits.PWM1IF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _PWM4Interrupt(void)
{
    IFS6bits.PWM4IF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _ADCP0Interrupt(void)
{
    IFS6bits.ADCP1IF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _ADCP1Interrupt(void)
{
    IFS6bits.ADCP1IF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _ADCP3Interrupt(void)
{
    IFS7bits.ADCP3IF = 0;
}

//void __attribute__((interrupt,no_auto_psv)) _ADCP6Interrupt(void)
//{
//    IFS7bits.ADCP6IF = 0;
//}