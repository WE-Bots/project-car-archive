/*
 * File:   PWM.c
 * Author: Kevin
 *
 * Created on July 14, 2016, 5:01 PM
 */

#include "PWM.h"

/* Use 1:1 timmer prescaller?? Need to check that this can actually reach the 20ms period
 * Use standard edge-aligned PWM
 * Don't need timer sync
 * No special event trigger
 * Complementary Independent Duty Cycle and Phase, Fixed Primary Period, Edge-Aligned. See p.44 for code ex
 * servo PWM2H
 * motor PWM1H
 *
 */
void PWMInit()
{
    PTPER = 36850;
    PHASE1=0;
    PHASE2=0;
    PDC1 = 2763;
    PDC2 = 2763;
    DTR1=25;
    DTR2=25;
    ALTDTR1=25;
    ALTDTR2=25;
    IOCON1bits.PENH = 1;
    IOCON2bits.PENH=1;
    PWMCON1bits.DTC = 2;
    PWMCON2bits.DTC = 2;
    FCLCON1bits.FLTMOD=3;
    FCLCON2bits.FLTMOD=3;
    PTCON2bits.PCLKDIV = 2;
    PTCONbits.PTEN=1;
}

void PWM1SetDutyCycleUS(int us)
{
    PDC1 =1.8425*us+0.5;
}

void PWM2SetDutyCycleUS(int us)
{
    PDC2 =1.8425*us+0.5;
}

void PWM1SetDutyCycle(int dutyCycle)
{
    PDC1 =368.5*dutyCycle+0.5;
}

void PWM2SetDutyCycle(int dutyCycle)
{
    PDC2 =368.5*dutyCycle+0.5;
}