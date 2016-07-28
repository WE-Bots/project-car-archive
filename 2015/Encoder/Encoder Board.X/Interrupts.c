//encoder board

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
        speedFreq = TMR1;   //get speed
        valueUpdated = 1;  //sets flag so average can be re calculated
        TMR1 = 0x00;        //reset timer
        LATBbits.LATB2 = 1; //turn Red LED on
        pulseCountDist++;
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
    if (sendReady && prevtxDone)          //check if there is data to send
    {
       if (txindex > txSize)       //resets flag and buffer if 4 bytes (all data) are sent
        {
            txindex = 0;
            prevtxindex = 0;
            prevtxDone = 0;
            sendReady = 0;
            justSent = 0;
            IFS1bits.SI2C1IF = 0;
        }

        else
        {
           if (txindex == 2)        //loads pulsecount in slave interrupt so that most accurate value is sent 
           {
               
               
              distBuffer = 0x00AA;
               txBuffer[3] = distPointer[0]; 
               //distBuffer = distBuffer << 1;  
               txBuffer[2] = distPointer[1];
               
               
               /*
               //txBuffer[4] = ((0x55 & 0xF0) >>4);          
               //txBuffer[5] = (0x55 & 0x0F);
               txBuffer[4] = ((distPointer[0] & 0xF0) >>4);           //loads 3rd byte (dist)
               txBuffer[5] = (distPointer[0] & 0x0F);
               txBuffer[6] = ((distPointer[1] & 0xF0) >>4);           //loads 3rd byte (dist)
               txBuffer[7] = (distPointer[1] & 0x0F);
               //we are letting it roll over pulseCountDist = 0;                 //clears distance to avoid overload
                */
               txBuffer[4] = (((txBuffer[0] ^ txBuffer[1]) ^ txBuffer[2])^txBuffer[3]);  //loads 4th byte (XOR of 3 data bytes for error checking)
           }          
            
            I2C1TRN = txBuffer[txindex];  //sends out current data
            prevtxBuffer[prevtxindex] = txBuffer[txindex];
            txindex++;                      //prepares for next data send
            prevtxindex++;
            justSent = 1;
            IFS1bits.SI2C1IF = 0;
        }

    }

    else    //if no new data is ready send oldest sent data 
    {
       if (prevtxindex > txSize)       //resets flag and buffer if 4 bytes (all data) are sent
        {
            prevtxindex = 0;
            prevtxDone = 1;
            justSent = 0;
            IFS1bits.SI2C1IF = 0;
        }

        else
        {
            I2C1TRN = prevtxBuffer[prevtxindex];  //sends out current data
            prevtxindex++;                      //prepares for next data send
            justSent = 1;
            IFS1bits.SI2C1IF = 0;
        }

    }
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