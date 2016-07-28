/*
 * File:   main.c
 * Author: Jacob
 *
 * Created on July 11, 2016, 7:55 PM
 */

#include <p33Exxxx.h>

#include "userVariables.h"
#include "inputCapture.h"
#include "PWM.h"
#include "UART.h"

int motorDuty = 1500;
int servoDuty = 1500;

float desMotor=0;
float desServo=0;

char EStopRemote=0;

int main(void) 
{
    //int i = 0;
    //int j = 0;
    
    //lED setup
    //start LED
    TRISDbits.TRISD3 = 0;       //set as output
    LATDbits.LATD3 = 1;         //turn on 
    
    //debug1 LED
    ANSELBbits.ANSB1 = 0;       //set as general IO
    TRISBbits.TRISB1 = 0;       //sets it as output
    LATBbits.LATB1 = 0;         //turn on 
    
    //debug2 LED
    ANSELBbits.ANSB2 = 0;       //set as general IO
    TRISBbits.TRISB2 = 0;       //sets it as output
    LATBbits.LATB2 = 0;         //turn on 
    
    //run input capture init function
    inputCapture_init();
    PWMInit();
    
    //start button 
    ANSELBbits.ANSB2 = 0;       //set as general IO
    TRISBbits.TRISB2 = 1;       //sets it as input
    
    //collision avoidance flag 
    TRISDbits.TRISD8 = 1;
   
    
    while (1)
    {
        
        //PWM1SetDutyCycleUS(motorDuty);
        //PWM2SetDutyCycleUS(servoDuty);
        /*
        LATDbits.LATD3 = 1;
        while (i < 10000)
        {
            i = i+1;
        }
        
        LATDbits.LATD3 = 0;
        while (j < 10000)
        {
            j = j+1;
        }
        
        i = 0;
        j = 0;
        */
    }
  
    return 0;
}
