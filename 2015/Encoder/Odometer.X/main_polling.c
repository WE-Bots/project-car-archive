/* 
 * File:   main.c
 * Author: jtryon
 *
 * Created on July 25, 2014, 5:57 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <p33FJ06GS101A.h>

// DSPIC33FJ06GS101A Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>
#include "Variable_Init.h"

// FICD
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = ON              // JTAG Enable bit (JTAG is enabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON              // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config FWDTEN = ON              // Watchdog Timer Enable bit (Watchdog timer always enabled)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = ON          // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = ON             // Peripheral pin select configuration (Allow only one reconfiguration)
#pragma config FCKSM = CSDCMD           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)

// FOSCSEL
#pragma config FNOSC = FRCDIVN          // Oscillator Source Selection (Internal Fast RC (FRC) Oscillator with postscaler)
#pragma config IESO = ON                // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF                // General Segment Code-Protect bit (General Segment Code protect is Disabled)



/*
 * 
 */

int main()
{
//sets up the timer
    T1CONbits.TON = 0; // Disable Timer
    T1CONbits.TCS = 0; // Select internal instruction cycle clock
    T1CONbits.TGATE = 0; // Disable Gated Timer mode
    T1CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    TMR1 = 0x00; // Clear timer register
    PR1 = 0xFFFF; // Load the period value

 
    TRISBbits.TRISB0 = 1;   //sets pin 5 CN0 to input

 //sets up Red LED
    TRISBbits.TRISB1 = 0; //sets it as output
    LATBbits.LATB1 = 0; //sets it low to start

 //sets up blue LED
    TRISBbits.TRISB2 = 0; //sets it as output
    LATBbits.LATB2 = 0; //sets it low to start

 while (1)
 {
     edgeDir = PORTBbits.RB0;

     if (edgeDir == 1)    //if on rising edge
    {
        speedFreq = TMR1;   //get speed
        TMR1 = 0x00;        //reset timer
        LATBbits.LATB1 = 1; //turn Red LED on
        pulseCountDist++;
    }

    else
    {
       LATBbits.LATB1 = 0; //turn Red LED off
       pulseCountDist++;
    }


  }

    return (EXIT_SUCCESS);
}
