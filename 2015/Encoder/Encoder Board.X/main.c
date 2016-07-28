//encoder board

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
#pragma config ICS = PGD2               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = ON              // JTAG Enable bit (JTAG is disabled)

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

int main()
{
    //setup I2C Module
    //I2C1MSK = 0xFFFF;
    TRISBbits.TRISB6 = 1;
    TRISBbits.TRISB7 = 1; 
    I2C1CONbits.A10M = 0;       //sets 7 bit address mode
    I2C1ADD = 8;               //writes address
    I2C1CONbits.GCEN = 0;       //disables general calls
    I2C1CONbits.DISSLW = 1;     //disables slew rate since not on 400kHz

    IPC4bits.SI2C1IP = 7;
    IFS1bits.SI2C1IF = 0;       //resets i2c slave interrupt
    IEC1bits.SI2C1IE = 1;       //enables slave interrupt
    I2C1CONbits.I2CEN = 1;      //enables i2c module
    I2C1TRN = 0x00;        //empty the transmit register (resets to 0xFF)

    //sets up the timer
    T1CONbits.TON = 0; // Disable Timer
    T1CONbits.TCS = 0; // Select internal instruction cycle clock
    T1CONbits.TGATE = 0; // Disable Gated Timer mode
    T1CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    TMR1 = 0x00; // Clear timer register
    PR1 = 0xFFFF; // Load the period value

   
    //Sets up the change notification interrupt
    ADPCFGbits.PCFG3 = 1;   //turns off ADC AN3 shared with CN0 pin (automatically turns on)
    TRISBbits.TRISB0 = 1;   //sets pin 5 CN0 to input
    CNEN1bits.CN0IE = 1;    //Enable CN0 pin for interrupt detection
    CNPU1bits.CN0PUE = 0;  //disables internal pullup on input change pin 0
    

    //sets up Red LED
    TRISBbits.TRISB1 = 0; //sets it as output
    LATBbits.LATB1 = 1; //sets it low to start

    //sets up blue LED
    TRISBbits.TRISB2 = 0; //sets it as output
    LATBbits.LATB2 = 0; //sets it High to start

    T1CONbits.TON = 1; // Start Timer
 

    //part of trying stuff with eugen
    SRbits.IPL = 0;             //cpu interrupt priority
    CORCONbits.IPL3 = 0;
    //INTCON2bits.DISI = 0;     //turns off global interrupt disable
    //DISICNT = 0;              //may not be needed was trying lots of stuff
    
    //interrupt setup
    IPC4bits.CNIP = 4; //set CN Interrupt priority
    IFS1bits.CNIF = 0; // Reset CN interrupt
    IEC1bits.CNIE = 1; // Enable CN interrupts

    while (1)
    {
        //when value is updated recalculate average
        if (valueUpdated)
        {
            speedSum = speedSum -  valueArray[oldValueIndex];
            valueArray[oldValueIndex] = speedFreq;
            speedSum = speedSum +  valueArray[oldValueIndex];
            average = 0x5533; //average = speedSum >> 3;    //bitshift 3 times to divide by 8 for mean average

            oldValueIndex++;        //move to next oldest value in array for next time
            if (oldValueIndex > 7)          //reset index if at end of array
                oldValueIndex = 0;
            sendReady = 0;
            
            
            txBuffer[1] = averagePointer[0]; 
            //average = average << 1;  
            txBuffer[0] = averagePointer[1];
              
            
            //txBuffer[1] = 0xFE;
            
            //**************old shit********************
            //txBuffer[0] = ((0xFE & 0xF0) >>4);         
            //txBuffer[1] = ((0xFE & 0x0F));      
            //txBuffer[2] = ((0xAA & 0xF0) >>4);         
            //txBuffer[3] = ((0xAA & 0x0F));
            /*
            txBuffer[0] = ((averagePointer[0] & 0xF0) >>4);         
            txBuffer[1] = (averagePointer[0] & 0x0F);      
            txBuffer[2] = ((averagePointer[1] & 0xF0) >>4);         
            txBuffer[3] = (averagePointer[1] & 0x0F);
             */ 
            sendReady = 1;              //sets flag that data is ready to send
            valueUpdated = 0;           //clears flag that there is new value to work with 
        }
    }

    return (EXIT_SUCCESS);
}

