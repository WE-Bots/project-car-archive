/*
 * File:   inputCapture.c
 * Author: Jacob
 *
 * Created on July 13, 2016, 6:18 PM
 */

#include "inputCapture.h"
#include "userVariables.h"

//toggle variables used to blink led with pole changes 
char FLtoggle = 0;
char FRtoggle = 0; 
char BLtoggle = 0;
char BRtoggle = 0;

//hold timer value from IC
volatile int FLcaptime = 0;
volatile int FRcaptime = 0;
volatile int BLcaptime = 0;
volatile int BRcaptime = 0;

volatile unsigned long FLcount = 0;
volatile unsigned long FRcount = 0;
volatile unsigned long BLcount = 0;
volatile unsigned long BRcount = 0;

void inputCapture_init (void)
{
  //setup HE LEDs
    //FL LED
    TRISDbits.TRISD6 = 0;       //set as output 
    LATDbits.LATD6 = 0;         //turn off  
    
    //FR LED
    TRISDbits.TRISD7 = 0;       //set as output 
    LATDbits.LATD7 = 0;         //turn off 
    
    //BL LED
    TRISDbits.TRISD5 = 0;       //set as output 
    LATDbits.LATD5 = 0;         //turn off 
    
    //BR LED
    TRISDbits.TRISD4 = 0;       //set as output 
    LATDbits.LATD4 = 0;         //turn off 
    
  //setup IC1 on FL
    //map output pin -> decided to only use A element on each HE sensor 
    ANSELBbits.ANSB8 = 0;
    TRISBbits.TRISB8 = 1;
    RPINR7bits.IC1R = 0b0101000;    //IC1 mapping to FL_HEA RPI40 
    
    //configure IC1 setup registers 
    IC1CON1bits.ICSIDL = 0;         //Input capture will continue to operate in CPU idle mode
    IC1CON1bits.ICTSEL = 0b111;     //use peripheral clock as source
    IC1CON1bits.ICM = 0b001;       //capture mode every rising/falling edge, interrupt every time
    IC1CON2bits.SYNCSEL = 0;        //No sync/trigger source
    
    //setup IC1 interrupt
    IFS0bits.IC1IF = 0;     //clear IC1 interrupt status flag
    IPC0bits.IC1IP = 4;     //set interrupt priority 4
    
  //setup IC2 on FR 
    //map output pin
    ANSELBbits.ANSB10 = 0;
    TRISBbits.TRISB10 = 1;
    RPINR7bits.IC2R = 0b0101010;    //IC2 mapping to FR_HEA RPI42
    
    //configure IC2 setup registers 
    IC2CON1bits.ICSIDL = 0;         //Input capture will continue to operate in CPU idle mode
    IC2CON1bits.ICTSEL = 0b111;     //use peripheral clock as source
    IC2CON1bits.ICM = 0b001;       //capture mode every rising/falling edge, interrupt every time
    IC2CON2bits.SYNCSEL = 0;        //No sync/trigger source
    
    //setup IC2 interrupt
    IFS0bits.IC2IF = 0;     //clear IC2 interrupt status flag
    IPC1bits.IC2IP = 4;     //set interrupt priority 4
    
  //setup IC3 on BL
    //map output pin   
    ANSELBbits.ANSB12 = 0;
    TRISBbits.TRISB12 = 1;
    RPINR8bits.IC3R = 0b0101100;    //IC3 mapping to BL_HEA RPI44
    
    //configure IC3 setup registers 
    IC3CON1bits.ICSIDL = 0;         //Input capture will continue to operate in CPU idle mode
    IC3CON1bits.ICTSEL = 0b111;     //use peripheral clock as source
    IC3CON1bits.ICM = 0b001;       //capture mode every rising/falling edge, interrupt every time
    IC3CON2bits.SYNCSEL = 0;        //No sync/trigger source
    
    //setup IC3 interrupt
    IFS2bits.IC3IF = 0;     //clear IC3 interrupt status flag
    IPC9bits.IC3IP = 4;     //set interrupt priority 4
    
   //setup IC4 on BR
    //map output pin
    ANSELBbits.ANSB14 = 0;
    TRISBbits.TRISB14 = 1;
    RPINR8bits.IC4R = 0b0101110;    //IC4 mapping to BR_HEA RPI46            
    
    //configure IC4 setup registers 
    IC4CON1bits.ICSIDL = 0;         //Input capture will continue to operate in CPU idle mode
    IC4CON1bits.ICTSEL = 0b111;     //use peripheral clock as source
    IC4CON1bits.ICM = 0b001;       //capture mode every rising/falling edge, interrupt every time
    IC4CON2bits.SYNCSEL = 0;        //No sync/trigger source
    
    //setup IC4 interrupt
    IFS2bits.IC4IF = 0;     //clear IC3 interrupt status flag
    IPC9bits.IC4IP = 4;     //set interrupt priority 4
 
  //enable all IC interrupts 
    IEC0bits.IC1IE = 1;     //enable IC1 interrupt
    IEC0bits.IC2IE = 1;     //enable IC2 interrupt 
    IEC2bits.IC3IE = 1;     //enable IC3 interrupt
    IEC2bits.IC4IE = 1;     //enable IC4 interrupt
}

void __attribute__ ((__interrupt__, no_auto_psv)) _IC1Interrupt(void)
{  
    FLcaptime = IC1BUF;
    IC1CON2bits.TRIGSTAT = 0;
    FLcount = FLcount + 1;
    
    if (FLtoggle == 0)
    {
        LATDbits.LATD6 = 1;
        FLtoggle = 1;
    }    
    
    else if (FLtoggle == 1)
    {
        LATDbits.LATD6 = 0;
        FLtoggle = 0;
    } 
        
    IFS0bits.IC1IF = 0;     //clear flag
}

void __attribute__ ((__interrupt__, no_auto_psv)) _IC2Interrupt(void)
{  
    FRcaptime = IC2BUF;
    IC2CON2bits.TRIGSTAT = 0;
    FRcount = FRcount + 1;
    
    if (FRtoggle == 0)
    {
        LATDbits.LATD7 = 1;
        FRtoggle = 1;
    }    
    
    else if (FRtoggle == 1)
    {
        LATDbits.LATD7 = 0;
        FRtoggle = 0;
    } 
        
    IFS0bits.IC2IF = 0;     //clear flag
}

void __attribute__ ((__interrupt__, no_auto_psv)) _IC3Interrupt(void)
{  
    BLcaptime = IC3BUF;
    IC3CON2bits.TRIGSTAT = 0;
    BLcount = BLcount + 1;
    
    if (BLtoggle == 0)
    {
        LATDbits.LATD5 = 1;
        BLtoggle = 1;
    }    
    
    else if (BLtoggle == 1)
    {
        LATDbits.LATD5 = 0;
        BLtoggle = 0;
    } 
        
    IFS2bits.IC3IF = 0;     //clear flag
}

void __attribute__ ((__interrupt__, no_auto_psv)) _IC4Interrupt(void)
{  
    BRcaptime = IC4BUF;
    IC4CON2bits.TRIGSTAT = 0;
    BRcount = BRcount + 1;
    
    if (BRtoggle == 0)
    {
        LATDbits.LATD4 = 1;
        BRtoggle = 1;
    }    
    
    else if (BRtoggle == 1)
    {
        LATDbits.LATD4 = 0;
        BRtoggle = 0;
    } 
        
    IFS2bits.IC4IF = 0;     //clear IC3 interrupt status flag
}