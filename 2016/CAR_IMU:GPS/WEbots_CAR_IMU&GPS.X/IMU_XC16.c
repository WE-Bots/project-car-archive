/*
 * File:   IMU_XC16.c
 * Author: patrick egan
 *
 * Created on June 21, 2016, 8:12 PM
 */

<<<<<<< HEAD
/* List of TODO's
 * - Check for whether there will be a LED on the PCB
 * - Figure out which ports are being used to send and receive data
 * - Figure out where to store Quaternion data
 * - Write a simple api for the Kalman Filter
 * - Write and research calibration code 
=======
/* List of extra TODO's
 * - Check for whether there will be a LED on the PCB //No
 * - Figure out which ports are being used to send and receive data //Look at the board I sent you we are using regular serial port communication for the GPS and the SDA1/SCL1 for the I2C
 * - Figure out where to store Quaternion data //This is given
 * - Write a simple api for the Kalman Filter //This is pretty math based so we need a few more math libraries as well which I can do
 * - Write and research calibration code
>>>>>>> d61afd7d6c6a54d2c3a67ec557b7aab359c78909
 */

/**************************************
 * TODO: Complete layout of memory addresses with descriptions
 * of what each thing does at its respective address
 * 
 * MEMORY STRUCTURE:
 * 
 *************************************/


// DSPIC33EP256MU806 Configuration Bit Settings

// 'C' source line config statements

//TODO: this will have to be moved into another file
// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = OFF                // General Segment Code-Protect bit (General Segment Code protect is disabled)
#pragma config GSSK = OFF               // General Segment Key bits (General Segment Write Protection and Code Protection is Disabled)

// FOSCSEL
#pragma config FNOSC = FRCDIVN          // Initial Oscillator Source Selection bits (Internal Fast RC (FRC) Oscillator with postscaler)
#pragma config IESO = ON                // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = ON             // Peripheral pin select configuration (Allow only one reconfiguration)
#pragma config FCKSM = CSDCMD           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON              // PLL Lock Wait Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = ON              // Watchdog Timer Enable bit (Watchdog timer always enabled)

// FPOR
#pragma config FPWRT = PWR128           // Power-on Reset Timer Value Select bits (128ms)
#pragma config BOREN = ON               // Brown-out Reset (BOR) Detection Enable bit (BOR is enabled)
#pragma config ALTI2C1 = OFF            // Alternate I2C pins for I2C1 (SDA1/SCK1 pins are selected as the I/O pins for I2C1)

// FICD
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config RSTPRI = PF              // Reset Target Vector Select bit (Device will obtain reset instruction from Primary flash)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FAS
#pragma config AWRP = OFF               // Auxiliary Segment Write-protect bit (Aux Flash may be written)
#pragma config APL = OFF                // Auxiliary Segment Code-protect bit (Aux Flash Code protect is disabled)
#pragma config APLK = OFF               // Auxiliary Segment Key bits (Aux Flash Write Protection and Code Protection is Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include "quaternion.h"
//I added this just for testing feel free to remove

//TODO: Setup pre-processor directives for defining macros which
//      will be useful later

//TODO: This is a COMPLETELY wrong file you will have to change this, IMU should be a
//      library
int main(void) {
    return 0;
    
}

//TODO: Setup global variables for ports used to send/receive data
//      and memory addresses used for saving quaternion data

//TODO: make an i2c library for the i2c communication
//TODO: make an interrupt system so when the GPS is ready to transmit to do so
//      then pull the IMU info after