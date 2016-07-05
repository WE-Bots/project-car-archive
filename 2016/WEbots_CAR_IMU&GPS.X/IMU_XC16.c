/*
 * File:   IMU_XC16.c
 * Author: patrick egan
 *
 * Created on June 21, 2016, 8:12 PM
 */

//<<<<<<< HEAD
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