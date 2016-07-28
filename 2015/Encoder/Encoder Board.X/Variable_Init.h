/* 
 * File:   Variable_Init.h
 * Author: jtryon
 *
 * Created on July 25, 2014, 8:09 PM
 */

#ifndef VARIABLE_INIT_H
#define	VARIABLE_INIT_H

//used in the interrupt declared in Inteurrupt.h
extern volatile char edgeDir ;  //keeps track of edge direction (1 = rising, 0 = low)
extern volatile unsigned int speedFreq ;//holds the time between pulses from TMR1
extern volatile int pulseCountDist  ;  //hold the number of pulses
extern volatile unsigned int txindex ;
extern volatile char sendReady;
extern volatile char valueUpdated;
extern volatile char txBuffer[9];

unsigned int speedSum = 0;
unsigned int valueArray[] = {0,0,0,0,0,0,0,0};
unsigned int oldValueIndex = 0;
unsigned int average = 0;
char *averagePointer = &average;   //pointer to the average int so it can be broken into two bytes for txing


/*
volatile unsigned int edgeDir = 1;   //keeps track of edge direction (1 = rising, 0 = low)
volatile unsigned int speedFreq = 0; //holds the speed value
volatile long pulseCountDist = 0;    //hold the number of pulses
*/

#endif



