/* 
 * File:   CAN.h
 * Author: Kevin
 *
 * Created on May 15, 2016, 12:37 PM
 *
 * Description: Basic functions for use of the CAN module on the dsPIC33EP256MU806.
 *              Developed for use as part of WE Bots Project C.A.R.
 */

#ifndef CAN_H
#define	CAN_H

#ifdef	__cplusplus
extern "C" {
#endif
#include <p33Exxxx.h>

#define NUM_OF_ECAN_BUFFERS 16  //only used for memory allocation

    //This is the ECAN message buffer declaration.
    extern unsigned int ecan1MsgBuffer[NUM_OF_ECAN_BUFFERS][8];

    //Public functions
    void CAN1Init();
    int CAN1IsTransmitComplete();
    void CAN1Transmit(unsigned int SID, unsigned int length, unsigned int* data);
    void CAN1TransmitRemote(unsigned int SID);



#ifdef	__cplusplus
}
#endif

#endif	/* CAN_H */

