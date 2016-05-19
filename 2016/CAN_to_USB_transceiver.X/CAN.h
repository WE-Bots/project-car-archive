/* 
 * File:   CAN.h
 * Author: Kevin
 *
 * Created on May 15, 2016, 12:37 PM
 */

#ifndef CAN_H
#define	CAN_H

#ifdef	__cplusplus
extern "C" {
#endif

#define NUM_OF_ECAN_BUFFERS 32
    //This is the ECAN message buffer declaration. Note the buffer alignment.
    unsigned int ecan1MsgBuf[NUM_OF_ECAN_BUFFERS][8]
    __attribute__((aligned(NUM_OF_ECAN_BUFFERS * 16)));

    //functions
    void init();
    int CANIsTransmitComplete();
    void CANTransmit(unsigned int SID, unsigned int length, unsigned int* data);
    void CANTransmitRemote(unsigned int SID);



#ifdef	__cplusplus
}
#endif

#endif	/* CAN_H */

