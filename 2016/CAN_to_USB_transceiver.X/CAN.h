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
#include "CAN_Msg_Priorities.h"

#define CANMSG_ESTOP            CLB_PRTY01 // E-Stop sent received over Bluetooth at Motor board
#define CANMSG_COLLEMERG        CLB_PRTY02 // Immenent Collision detected on Collision Avoidance board
#define CANMSG_BRNOUTWARN       CLB_PRTY03 // Main power brown-out warning from Power board
#define CANMSG_TEMPWARN         CLB_PRTY04 // Over temp warning from Power board

#define CANMSG_WHEELSPD         CLC_PRTY01 // Wheel speeds reported by Motors board
#define CANMSG_WHEELANG         CLC_PRTY02 // Wheel (steering) angle reported by Motors board
#define CANMSG_ODOMETRY         CLC_PRTY03 // Odometry reported by Motors board
#define CANMSG_OBSDIST          CLC_PRTY04 // Distance to in path obstacles reported by Collision Avoidance board
#define CANMSG_DESTRAJ          CLC_PRTY05 // Desired Wheel-Angle and Wheel-Speed reported by Jetson

#define CANMSG_BATTWARN         CLD_PRTY01 // Low battery warning reported by power board
#define CANMSG_NAVRPY           CLD_PRTY02 // Roll-Pitch-Yaw navigation data reported by IMU/GPS board
#define CANMSG_NAVLONLAT        CLD_PRTY03 // Longitude-Latitude navigation data reported by IMU/GPS board
#define CANMSG_START            CLD_PRTY04 // Start buton has been pushed. The CAR is ready to race.

#define NUM_OF_ECAN_BUFFERS 16  //only used for memory allocation

    //This is the ECAN message buffer declaration.
    extern unsigned int ecan1MsgBuffer[NUM_OF_ECAN_BUFFERS][8];

    //Public functions
    void CAN1Init();
    int CAN1IsTransmitComplete();
    int CAN1Transmit(unsigned int SID, unsigned int length, unsigned int* data);
    int CAN1TransmitRemote(unsigned int SID, unsigned int length);
    void CAN1EmptyReceiveBuffer(int index);
    void CAN1CheckReceiveBuffer();

    //ISRs
    void __attribute__((__interrupt__, no_auto_psv)) _DMA1Interrupt(void);

#ifdef	__cplusplus
}
#endif

#endif	/* CAN_H */

