#include "CAN.h"

void CANInit()
{
    //remap IO for CAN module

    //put module in configure mode
    C1CTRL1bits.REQOP = 4;
    while (C1CTRL1bits.OPMODE != 4);
    C1CTRL1bits.WIN = 0;

    //Set up the CAN module for 250kbps speed with 10 Tq per bit.
    C1CFG1 = 0x47; // BRP = 8 SJW = 2 Tq
    C1CFG2 = 0x2D2;
    C1FCTRL = 0xC01F; // No FIFO, 32 Buffers

    //Assign 32x8word Message Buffers for ECAN1 in device RAM. This example uses DMA0 for TX.
    DMA0CONbits.SIZE = 0x0;
    DMA0CONbits.DIR = 0x1;
    DMA0CONbits.AMODE = 0x2;
    DMA0CONbits.MODE = 0x0;
    DMA0REQ = 70;
    DMA0CNT = 7;
    DMA0PAD = (volatile unsigned int) &C1TXD;
    DMA0STAL = (unsigned int) &ecan1MsgBuf;
    DMA0STAH = (unsigned int) &ecan1MsgBuf;
    DMA0CONbits.CHEN = 0x1;

    //Configure Message Buffer 0 for Transmission and assign priority
    C1TR01CONbits.TXEN0 = 0x1;
    C1TR01CONbits.TX0PRI = 0x3;

    //put module in normal mode
    C1CTRL1bits.REQOP = 0;
    while (C1CTRL1bits.OPMODE != 0);
}

int CANIsTransmitComplete()
{
    return (C1TR01CONbits.TXREQ0 != 1);
}

void CANTransmit(unsigned int SID, unsigned int length, unsigned int* data)
{
    /* Write to message buffer 0 */
    /* CiTRBnSID = 0bxxx1 0010 0011 1100
    IDE = 0b0
    SRR = 0b0
    SID<10:0>= 0b100 1000 1111 */
    //ecan1MsgBuf[0][0] = (SID&0x07ff)<<2
    ecan1MsgBuf[0][0] = 0x123C;
    /* CiTRBnEID = 0bxxxx 0000 0000 0000
    EID<17:6> = 0b0000 0000 0000 */
    ecan1MsgBuf[0][1] = 0x0000;
    /* CiTRBnDLC = 0b0000 0000 xxx0 1111
    EID<17:6> = 0b000000
    RTR = 0b0
    RB1 = 0b0
    RB0 = 0b0
    DLC = 0b1111 */
    //ecan1MsgBuf[0][2] = length&0x0008
    ecan1MsgBuf[0][2] = 0x0008;
    /* Write message data bytes */
    /*for (int i = 0; i < (length+1)/2; i)
    {
        ecan1MsgBuf[0][i+3] = data[i];
    }*/
    ecan1MsgBuf[0][3] = 0xabcd;
    ecan1MsgBuf[0][4] = 0xabcd;
    ecan1MsgBuf[0][5] = 0xabcd;
    ecan1MsgBuf[0][6] = 0xabcd;
    /* Request message buffer 0 transmission */
    C1TR01CONbits.TXREQ0 = 0x1;
}

void CANTransmitRemote(unsigned int SID)
{
    /* Write to message buffer 0 */
    /* CiTRBnSID = 0bxxx1 0010 0011 1100
    IDE = 0b0
    SRR = 0b0
    SID<10:0>= 0b100 1000 1111 */
    //ecan1MsgBuf[0][0] = (SID&0x07ff)<<2;
    ecan1MsgBuf[0][0] = 0x123C;
    /* CiTRBnEID = 0bxxxx 0000 0000 0000
    EID<17:6> = 0b0000 0000 0000 */
    ecan1MsgBuf[0][1] = 0x0000;
    /* CiTRBnDLC = 0b0000 0000 xxx0 1111
    EID<17:6> = 0b000000
    RTR = 0b1
    RB1 = 0b0
    RB0 = 0b0
    DLC = 0b0000 */
    ecan1MsgBuf[0][2] = 0x0200;
    /* Request message buffer 0 transmission */
    C1TR01CONbits.TXREQ0 = 0x1;
}