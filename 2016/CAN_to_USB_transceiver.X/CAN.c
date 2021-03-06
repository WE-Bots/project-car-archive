/*
 * File:   CAN.c
 * Author: Kevin
 *
 * Created on May 15, 2016, 12:37 PM
 */

#include "CAN.h"
#include "UART.h"

/*Uncomment the board being used*/
#define CAN2USB
//#define MOTOR_CONTROLLER
//#define COLLISION_AVOIDANCE
//#define IMU_GPS



//This is the ECAN message buffer declaration. Note the buffer alignment and the start address.
unsigned int ecan1MsgBuffer[NUM_OF_ECAN_BUFFERS][8]
__attribute__((address(0x7000), aligned(NUM_OF_ECAN_BUFFERS * 16)));

volatile unsigned int CANReceiveCount = 0;
#ifdef MOTOR_CONTROLLER
extern char manualEStop;
extern char collisionEmergency;
extern float desSpeed;
extern float desAngle;
extern char start;
#endif
#ifdef COLLISION_AVOIDANCE
extern char manualEStop;
extern char remoteEStop;
extern char start;
#endif

/*DMA Channel 1 ISR*/
void __attribute__((__interrupt__, no_auto_psv)) _DMA1Interrupt(void)
{
    //init uart transfer
    CANReceiveCount++;
    IFS0bits.DMA1IF = 0; /* Clear interrupt flag */
}

void CAN1Init()
{
    //remap IO for CAN module
#ifdef CAN2USB
    TRISDbits.TRISD1 = 0;
    LATDbits.LATD1 = 1; //put tranceiver in standby mode
    RPINR26bits.C1RXR = 0b1000011;
    RPOR1bits.RP66R = 0b001110;
    TRISDbits.TRISD2=0;
    
#endif
#ifdef MOTOR_CONTROLLER
    RPINR26bits.C1RXR = 0b1001001;
    RPOR0bits.RP65R = 0b001110;
    TRISDbits.TRISD8 = 0;
    LATDbits.LATD8 = 1; //put tranceiver in standby mode
#endif
#ifdef COLLISION_AVOIDANCE
    RPINR26bits.C1RXR = 0b1010011;
    RPOR5bits.RP84R = 0b001110;
    TRISEbits.TRISE0 = 0;
    LATEbits.LATE0 = 1; //put tranceiver in standby mode
#endif

    //put module in configuration mode
    C1CTRL1bits.REQOP = 4;
    while (C1CTRL1bits.OPMODE != 4);

    C1CTRL1bits.WIN = 1;

    /*Set bit timing for 230kHz baud rate*/
    C1CFG1bits.BRP = 0;
    C1CFG1bits.SJW = 1;
    C1CFG2bits.SEG1PH = 2;
    C1CFG2bits.SEG2PHTS = 1;
    C1CFG2bits.SEG2PH = 1;
    C1CFG2bits.PRSEG = 1;
    C1CFG2bits.SAM = 1;
    C1CTRL1bits.CANCKS = 0;

    /*Use acceptance filter mask 0 for all filters*/
    C1FMSKSEL1 = 0;
    C1FMSKSEL2 = 0;
    /*Set acceptance filter mask 0 to use all filter bits*/
    C1RXM0SIDbits.SID = 0x7FF;
    /*Set acceptance filters to match SIDs*/
    C1RXF1SIDbits.SID = CANMSG_ESTOP;
    C1RXF2SIDbits.SID = CANMSG_COLLEMERG;
    C1RXF3SIDbits.SID = CANMSG_BRNOUTWARN;
    C1RXF4SIDbits.SID = CANMSG_TEMPWARN;
    C1RXF5SIDbits.SID = CANMSG_WHEELSPD;
    C1RXF6SIDbits.SID = CANMSG_WHEELANG;
    C1RXF7SIDbits.SID = CANMSG_ODOMETRY;
    C1RXF8SIDbits.SID = CANMSG_OBSDIST;
    C1RXF9SIDbits.SID = CANMSG_DESTRAJ;
    C1RXF10SIDbits.SID = CANMSG_BATTWARN;
    C1RXF11SIDbits.SID = CANMSG_NAVRPY;
    C1RXF12SIDbits.SID = CANMSG_NAVLONLAT;
    C1RXF13SIDbits.SID = CANMSG_START;
    C1RXF14SIDbits.SID = 0x07FF; //unused
    C1RXF15SIDbits.SID = 0x07FF; //unused
    /*Set filters to check for for standard frames*/
    C1RXM0SIDbits.MIDE = 1;
    C1RXF1SIDbits.EXIDE = 0;
    C1RXF2SIDbits.EXIDE = 0;
    C1RXF3SIDbits.EXIDE = 0;
    C1RXF4SIDbits.EXIDE = 0;
    C1RXF5SIDbits.EXIDE = 0;
    C1RXF6SIDbits.EXIDE = 0;
    C1RXF7SIDbits.EXIDE = 0;
    C1RXF8SIDbits.EXIDE = 0;
    C1RXF9SIDbits.EXIDE = 0;
    C1RXF10SIDbits.EXIDE = 0;
    C1RXF11SIDbits.EXIDE = 0;
    C1RXF12SIDbits.EXIDE = 0;
    C1RXF13SIDbits.EXIDE = 0;
    C1RXF14SIDbits.EXIDE = 0;
    C1RXF15SIDbits.EXIDE = 0;
    /*Set filter to use buffers to store messages*/
    C1BUFPNT1bits.F1BP = 1;
    C1BUFPNT1bits.F2BP = 2;
    C1BUFPNT1bits.F3BP = 3;
    C1BUFPNT2bits.F4BP = 4;
    C1BUFPNT2bits.F5BP = 5;
    C1BUFPNT2bits.F6BP = 6;
    C1BUFPNT2bits.F7BP = 7;
    C1BUFPNT3bits.F8BP = 8;
    C1BUFPNT3bits.F9BP = 9;
    C1BUFPNT3bits.F10BP = 10;
    C1BUFPNT3bits.F11BP = 11;
    C1BUFPNT4bits.F12BP = 12;
    C1BUFPNT4bits.F13BP = 13;
    C1BUFPNT4bits.F14BP = 14;
    C1BUFPNT4bits.F15BP = 15;
    /*Enable the first 13 filters*/
    C1FEN1 = 0x3FFE;
    /*Use 16 DMA buffers*/
    C1FCTRLbits.DMABS = 4;
    /*No FIFO buffers*/
    C1FCTRLbits.FSA = 15;
    /*No devicenet filtering*/
    C1CTRL2bits.DNCNT = 0;
    /*Set up DMA module*/
    DMA0CONbits.SIZE = 0;
    DMA0CONbits.DIR = 1;
    DMA0CONbits.AMODE = 2;
    DMA0CONbits.MODE = 0;
    DMA0REQ = 0b01000110;
    DMA0CNT = 7;
    DMA0PAD = (volatile unsigned int) &C1TXD;
    DMA0STAL = (unsigned int) ecan1MsgBuffer;
    DMA0STAH = 0;
    DMA0CONbits.CHEN = 1;
    DMA1CONbits.SIZE = 0;
    DMA1CONbits.DIR = 0;
    DMA1CONbits.AMODE = 2;
    DMA1CONbits.MODE = 0;
    DMA1REQ = 0b00100010;
    DMA1CNT = 7;
    DMA1PAD = (volatile unsigned int) &C1RXD;
    DMA1STAL = (unsigned int) ecan1MsgBuffer;
    DMA1STAH = 0;
    DMA1CONbits.CHEN = 1;

    /*Setup Tx buffer*/
    C1CTRL1bits.WIN = 0;
    C1TR01CONbits.TXEN0 = 1; //Set buffer 0 to Tx
    C1TR01CONbits.RTREN0 = 0; //No auto remote transmit
    C1TR01CONbits.TX0PRI = 3; //Set priority to highest

    /*Initialize the interrupts*/
    IEC0bits.DMA0IE = 0;
    IEC0bits.DMA1IE = 1;

    /*Put module in normal mode*/
    C1CTRL1bits.REQOP = 0;
    while (C1CTRL1bits.OPMODE != 0);
#ifdef CAN2USB
    LATDbits.LATD1 = 0; //put tranceiver in normal mode
#endif
#ifdef MOTOR_CONTROLLER
    LATDbits.LATD8 = 0; //put tranceiver in normal mode
#endif
#ifdef COLLISION_AVOIDANCE
    LATEbits.LATE0 = 0; //put tranceiver in normal mode
#endif
}

int CAN1IsTransmitComplete()
{
    return (C1TR01CONbits.TXREQ0 != 1);
}

int CAN1Transmit(unsigned int SID, unsigned int length, unsigned int* data)
{
    /*check the length is within range and if the buffer is free*/
    if (length > 8 || !CAN1IsTransmitComplete())
    {
        return 0;
    }
    /* Write to message buffer 0 */
    /* CiTRBnSID = 0bxxx1 0010 0011 1100
    IDE = 0b0
    SRR = 0b0
    SID<10:0>= 0b100 1000 1111 */
    ecan1MsgBuffer[0][0] = ((SID & 0x07FF) << 2) & 0xFFFC;
    //ecan1MsgBuffer[0][0] = 0x123C;
    /* CiTRBnEID = 0bxxxx 0000 0000 0000
    EID<17:6> = 0b0000 0000 0000 */
    ecan1MsgBuffer[0][1] = 0x0000;
    /* CiTRBnDLC = 0b0000 0000 xxx0 1111
    EID<17:6> = 0b000000
    RTR = 0b0
    RB1 = 0b0
    RB0 = 0b0
    DLC = 0b1111 */
    ecan1MsgBuffer[0][2] = (length) & 0x000F;
    //ecan1MsgBuffer[0][2] = 0x0008;
    /* Write message data bytes */
    int i;
    for (i = 0; i < (length+1) / 2; i++)
    {
        ecan1MsgBuffer[0][i + 3] = data[i];
    }
    //    ecan1MsgBuffer[0][3] = 0xabcd;
    //    ecan1MsgBuffer[0][4] = 0xabcd;
    //    ecan1MsgBuffer[0][5] = 0xabcd;
    //    ecan1MsgBuffer[0][6] = 0xabcd;
    /* Request message buffer 0 transmission */
    C1TR01CONbits.TXREQ0 = 0x1;
    return 1;
}

int CAN1TransmitRemote(unsigned int SID, unsigned int length)
{
    /*check if the buffer is free*/
    if (!CAN1IsTransmitComplete())
    {
        return 0;
    }
    /*check the length is within range*/
    if (length > 8)
    {
        length = 8;
    }
    /* Write to message buffer 0 */
    /* CiTRBnSID = 0bxxx1 0010 0011 1100
    IDE = 0b0
    SRR = 0b1
    SID<10:0>= 0b100 1000 1111 */
    //    ecan1MsgBuffer[0][0] = ((SID & 0x07ff) << 2) | 0x2 & 0xFFFE;
    ecan1MsgBuffer[0][0] = 0x123E;
    /* CiTRBnEID = 0bxxxx 0000 0000 0000
    EID<17:6> = 0b0000 0000 0000 */
    ecan1MsgBuffer[0][1] = 0x0000;
    /* CiTRBnDLC = 0b0000 0000 xxx0 0000
    EID<17:6> = 0b000000
    RTR = 0b0
    RB1 = 0b0
    RB0 = 0b0
    DLC = 0b0000 */
    //    ecan1MsgBuffer[0][2] = length & 0x000F;
    ecan1MsgBuffer[0][2] = 0x0000;
    /* Request message buffer 0 transmission */
    C1TR01CONbits.TXREQ0 = 0x1;
    return 1;
}

void CAN1EmptyReveiveBuffer(int index)
{
#ifdef CAN2USB
    char str[13];
    if (index > 0 && index < 16)
    {
        str[0] = '<';
        str[1] = (char) ((ecan1MsgBuffer[index][0] & 0x1FFC) >> 2);
        str[2] = (char) ((ecan1MsgBuffer[index][0] & 0x1FFC) >> 10);
        int i;
        for (i = 0; i < (ecan1MsgBuffer[index][2] & 0x000F); i++)
        {
            if (i & 1)
                str[3 + i] = (char) (ecan1MsgBuffer[index][3 + i / 2] >> 8);
            else
                str[3 + i] = (char) ecan1MsgBuffer[index][3 + i / 2];
        }
        /*Calculate checksum*/
        int j;
        for (j = 0; j < i + 2; j++)
        {
            str[3 + i] = str[j + 1];
        }
        str[4 + i] = '>';
        UART1WriteStr(str, i + 5);
    }
#endif
}

void CAN1CheckReceiveBuffer()
{
    while (CANReceiveCount)
    {
        CANReceiveCount--;
        if (C1RXFUL1bits.RXFUL1)
        {
#ifdef CAN2USB
            CAN1EmptyReveiveBuffer(1);
#endif
#ifdef MOTOR_CONTROLLER
            manualEStop = ecan1MsgBuffer[1][3] & 0x1;
#endif
#ifdef COLLISION_AVOIDANCE
            manualEStop = ecan1MsgBuffer[1][3] & 0x1;
            remoteEStop = ecan1MsgBuffer[1][3] & 0x2;
#endif
            C1RXFUL1bits.RXFUL1 = 0;
        }
        else if (C1RXFUL1bits.RXFUL2)
        {
#ifdef CAN2USB
            CAN1EmptyReveiveBuffer(2);
#endif
#ifdef MOTOR_CONTROLLER
            collisionEmergency = ecan1MsgBuffer[2][3]&0xff;
#endif
            C1RXFUL1bits.RXFUL2 = 0;
        }
        else if (C1RXFUL1bits.RXFUL3)
        {
#ifdef CAN2USB
            CAN1EmptyReveiveBuffer(3);
#endif
            C1RXFUL1bits.RXFUL3 = 0;
        }
        else if (C1RXFUL1bits.RXFUL4)
        {
#ifdef CAN2USB
            CAN1EmptyReveiveBuffer(4);
#endif
            C1RXFUL1bits.RXFUL4 = 0;
        }
        else if (C1RXFUL1bits.RXFUL5)
        {
#ifdef CAN2USB
            CAN1EmptyReveiveBuffer(5);
#endif
            C1RXFUL1bits.RXFUL5 = 0;
        }
        else if (C1RXFUL1bits.RXFUL6)
        {
#ifdef CAN2USB
            CAN1EmptyReveiveBuffer(6);
#endif
            C1RXFUL1bits.RXFUL6 = 0;
        }
        else if (C1RXFUL1bits.RXFUL7)
        {
#ifdef CAN2USB
            CAN1EmptyReveiveBuffer(7);
#endif
            C1RXFUL1bits.RXFUL7 = 0;
        }
        else if (C1RXFUL1bits.RXFUL8)
        {
#ifdef CAN2USB
            CAN1EmptyReveiveBuffer(8);
#endif
            C1RXFUL1bits.RXFUL8 = 0;
        }
        else if (C1RXFUL1bits.RXFUL9)
        {
#ifdef CAN2USB
            CAN1EmptyReveiveBuffer(9);
#endif
#ifdef MOTOR_CONTROLLER
            desSpeed = ecan1MsgBuffer[9][3];
            desAngle = ecan1MsgBuffer[9][4];
#endif
            C1RXFUL1bits.RXFUL9 = 0;
        }
        else if (C1RXFUL1bits.RXFUL10)
        {
#ifdef CAN2USB
            CAN1EmptyReveiveBuffer(10);
#endif
            C1RXFUL1bits.RXFUL10 = 0;
        }
        else if (C1RXFUL1bits.RXFUL11)
        {
#ifdef CAN2USB
            CAN1EmptyReveiveBuffer(11);
#endif
            C1RXFUL1bits.RXFUL11 = 0;
        }
        else if (C1RXFUL1bits.RXFUL12)
        {
#ifdef CAN2USB
            CAN1EmptyReveiveBuffer(12);
#endif
            C1RXFUL1bits.RXFUL12 = 0;
        }
        else if (C1RXFUL1bits.RXFUL13)
        {
#ifdef CAN2USB
            CAN1EmptyReveiveBuffer(13);
#endif
#ifdef MOTOR_CONTROLLER
            start = 1;
#endif
#ifdef COLLISION_AVOIDANCE
            start = 1;
#endif
            C1RXFUL1bits.RXFUL13 = 0;
        }
        else if (C1RXFUL1bits.RXFUL14)
        {
#ifdef CAN2USB
            CAN1EmptyReveiveBuffer(14);
#endif
            C1RXFUL1bits.RXFUL14 = 0;
        }
        else if (C1RXFUL1bits.RXFUL15)
        {
#ifdef CAN2USB
            CAN1EmptyReveiveBuffer(15);
#endif
            C1RXFUL1bits.RXFUL15 = 0;
        }
    }
}