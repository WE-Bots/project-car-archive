#include "CAN.h"
#include <uart.h>

//This is the ECAN message buffer declaration. Note the buffer alignment and the start address.
unsigned int ecan1MsgBuffer[NUM_OF_ECAN_BUFFERS][8]
__attribute__((address(0x7000), aligned(NUM_OF_ECAN_BUFFERS * 16)));

volatile unsigned int CANRecieveCount=0;

/*DMA Channel 1 ISR*/
void __attribute__((__interrupt__)) _DMA1Interrupt(void)
{
    //init uart transfer
    CANRecieveCount++;
    IFS0bits.DMA1IF = 0; /* Clear interrupt flag */
}

void CAN1Init()
{
    //remap IO for CAN module
    RPINR26bits.C1RXR = 0b1000011;
    RPOR1bits.RP66R = 0b001110;

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
    C1RXF1SIDbits.SID = 1;
    C1RXF2SIDbits.SID = 2;
    C1RXF3SIDbits.SID = 3;
    C1RXF4SIDbits.SID = 4;
    C1RXF5SIDbits.SID = 5;
    C1RXF6SIDbits.SID = 6;
    C1RXF7SIDbits.SID = 7;
    C1RXF8SIDbits.SID = 8;
    C1RXF9SIDbits.SID = 9;
    C1RXF10SIDbits.SID = 10;
    C1RXF11SIDbits.SID = 11;
    C1RXF12SIDbits.SID = 12;
    C1RXF13SIDbits.SID = 13;
    C1RXF14SIDbits.SID = 14;
    C1RXF15SIDbits.SID = 15;
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
    /*Enable the filters*/
    C1FEN1 = 0xFFFE;
    /*Use 16 DMA buffers*/
    C1FCTRLbits.DMABS = 4;
    /*No FIFO buffers*/
    C1FCTRLbits.FSA = 15;
    /*No cevicenet filtering*/
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
    C1TR01CONbits.TXEN0 = 1; //Set beffure 0 to Tx
    C1TR01CONbits.RTREN0 = 0; //No auto remote transmit
    C1TR01CONbits.TX0PRI = 3; //Set priority to highest

    /*Initialize the interrupts*/
    IEC0bits.DMA0IE = 0;
    IEC0bits.DMA1IE = 1;


    /*Put module in normal mode*/
    C1CTRL1bits.REQOP = 0;
    while (C1CTRL1bits.OPMODE != 0);
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
    //    ecan1MsgBuffer[0][0] = ((SID & 0x07ff) << 2) & 0xFFFC;
    ecan1MsgBuffer[0][0] = 0x123C;
    /* CiTRBnEID = 0bxxxx 0000 0000 0000
    EID<17:6> = 0b0000 0000 0000 */
    ecan1MsgBuffer[0][1] = 0x0000;
    /* CiTRBnDLC = 0b0000 0000 xxx0 1111
    EID<17:6> = 0b000000
    RTR = 0b0
    RB1 = 0b0
    RB0 = 0b0
    DLC = 0b1111 */
    //    ecan1MsgBuffer[0][2] = (length - 1) & 0x000F;
    ecan1MsgBuffer[0][2] = 0x0008;
    /* Write message data bytes */
    //    for (int i = 0; i < length / 2; i++)
    //    {
    //        ecan1MsgBuffer[0][i + 3] = data[i];
    //    }
    ecan1MsgBuffer[0][3] = 0xabcd;
    ecan1MsgBuffer[0][4] = 0xabcd;
    ecan1MsgBuffer[0][5] = 0xabcd;
    ecan1MsgBuffer[0][6] = 0xabcd;
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

void CAN1EmptyReveiveBuffer()
{
    while(CANRecieveCount--)
    {
        unsigned char str[13];
        if (C1RXFUL1bits.RXFUL1)
        {
            str[0]='<';
            str[1]=(ecan1MsgBuffer[1][0] & 0x1FFC)>>2;
            str[2]=(ecan1MsgBuffer[1][0] & 0x1FFC)>>10;
            str[3]=ecan1MsgBuffer[1][3];
            str[4]=ecan1MsgBuffer[1][3]>>8;
            str[5]='>';
            str[6]='\0';
            C1RXFUL1bits.RXFUL1=0;
            putsUART1((unsigned int *)str);
        }
    }
}