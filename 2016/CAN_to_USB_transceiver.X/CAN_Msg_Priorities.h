/* 
 * File:   CAN_Msg_Priorities.h
 * Author: DJacques
 *
 * Created on June 9, 2016, 7:58 PM
 */

#ifndef CAN_MSG_PRIORITIES_H
#define	CAN_MSG_PRIORITIES_H

#ifdef	__cplusplus
extern "C"
{
#endif
/************************************
 *  Define CAN message headers (IDs)
 * *********************************/
// Class A messages are for Emergency/Warnings
#define CLA_PRTY01 0x0000
#define CLA_PRTY02 0x0001
#define CLA_PRTY03 0x0002
#define CLA_PRTY04 0x0003
#define CLA_PRTY05 0x0004
#define CLA_PRTY06 0x0005
#define CLA_PRTY07 0x0006
#define CLA_PRTY08 0x0007
#define CLA_PRTY09 0x0008
#define CLA_PRTY10 0x0009
#define CLA_PRTY11 0x000A
#define CLA_PRTY12 0x000B
#define CLA_PRTY13 0x000C
#define CLA_PRTY14 0x000D
#define CLA_PRTY15 0x000E
#define CLA_PRTY16 0x000F

// Class B messages are for Steering / Kinematics
#define CLB_PRTY01 0x0010
#define CLB_PRTY02 0x0011
#define CLB_PRTY03 0x0012
#define CLB_PRTY04 0x0013
#define CLB_PRTY05 0x0014
#define CLB_PRTY06 0x0015
#define CLB_PRTY07 0x0016
#define CLB_PRTY08 0x0017
#define CLB_PRTY09 0x0018
#define CLB_PRTY10 0x0019
#define CLB_PRTY11 0x001A
#define CLB_PRTY12 0x001B
#define CLB_PRTY13 0x001C
#define CLB_PRTY14 0x001D
#define CLB_PRTY15 0x001E
#define CLB_PRTY16 0x001F

// Class C messages are for Telemetry / Mapping data
#define CLC_PRTY01 0x0020
#define CLC_PRTY02 0x0021
#define CLC_PRTY03 0x0022
#define CLC_PRTY04 0x0023
#define CLC_PRTY05 0x0024
#define CLC_PRTY06 0x0025
#define CLC_PRTY07 0x0026
#define CLC_PRTY08 0x0027
#define CLC_PRTY09 0x0028
#define CLC_PRTY10 0x0029
#define CLC_PRTY11 0x002A
#define CLC_PRTY12 0x002B
#define CLC_PRTY13 0x002C
#define CLC_PRTY14 0x002D
#define CLC_PRTY15 0x002E
#define CLC_PRTY16 0x002F  

// Class D messages are for Telemetry / Mapping data
#define CLD_PRTY01 0x0030
#define CLD_PRTY02 0x0031
#define CLD_PRTY03 0x0032
#define CLD_PRTY04 0x0033
#define CLD_PRTY05 0x0034
#define CLD_PRTY06 0x0035
#define CLD_PRTY07 0x0036
#define CLD_PRTY08 0x0037
#define CLD_PRTY09 0x0038
#define CLD_PRTY10 0x0039
#define CLD_PRTY11 0x003A
#define CLD_PRTY12 0x003B
#define CLD_PRTY13 0x003C
#define CLD_PRTY14 0x003D
#define CLD_PRTY15 0x003E
#define CLD_PRTY16 0x003F
  
#define UNASSIGNED 0xA5A5


#ifdef	__cplusplus
}
#endif

#endif	/* CAN_MSG_PRIORITIES_H */

