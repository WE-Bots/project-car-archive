 #ifndef _STDINT_H_
 #define _STDINT_H_

/*****************************************************
**Each message includes a 2 byte SID to identify the**
**  message prior to the data bytes and a checksum  **
**             byte after the data bytes            **
*****************************************************/
 #define _EMSTOP_ 0x0010  //Emergency stop message
 /**1 byte message; If any bits are set then trigger an
 *emergency stop, otherwise if the byte sent is 0 then
 *proceed as normal.
 **/
 #define _COLEMG_ 0x0011  //Imminent collision detected
 //1 byte message; 0xFFFF: stop | 0x0000: go
 #define _WHLSPD_ 0x0020  //Wheel speed
 /** 8 byte message; 2 byte float for each wheel; each float
 *represents speed in m/s; bytes are received in order
 *Front-Right wheel, Front-Left wheel, Back-Right wheel,
 *Back-Left wheel.
 **/
 #define _ODOMET_ 0x0022  //Odometry message
 /** 8 byte message; 2 byte float for each wheel; each float
 *represents distance in meters; received in order Front-Right
 *wheel, Front-Left wheel, Back-Right wheel, Back-Left wheel.
 **/
 #define _DESTRAJ_ 0x0024 //Desired trajectory message sent out
 /** 4 byte message; 2 byte float for speed in m/s and 2 byte
 *float for angle in degrees; send order speed, angle.
 **/
 #define _START_ 0x0033   //Start message
 // 2 byte message (SID);

//Macros to detect endianness when using gcc compiler
 #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN
 #define __LSB_FIRST_MODE__
 #elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN
 #define __MSB_FIRST_MODE__
 #endif

#include <stdlib.h>
#include <stdint.h>
//#include <stdbool.h>

class MessageParser
{
private:
  char parse_buffer[8];
  bool _emerg_stop, _coll_stop, _start;
  short *_wheel_speed[4], *_odometer[4];
  char _calc_traj[2];
public:
  MessageParser();
  bool parse_message(uint8_t *buffer, int length, int8_t &cnt);
  int get_wheel_speed();
  int get_odometer();
  bool get_Start();
private:
  bool parse_int_data(short** parsed_data, uint8_t *buffer, int &idx, int length, int end);
};
#endif  //#ifndef _STDINT_H_
