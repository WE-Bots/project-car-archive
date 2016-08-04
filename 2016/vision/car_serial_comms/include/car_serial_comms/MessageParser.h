/*
 * Insert documentation
 */

 #ifndef _STDINT_H_
 #define _STDINT_H_

 #define _EMSTOP_ 0x21  //Emergency stop message
 /**1 byte message; If any bits are set then trigger an
 *emergency stop, otherwise if the byte sent is 0 then
 *proceed as normal.
 **/
 #define _COLEMG_ 0x22  //Imminent collision detected
 //1 byte message; 0xFFFF: stop | 0x0000: go
 #define _WHLSPD_ 0x31  //Wheel speed
 /** 8 byte message; 2 byte float for each wheel; each float
 *represents speed in m/s; bytes are received in order
 *Front-Right wheel, Front-Left wheel, Back-Right wheel,
 *Back-Left wheel.
 **/
 #define _ODOMET_ 0x33  //Odometry message
 /** 8 byte message; 2 byte float for each wheel; each float
 *represents distance in meters; received in order Front-Right
 *wheel, Front-Left wheel, Back-Right wheel, Back-Left wheel.
 **/
 #define _OBSDIST_ 0x34 //Distance of obstacles message
/** 7 byte message; ONLY READ FIRST 3 bytes; 1 byte int for
*each ultrasonic sensor (three ultrasonic sensors); each int
*represents distance in cm from obstacle; bytes are received
*in order front, right, left sensor.
**/
 #define _DESTRAJ_ 0x35 //Desired trajectory message sent out
 /** 4 byte message; 2 byte float for speed in m/s and 2 byte
 *float for angle in degrees; send order speed, angle.
 **/

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
  char parse_buffer[5];
  int steering;
  int throttle;
  char _sid[2];   //Holds the id of the incoming message
public:
  MessageParser();
  bool parse_message(uint8_t *buffer, int length, int8_t &cnt);
  int get_steering();
  int get_throttle();
private:
  bool next_int(int &retval, uint8_t *buffer, int &idx, int length);
};

#endif  //#ifndef _STDINT_H_
