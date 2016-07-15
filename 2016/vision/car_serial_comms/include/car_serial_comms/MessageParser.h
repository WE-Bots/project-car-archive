/*
 * Insert documentation
 */

 #ifndef _STDINT_H_
 #define _STDINT_H_

 #define _EMSTOP_ 0x21  //Emergency stop message
 //bit0 = 0: stop | bit0 = 1: start/restart
 #define _COLEMG_ 0x22  //Imminent collision detected
 //0xFFFF: stop | 0x0000: go
 #define _BRNOUT_ 0x23  //Main power brown out warning
 #define _OVRHT_ 0x24   //Over temp warning from power board
 #define _WHLSPD_ 0x31  //Wheel speed
 //8 byte message, 2 byte float for each wheel, bytes are
 //received in the order Front-Right wheel, Front-Left wheel,
 //Back-Right wheel, Back-Left wheel
 #define _ODOMET_ 0x33  //Odometry

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
