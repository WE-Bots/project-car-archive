/*
 * Insert documentation
 */

 #ifndef _STDINT_H_
 #define _STDINT_H_

//Macros to detect endianness when using gcc compiler
 #if __BYTE_ORDER__ = __ORDER_LITTLE_ENDIAN
 #define __LSB_FIRST_MODE__
 #elif __BYTE_ORDER__ = __ORDER_BIG_ENDIAN
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
