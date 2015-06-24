/*
 * Insert documentation
 */

#include <stdlib.h>
#include <stdint.h>
//#include <stdbool.h>

class MessageParser
{
private:
  char parse_buffer[5];
  int steering;
  int throttle;
public:
  MessageParser();
  bool parse_message(uint8_t *buffer, int length, int8_t &cnt);
  int get_steering();
  int get_throttle();
private:
  bool next_int(int &retval, uint8_t *buffer, int &idx, int length);
};
