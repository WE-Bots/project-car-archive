
/*
 * Insert documentation
 */

class MessageParser
{
private:
  char parse_buffer[5];
  int steering;
  int throttle;
public:
  MessageParser();
  bool parse_message(uint8_t *buffer, int length);
private:
  bool next_int(int &retval, uint8_t *buffer, int &idx, int length);
}
