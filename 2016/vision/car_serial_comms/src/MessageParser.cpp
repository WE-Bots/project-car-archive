#include "car_serial_comms/MessageParser.h"

//EZbake constructor
MessageParser::MessageParser() {
    _emerg_stop = false;
    _coll_stop = false;
    _start = false;
}

/**
* parse_message(): Read a CAN message byte by byte from
* buffer.
*
* @buffer: buffer from which byte message is read
* @length: length of the buffer being used to read
* @cnt:
* @return
*/
bool MessageParser::parse_message(uint8_t *buffer, int length, int8_t &cnt)
{
    // Get data out of the buffer
    int idx = 0, inc, data_end, offset;
    char sid_buffer[2], checksum_byte;
    short* sid_val;
    while (buffer[idx] != '<' && idx < (length-1))
      idx++;
    offset = idx;
    if (buffer[idx] != '<')
      return false;
    if (++idx == (length-1))
      return false;

  #ifdef __MSB_FIRST_MODE__
  for (int i = 0; i < 2; i++)
    sid_buffer[i] = buffer[idx++];

  sid_val = (short*) sid_buffer;

  #elif __LSB_FIRST_MODE__
  for (int i = 1; i >= 0; i--)
    sid_buffer[i] = buffer[idx++];

  char sid_temp[2];
  sid_temp[0] = sid_buffer[1];
  sid_temp[1] = sid_buffer[0];
  sid_val = (short*) sid_buffer;
  #endif

  if(*sid_val == _EMSTOP_)
    data_end = 1;
    //TODO: Parse and find out if there is an emergency stop
  else if(*sid_val == _COLEMG_)
    data_end = 1;
    //TODO: Parse and find out if there will be a collision
  else if(*sid_val == _WHLSPD_) {
    data_end = 8;
    if (!parse_int_data(_wheel_speed, buffer, idx, length, data_end))
      return false;
  }
  else if(*sid_val == _ODOMET_) {
    data_end = 8;
    if (!parse_int_data(_odometer, buffer, idx, length, data_end))
      return false;
  }
  else if(*sid_val == _START_) {
    if(idx<=(length-2))
        _start = true;
    else
      return false;
  }

  for (int i = 0; i < length - 3; i++)
    checksum_byte += buffer[i + offset];

  if(buffer[idx++] != checksum_byte)
    return false;

  if (idx>=length || buffer[idx] != '>')
    return false;
  idx++;
  for (int i=0; i<(length-idx); i++)
    buffer[i] = buffer[i+idx];
  cnt = cnt - idx; // Next empty slot / new size
  return true;
}

/** get_wheel_speed(): Obtain the average speed of the wheels
* @return: Calculated average of wheels speed
*/
int MessageParser::get_wheel_speed() {
  short avg_wheel_speed = 0;
  for (int i = 0; i < 4; i++)
    avg_wheel_speed += *_wheel_speed[i];
  avg_wheel_speed /= 4;
  return avg_wheel_speed;
}

/** get_odometer(): Obtain odometer distance info
* @return: Total odometer distance
*/
int MessageParser::get_odometer() {
  short avg_wheel_odom = 0;
  for (int i = 0; i < 4; i++)
    avg_wheel_odom += *_odometer[i];
  avg_wheel_odom /= 4;
  return avg_wheel_odom;
}

bool MessageParser::get_Start() { return _start; }

bool MessageParser::parse_int_data(short** parsed_data, uint8_t *buffer, int &idx, int length, int end)
{
  // Move subsequent chars into parse buffer
  if (buffer[idx]!='-' && (buffer[idx]<48 || buffer[idx]>57))
    return false;
  int inc = 0, data_idx = 0;
  int parse_idx;

  #ifdef __LSB_FIRST_MODE__
  parse_idx=end;
  inc=-1;
  #elif __MSB_FIRST_MODE__
  parse_idx=0;
  inc=1;
  #endif

  parse_buffer[parse_idx] = buffer[idx];
  idx++;

  // Get the rest of the number
  while ((buffer[idx]>=48) &&
         (buffer[idx]<=57) &&
         (data_idx<end)    &&
         (idx<=(length-1)) &&
         //Based on endian mode, need to Check
         //parse index from either end
         ((parse_idx<(length-5)) ||
         (parse_idx>(length-5))))
         {
           parse_idx += inc;
           parse_buffer[parse_idx] = buffer[idx++];
         }

  short temp_data;
  // Check if a valid number (only invalid if "-", otherwise OK)
  #ifdef __MSB_FIRST_MODE__
  if (parse_idx==0 && parse_buffer[0]=='-')
    return false;

  // Get the number out
  for (int i=0,j=0; i < 4; i++){
    temp_data = parse_buffer[j++];
    temp_data << 8;
    temp_data += parse_buffer[j++];
    parsed_data[i++] = temp_data;
    temp_data = 0;
  }
  #elif __LSB_FIRST_MODE__
  if (parse_idx==(parse_buffer.length - 1) && parse_buffer[parse_buffer.length-1]=='-')
    return false;

  // Get the number out
  for (int i=0,j=1; i < 4; i++){
    temp_data = parse_buffer[j--];
    temp_data << 8;
    temp_data += parse_buffer[j];
    parsed_data[i++] = temp_data;
    temp_data = 0;
    j+=3;
  }
  #endif
  return true;
}
