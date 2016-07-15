/*
 * Insert documentation
 */

#include "car_serial_comms/MessageParser.h"

MessageParser::MessageParser()
{
  return;
}

bool MessageParser::parse_message(uint8_t *buffer, int length, int8_t &cnt)
{
  // Get data out of the buffer
  int idx = 0, inc;
  while (buffer[idx] != '<' && idx < (length-1))
    idx++;
  if (buffer[idx] != '<')
    return false;
  if (++idx == (length-1))
    return false;

  if (next_int(steering,buffer,idx,length))
  {
    idx++; // Next!
    if (next_int(throttle,buffer,idx,length))
    {
      if (idx>=length || buffer[idx] != '>')
        return false;
      idx++;
      for (int i=0; i<(length-idx); i++)
        buffer[i] = buffer[i+idx];
      cnt = cnt - idx; // Next empty slot / new size
      return true;
    }
  }
  return false;
}

int MessageParser::get_steering()
{
  return steering;
}

int MessageParser::get_throttle()
{
  return throttle;
}

bool MessageParser::next_int(int &retval, uint8_t *buffer, int &idx, int length)
{
  // Move subsequent chars into parse buffer
  if (buffer[idx]!='-' && (buffer[idx]<48 || buffer[idx]>57))
    return false;
  int inc = 0;
  
  #ifdef __LSB_FIRST_MODE__
  int parse_idx=parse_buffer.length - 1;
  inc=-1;
  #elif __MSB_FIRST_MODE__
  int parse_idx=0;
  inc=1;
  #endif

  parse_buffer[parse_idx] = buffer[idx];
  idx++;

  // Get the rest of the number
  while ((buffer[idx]>=48) &&
         (buffer[idx]<=57) &&
         (idx<=(length-1))  &&
         (parse_idx<4))
         {
           parse_idx += inc;
           parse_buffer[parse_idx] = buffer[idx++];
         }
  // Check if a valid number (only invalid if "-", otherwise OK)
  #ifdef __MSB_FIRST_MODE__
  if (parse_idx==0 && parse_buffer[0]=='-')
    return false;
  #elif __LSB_FIRST_MODE__
  if (parse_idx==(parse_buffer.length - 1) && parse_buffer[parse_buffer.length-1]=='-')
    return false;
  #endif

  // Add null terminator
  parse_buffer[++parse_idx] = '\0';

  // Get the number out
  retval = atoi(parse_buffer);

  // Got a reading!
  return true;
}
