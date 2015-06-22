/*
 * Insert documentation
 */

MessageParser::MessageParser()
{
  return;
}

bool MessageParser::parse_message(const uint8_t *buffer, int length)
{
  // Get data out of the buffer
  int idx = 0;
  while (buffer[idx] != '<' && idx < (length-1))
    idx++;
  if (buffer[idx] != '<')
    return false;

  if (next_int(steering,buffer,idx,length)
  {
    if (next_int(throttle,buffer,idx,length))
    {
      if (idx==length || buffer[idx] != '<')
        return false;
      idx++;
      for (int i=0; i<(length-idx); i++)
        buffer[i] = buffer[i+idx];
      return true;
    }
  }
  return false;
}

bool MessageParser::next_int(int &retval, uint8_t *buffer, int &idx, int length)
{
  // Move subsequent chars into parse buffer
  if (buffer[idx]!='-' && (buffer[idx]<'0' || buffer[idx]>'9'))
    return false;
  int parse_idx=0;
  parse_buffer[parse_idx] = buffer[idx];
  idx++;

  // Get the rest of the number
  while (buffer[idx]>='0' && buffer[idx]<='9' && idx<=(length-1) && parse_idx<4)
    parse_buffer[++parse_idx] = buffer[idx++];

  // Check if a valid number (only if "-", otherwise OK)
  if (parse_idx==0 && parse_buffer[0]=='-')
    return false;

  // Add null terminator
  parse_buffer[++parse_idx] = '\0';

  // Get the number out
  retval = atoi(parse_buffer);

  // Got a reading!
  return true;
}
