#include <string.h>

/*
  class Car_Comms
  /brief
  -Sends and receives serial id-value pairs.
  -Uses and deals with 16-bit checksums. In this case it's practically sending
   data twice.
  -All methods are static. Do not instantiate.
*/
class Car_Comms {
public:
  /*
    function get_value
    /brief
    -Attempts to read an id and value using our comms protocol.
    -Reference arguments are onlt updated if a new package is completed -
     they will be unmodified if a complete, valid entry is not obtained.

    /params
    -uint8_t &id: Reference to id variable to read into
    -int16_t &value: Reference to value variable to read into

    /return
    -bool: True if a id-value pair was obtained. False otherwise.
  */
  static bool get_value(uint8_t &id, int16_t &value) {
    // Static variables remain between calls
    static char buffer[3][7]; // Three parameters, length 6 + '\0' each
    static uint charIdx[3] = {0,0,0}; // Index for each char container
    static bool buffIdx = 0; // Current container receiving data

    // Read chars until there is no data in the buffer or we find a terminator
    while (Serial.available() > 0) {
      // Check for start of package
      if (Serial.peek() == '<') {
        Serial.read(); // Eat the byte
        buffIdx = 0;
        for (int i = 0; i < 3; ++i)
          charIdx[i] = 0;
      }
      else if (Serial.peek() == ',') {
        Serial.read(); // Eat the byte
        buffer[buffIdx][charIdx[buffIdx]++] = '\0'; // Add null terminator
        buffIdx++;
      }
      else if (Serial.peek() == '>') {
        // Don't eat the byte - leave it for a check outside of the loop
        buffer[buffIdx][charIdx[buffIdx]++] = '\0'; // Add null terminator
        buffIdx++;
        break;
      }
      else // Must be a digit. Maybe check just in case?
        buffer[buffIdx][charIdx[buffIdx]++] = Serial.read();
    }

    // Check if buffer was emptied (ALWAYS check before peeking)
    if (Serial.available() == 0)
      return false; // Processed whole buffer without receiving complete packet

    // Check if terminator was reached
    if (Serial.peek() != '>')
      return false; // Not sure how you could even get here.
                    // Maybe came in after the check in the loop.

    // Reserve return value
    uint8_t retid;
    int16_t retval;
    int16_t chksum;
    // Get id
    retid = atoi(buffer[0]);
    // Get value
    retval = atoi(buffer[1]);
    // Get checksum
    chksum = atoi(buffer[2]);

    // Compare checksum
    if ((id + retval) != check)
      return false;

    // Checksum was valid, pass the variables back to the caller, return true
    id = retid;
    value = retval;
    return true;
  }

  /*
    function send_value
    /brief
    -Sends an id-value pair over serial in our format.
    -Always returns true because I can't think of a failure mechanism.

    /params
    -uint8_t id: The id to send.
    -int16_t value: The value to send.

    /return
    -bool: True if transmission was successful. Always true.
  */
  static bool send_value(unit8_t id, int16_t value) {
    Serial.print('<');
    Serial.print(id);
    Serial.print(',');
    Serial.print(value);
    Serial.print(',');
    Serial.print(id+value);
    Serial.print('>');
    return true;
  }

}
