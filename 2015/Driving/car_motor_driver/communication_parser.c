#include <string.h>

class Car_Comms {
  static boolean get_value(uint8_t &id, int16_t &value) {
    // Reserve return value
    int16_t retval = 0;
    uint_t retid;
    // Get id
    retid = Serial.parseInt();
    // Get value
    retval = Serial.parseInt();
    // Get checksum
    int16_t check = Serial.parseInt();
    // Compare check sum
    if ((id + retval) != check)
      return false;
    id = retid;
    value = retval;
    return true;
  }

  static bool send_value(unit8_t id, int16_t value) {
    Serial.print(id);
    Serial.print(',');
    Serial.print(value);
    Serial.print(',');
    Serial.print(id+value);
  }
}
