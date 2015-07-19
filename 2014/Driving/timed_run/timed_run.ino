#include <Servo.h>
Servo motor;
Servo stear;
unsigned long time=0;

void setup()
{
  motor.attach(9);
  stear.attach(3);
}

void loop()
{
    time=millis();
    if (time<7000 && time>2000)
    {
      motor.write(65);
    }
    else
    {
      motor.write(90);
    }
    stear.write(90);
}
