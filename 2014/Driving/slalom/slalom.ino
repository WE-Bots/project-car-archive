//start angled to the right about 30 degrees


#include <Servo.h>
Servo motor;
Servo stear;
unsigned long time=0;
int angle=0;
boolean angle_inc=1;

void setup()
{
  motor.attach(9);
  stear.attach(10);
}

void loop()
{
    time=millis();
    if (time<8000 && time>3000)
    {
      motor.write(82);
      stear.write(90+angle);
    }
    else if (time<3000 && time>2000)
    {
      motor.write(82);
      stear.write(90+angle);
    }
    else
    {
      motor.write(90);
      stear.write(90);
    }
    if (time%80==0 && time>3000)
    {
      if (angle_inc)
      {
        angle++;
        if (angle>29)
        {
          angle_inc=0;
        }
      }
      else
      {
        angle--;
        if (angle<-29)
        {
          angle_inc=1;
        }
      }
    }
}
