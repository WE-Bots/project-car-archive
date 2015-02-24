

//uncomment for control communication type
//#define I2C
#define BLUETOOTH
#define EMERGENCY_STOP

#include <Wire.h>
#include <Servo.h> 

Servo motor;
Servo steer;

int steering_angle=0; 
int base_speed=0;
int speed_error=0;
int real_speed=0;
int accumulated_speed_error=0;
boolean _ready=false;
boolean emergency_stop=true;
unsigned long timer=0;

void setup() 
{
#ifdef I2C
  Wire.begin(4);                 //join as device #4
  Wire.onReceive(control);
  Wire.onRequest(check);
#endif

  Serial.begin(9600);

  motor.attach(9);     // range 0-180
  steer.attach(10);    // range 50-120   below 90 turns right
  while(millis()<2000)
  {
  }
  timer=millis();
  _ready=true;
} 

void loop() 
{ 
#ifdef BLUETOOTH
  if(Serial.available()>0)
  {
    base_speed=Serial.parseInt();
    //steering_angle=Serial.parseInt();
  }
#endif

#ifdef EMERGENCY_STOP
  if (Serial.available()>0)
  {
    emergency_stop=(Serial.read()-48);    // test this is still working. might be able to use parseint and get real value rather than asci
    timer=millis();
  }
  if ((millis()-timer)>=700)
  {
    emergency_stop=true;
  }
#endif

  //emergency_stop=false;
  if (emergency_stop)
  {
    motor.write(90);
    steer.write(90);
  }
  else
  {
    motor.write(90-constrain(base_speed+speed_error,-90,90));
    steer.write(90-constrain(steering_angle,-30,40));
  }
} 

#ifdef I2C
void control (int num)
{
  //read base speed, speed error and stearing angle
  base_speed=Wire.read();
  steering_angle=Wire.read();
  real_speed=Wire.read();
  if (!emergency_stop)
  {
    accumulated_speed_error+=base_speed-real_speed;
  }
  speed_error=constrain(0.5*(base_speed-real_speed)+0.5*accumulated_speed_error,-10,10);
}

void check()
{
  Wire.write(_ready);
}
#endif


