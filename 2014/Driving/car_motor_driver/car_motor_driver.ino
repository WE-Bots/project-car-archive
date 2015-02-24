

//uncomment for communication type
//#define I2C
#define SERIAL

#include <Wire.h>
#include <Servo.h> 
#include <SoftwareSerial.h>
SoftwareSerial mySerial(12, 13); // RX, TX

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

#ifdef SERIAL
  Serial.begin(9600);
#endif

  motor.attach(9);     // range 0-180
  steer.attach(10);    // range 50-120   below 90 turns right
  mySerial.begin(9600);
  while(millis()<2000)
  {
  }
  //mySerial.write("AT+PIN1234");
  timer=millis();
  _ready=true;
} 

void loop() 
{ 
#ifdef SERIAL
  if(Serial.available()>0)
  {
    base_speed=Serial.parseInt();
    //steering_angle=Serial.parseInt();
  }
#endif

  if (mySerial.available()>0)
  {
    emergency_stop=(mySerial.read()-48);
    timer=millis();
  }
  if ((millis()-timer)>=700)
  {
    emergency_stop=true;
  }
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
  delay(15);                           // waits for the servo to get there 
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


