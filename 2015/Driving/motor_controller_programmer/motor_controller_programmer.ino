
#include <Servo.h>

int throttle=90;
Servo motor;


void setup() {
  // put your setup code here, to run once:
motor.attach(9);
pinMode(7,INPUT_PULLUP);
pinMode(6,INPUT_PULLUP);
pinMode(5,INPUT_PULLUP);
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
if(!digitalRead(5))
throttle=0;
if(!digitalRead(6))
throttle=90;
if(!digitalRead(7))
throttle=180;
motor.write(throttle);
Serial.println(throttle);

}
