#include <Servo.h>

unsigned int ir1Pin = 14;
unsigned int ir2Pin = 15;
unsigned int ir1ServoPin = 11;
unsigned int ir2ServoPin = 12;
Servo ir1Servo;
Servo ir2Servo;
unsigned int servo1Pos = 0;
unsigned int servo2Pos = 0;
unsigned int servo1Dir = 0; 
unsigned int servo2Dir = 0; 
unsigned int servo1Stop = 0;
unsigned int servo2Stop = 0;
unsigned long servoPrevTime = 0;
unsigned int servoDelay = 100;
unsigned int servoSweepAngle = 10;

unsigned int usEchoPin = 7;
unsigned int usTrigPin = 8;
unsigned long usTime = 0; 
unsigned long usPrevTime = 0;
unsigned int usErrorThreshold = 800;
unsigned int usReadDelay = 250;
unsigned long usReadPrevTime = 0; 
unsigned int usThreshold = 25; 

unsigned int led1GPin = 3;
unsigned int led1RPin = 4; 
unsigned int led1BPin = 5; 
unsigned int led2GPin = 22;
unsigned int led2RPin = 20; 
unsigned int led2BPin = 21; 

unsigned int led1GValue;
unsigned int led1RValue;
unsigned int led1BValue;
unsigned int led2GValue;
unsigned int led2RValue;
unsigned int led2BValue;

unsigned int alert1Pin = 16;
unsigned int alert2Pin = 17; 
unsigned int alert3Pin = 18; 
unsigned int alert1 = 0;
unsigned int alert2 = 0; 
unsigned int alert3 = 0;

void setup()
{
  ir1Servo.attach(ir1ServoPin);
  ir2Servo.attach(ir2ServoPin);
  pinMode (ir1Pin, INPUT);
  pinMode (ir2Pin, INPUT);
  
  pinMode (led1GPin, OUTPUT);
  pinMode (led1RPin, OUTPUT);
  pinMode (led1BPin, OUTPUT);
  pinMode (led2GPin, OUTPUT);
  pinMode (led2RPin, OUTPUT);
  pinMode (led2BPin, OUTPUT);
  
  pinMode (usTrigPin, OUTPUT);
  digitalWrite(usTrigPin, HIGH);
  pinMode(usEchoPin, INPUT);
  
  pinMode (alert1Pin, OUTPUT);
  pinMode (alert2Pin, OUTPUT);
  pinMode (alert3Pin, OUTPUT);
  pinMode (13, OUTPUT);
  digitalWrite (13, HIGH);
  
  Serial.begin(9600);
}

void loop()
{
  if ( (millis() - usReadPrevTime) > usReadDelay)
  {
    digitalWrite(usEchoPin, LOW);
    digitalWrite(usTrigPin, LOW);
    delayMicroseconds(20);
    digitalWrite(usTrigPin, HIGH);
    usTime = pulseIn(usEchoPin, HIGH) /58;
    if (usTime > usPrevTime)
    {
      if (usTime - usPrevTime > usErrorThreshold)
          usTime = usPrevTime;
    }
    if (usTime < usPrevTime)
    {
      if (usPrevTime - usTime > usErrorThreshold)
          usTime = usPrevTime;
    }
    usReadPrevTime = millis();
  }
  
  Serial.println(usTime);
  
  if (usTime < usThreshold)
  {
    digitalWrite(alert1Pin, HIGH);
    alert1 = 1;
    /*
    analogWrite(led1GPin, 0); 
    analogWrite(led1BPin, 0); 
    analogWrite(led1RPin, 255);
  
    analogWrite(led2GPin, 0); 
    analogWrite(led2BPin, 0); 
    analogWrite(led2RPin, 255);
    */
  }
  
  else
  { 
    digitalWrite(alert1Pin, LOW);
    alert1 = 0;
  }
  
  /*if (digitalRead(ir1Pin))
  {
    digitalWrite(alert2Pin, HIGH); 
    alert2 = 1;
    servo1Stop = 1;  
    /*
    analogWrite(led1GPin, 0); 
    analogWrite(led1BPin, 0); 
    analogWrite(led1RPin, 255);
  
    analogWrite(led2GPin, 0); 
    analogWrite(led2BPin, 0); 
    analogWrite(led2RPin, 255);
    
    
  }
  
  else 
  {
    digitalWrite(alert2Pin, LOW);
    alert2 = 0;
    servo1Stop = 0;
  }
    
   if (digitalRead(ir2Pin))
   {
     digitalWrite(alert3Pin, HIGH);
     servo2Stop = 1;
     alert3 = 1;
     /*
     analogWrite(led1GPin, 0); 
     analogWrite(led1BPin, 0); 
     analogWrite(led1RPin, 255);
  
    analogWrite(led2GPin, 0); 
    analogWrite(led2BPin, 0); 
    analogWrite(led2RPin, 255); 
    
   }
  
  else 
  {
    digitalWrite(alert3Pin, LOW);
    servo2Stop = 0; 
    alert3 = 0;
  }
    
    if ((millis() - servoPrevTime) > servoDelay) 
    {
      if (!servo1Stop)
      {
        if (servo1Pos >= 180)
          servo1Dir = 1;
      
        if (servo1Pos == 0)
          servo1Dir = 0;
      
        if (servo1Dir)
          servo1Pos = servo1Pos - servoSweepAngle;
        else 
          servo1Pos = servo1Pos + servoSweepAngle;
      }
      
      if (!servo2Stop)
      {
        if (servo2Pos >= 180)
        servo2Dir = 1;
      
        if (servo2Pos == 0)
          servo2Dir = 0;
      
        if (servo2Dir)
          servo2Pos = servo2Pos - servoSweepAngle;
        else 
          servo2Pos = servo2Pos + servoSweepAngle;
      }
     
      servoPrevTime = millis();
    } 
    
    ir1Servo.write(servo1Pos);
    ir2Servo.write(servo2Pos);  
    */
    //if (alert1 || alert2 || alert3)
    if (alert1)
    {
    analogWrite(led1GPin, 0); 
    analogWrite(led1BPin, 0); 
    analogWrite(led1RPin, 255);
  
    analogWrite(led2GPin, 0); 
    analogWrite(led2BPin, 0); 
    analogWrite(led2RPin, 255);
    }
    
    else 
    {
    analogWrite(led1GPin, 0); 
    analogWrite(led1BPin, 250); 
    analogWrite(led1RPin, 200);
  
    analogWrite(led2GPin, 0); 
    analogWrite(led2BPin, 250); 
    analogWrite(led2RPin, 200);
    }
}
   
