

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
int base_speed=10;
int speed_error=0;
int real_speed=5;
int accumulated_speed_error=0;
boolean _ready=false;
boolean emergency_stop=false;
unsigned long timer=0;
char* _pointer;
int pointer_counter=0;
int left_speed;
long left_dist;
int right_speed;
long right_dist;
long dist;
int echo_time[4];
int echo_time_counter=0;
int echo_time_avg=0;


uint8_t receive_buffer[9];
uint8_t inByte;
uint8_t inByte_old;
boolean error_state;

uint8_t lightState;
float   dst=0;
float   ang;

int stop_couter=0;
long stop_timer=0;

void setup() 
{
#ifdef I2C
  Wire.begin();                 //join as device #4
#endif

#ifdef SERIAL
  Serial.begin(9600);
#endif

  error_state = 0;
  inByte = 0;
  inByte_old = 8;
  pinMode(3,OUTPUT);



  pinMode(8,OUTPUT);
  pinMode(7,INPUT);

  motor.attach(9);     // range 0-180
  steer.attach(10);    // range 50-120   below 90 turns right
  mySerial.begin(9600);
  //mySerial.write("AT+PIN1234");
  timer=millis();
  _ready=false;

} 

void loop() 
{ 
  if (millis()>5000)
  {
    if (Serial.available()>1){
      //Data arrives in pairs
      inByte = Serial.read();
      if (inByte!=(inByte_old+1)){
        if ((inByte_old==8)&&(inByte==0)){
          //Actually OK
        }
        else{
          error_state=1;
          inByte_old = 8;
        }
      }
      if (error_state){
        //wait for the state to become valid again
        //inByte = Serial.read();
        if (inByte == 8){
          error_state = 0;
          Serial.read(); // Waste a byte
        }
      }
      else{
        if ((0<=inByte)&&(8>=inByte))
          receive_buffer[inByte] = Serial.read();
        else{
          Serial.read();
          error_state=1;
        }
        if (inByte == 8){
          //Scan in new information
          lightState = receive_buffer[0];
          if (lightState=2)
          {
            emergency_stop=false;
            Serial.write(0);
          }
          uint8_t* bytePtr = (uint8_t*)&dst;
          for (int i = 1; i<5; i++){
            *bytePtr = receive_buffer[i];
            bytePtr++;
          }
          bytePtr = (uint8_t*)&ang;
          for (int i = 5; i<9; i++){
            *bytePtr = receive_buffer[i];
            bytePtr++;
          }
        }
        inByte_old = inByte;
      }
    }



    if (dst>0)
    {
      steering_angle++;    //turn left
    }
    else if (dst<0)
    {
      steering_angle--;    //turn right
    }

    motor.write(90-constrain(base_speed+speed_error,-90,90));
    steer.write(90-constrain(steering_angle,-30,40));

    delay(15);                           // waits for the servo to get there 
  } 
  else
  {
    motor.write(90);
    steer.write(90);
  }
}
