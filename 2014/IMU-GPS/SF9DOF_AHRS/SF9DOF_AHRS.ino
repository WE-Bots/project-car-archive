
// Sparkfveun 9DOF Razor IMU AHRS
// 9 Degree of Measurement Attitude and Heading Reference System
// Firmware v1.0
//
// Released under Creative Commons License 
// Code by Doug Weibel and Jose Julio
// Based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel

// Axis definition: 
   // X axis pointing forward (to the FTDI connector)
   // Y axis pointing to the right 
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise

/* Hardware version - v13
	
	ATMega328@3.3V w/ external 8MHz resonator
	High Fuse DA
        Low Fuse FF
	
	ADXL345: Accelerometer
	HMC5843: Magnetometer
	LY530:	Yaw Gyro
	LPR530:	Pitch and Roll Gyro

        Programmer : 3.3v FTDI
        Arduino IDE : Select board  "Arduino Duemilanove w/ATmega328"
*/
// This code works also on ATMega168 Hardware

//TODO: Use i2c_t3 instead
#include <i2c_t3.h>
#include <TinyGPS++.h>
//#include <Wire.h>
//#include <SPI.h>
#include "pins_arduino.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "L3G4200D.h"

// ADXL345 Sensitivity(from datasheet) => 4mg/LSB   1G => 1000mg/4mg = 256 steps
// Tested value : 249
#define GRAVITY 249  //this equivalent to 1G in the raw data coming from the accelerometer 
#define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data, 
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

//#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

// I2C command definitions
#define WRITE 0x10
#define READ  0x20
// Slave Memory
#define MEM_LEN 21
uint8_t mem[MEM_LEN];
uint8_t cmd;
size_t  addr;
int     bytes;

#define STATUS_LED 13 

TinyGPSPlus gps;

int SENSOR_SIGN[9] = {-1,1,-1,1,1,1,-1,-1,-1};  //Correct directions x,y,z - gyros, accels, magnetormeter

float G_Dt=0.0035;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible *cough* 285 Hz *cough*

long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values 
int  AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors
int  ACC[3];          //array that store the accelerometers data

float MAG_Heading;

float Accl_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0}; //Store the gyros turn rate in a vector
float Mgnt_Vector[3]= {0,0,0}; //Store the mgnt readings in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {1,0,0},
  {0,1,0},
  {0,0,1}
}; 
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {0,0,0},
  {0,0,0},
  {0,0,0}
};

//Sensor objects
L3G4200D gyro;
HMC5883L magnetometer;
ADXL345  accelerometer;

void setup()
{
  Serial.begin(115200);  // Used to communicate with master computer
  Serial2.begin(4800);   // Connection to GPS 

  pinMode (STATUS_LED,OUTPUT);  // Status LED
 
  // Master on the first I2C bus
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
  // Slave on second bus
  Wire1.begin(I2C_SLAVE, 0x01, I2C_PINS_29_30, I2C_PULLUP_EXT, I2C_RATE_400);
  
  // Register Slave events
  Wire1.onReceive(receiveEvent);
  Wire1.onRequest(requestEvent);
  
  Serial.println("Alive!");
    
  // Wait 50 ms for devices to turn on
  delay(50);
  
  // Initialize sensors
  gyro.setupGyro();
  magnetometer.setupMagnetometer();
  accelerometer.setupAccel();

  digitalWrite(STATUS_LED,LOW);
  
  // Removed calibration code... Hard coded calibration so that the car doesn't have to be level on startup.
  
  digitalWrite(STATUS_LED,HIGH);
    
  timer=micros();
  counter=0;
  
  mem[0] = 0x01;
}

void loop() // Main Loop
{  
  // Update GPS
  while(Serial2.available()>0){
    int c = Serial2.read();
    if (gps.encode(c)){
//      Serial.print("lat: ");
//      Serial.print(gps.location.lat());
//      Serial.print("   lon: ");
//      Serial.print(gps.location.lng());
//      Serial.print("   date: ");
//      Serial.print(gps.date.year());
//      Serial.print("-");
//      Serial.print(gps.date.month());
//      Serial.print("-");
//      Serial.print(gps.date.day());
//      Serial.print("   time: ");
//      Serial.print(gps.time.hour());
//      Serial.print(":");
//      Serial.print(gps.time.minute());
//      Serial.print(":");
//      Serial.println(gps.time.second());
      //Copy updated values into the data array
      float latlng[2];
      latlng[0] = gps.location.lat();
      latlng[1] = gps.location.lng();
      uint8_t* bytePtr = (uint8_t*)latlng;
      for (int b=0x0D; b<0x15; b++){
        mem[b] = *bytePtr;
        bytePtr++;
      }
    }
  }
  
  // Update IMU
  if((micros()-timer)>=3333)  // Main loop runs at 300 Hz
  {
    counter++;
    timer_old = timer;
    timer=micros();
    
    if (timer>timer_old)
      G_Dt = (timer-timer_old)/1000000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;
    
    //Read sensors
    gyro.updateGyroValuesWithRepeatedStart();
    accelerometer.updateAccelValuesWithRepeatedStart();
    gyro.getGyroValues(Gyro_Vector);
    accelerometer.getAccelerometerValues(Accl_Vector);
    
    for (int i = 0; i < 3; i++){
      Gyro_Vector[i] = ToRad(Gyro_Vector[i]);
    }
  
    if (counter > 10)  // Read compass data at 30Hz... (5 loop runs)
    {
      counter=0;
      magnetometer.updateMagValuesWithRepeatedStart();
      magnetometer.getMagnetometerValues(Mgnt_Vector);
    }
    
    // Calculations...
    Matrix_update(); 
    Normalize();
    Compass_Heading();
    Drift_correction();
    Euler_angles();
    // ***
    //printdata();
    
    // Turn off the LED when you saturate any of the gyros.
    if((abs(Gyro_Vector[0])>=ToRad(1000))||(abs(Gyro_Vector[1])>=ToRad(1000))||(abs(Gyro_Vector[2])>=ToRad(1000)))
    {
      if (gyro_sat<50)
        gyro_sat+=10;
    }
    else
    {
      if ((gyro_sat>0)&&(gyro_sat>-126))  // Prevent wrapping!
        gyro_sat--;
    }
  
    if (gyro_sat>0)
      digitalWrite(STATUS_LED,LOW);
    else
      digitalWrite(STATUS_LED,HIGH);
    
    //Copy updated values into the data array
    float DAISY[3];  //roll pitch yaw
    DAISY[0] = ToDeg(roll);
    DAISY[1] = ToDeg(pitch);
    DAISY[2] = ToDeg(yaw);
    uint8_t* bytePtr = (uint8_t*)DAISY;
    //for (int b=0x01; b<0x0C; b++){
    for (int b=0x01; b<0x0D; b++){
      mem[b] = *bytePtr;
      bytePtr++;
    }
  }
  // TODO: Check if GPS is good to go
}

//
// handle Slave Rx Event (incoming I2C request/data)
//
void receiveEvent(size_t len)
{
  if(Wire1.available())
  {
    // grab command
    cmd = Wire1.readByte();
    //Don't actually do anything for now.
    //Could be used to select the status byte.
  }
}

//
// handle Slave Tx Event (outgoing I2C data)
//
void requestEvent(void)
{
  //Need a way to get length, but for now 'cmd' is where to start reading from...
  //Serial.println("Request made");
  //for (int i=1; i<5; i++)
//  float flMem[5];
//  uint8_t* bytePtr = (uint8_t*)flMem;
//  Serial.println("Stuff:");
//  for (int i=1; i<21; i++){
//    *bytePtr=mem[i];
//    bytePtr++;
//  }
//  Serial.println("meh");
//  for (int i=0; i<5; i++)
//    Serial.println(flMem[i]);
  for(int i=1; i<21; i++){
    Wire1.write(mem[i]);
  }
}
