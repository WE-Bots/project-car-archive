#include "L3G4200D.h"

L3G4200D::L3G4200D(){
  x=0;
  y=0;
  z=0;

}

// Get Gyro Values  
void L3G4200D::getGyroValues(float* array){
  array[0] = x;
  array[1] = y;
  array[2] = z;
}

//// Update Gyro Values
//void L3G4200D::updateGyroValues(){
//  unsigned char x_l = readRegister(OUT_X_L);
//  unsigned char x_h = readRegister(OUT_X_H);
//  unsigned char y_l = readRegister(OUT_Y_L);
//  unsigned char y_h = readRegister(OUT_Y_H);
//  unsigned char z_l = readRegister(OUT_Z_L);
//  unsigned char z_h = readRegister(OUT_Z_H);
//  
//  x = x_l | (x_h << 8); 
//  y = y_l | (y_h << 8);
//  z = z_l | (z_h << 8);
//}


// Update Gyro Values with Repeated Start                                                             
void L3G4200D::updateGyroValuesWithRepeatedStart(){
  unsigned char byteArray[6];
  readSequentialRegisters(OUT_X_L, byteArray, 6); // 
  
  x = (int16_t)(byteArray[0] | (byteArray[1] << 8)); // 
  y = (int16_t)(byteArray[2] | (byteArray[3] << 8));
  z = (int16_t)(byteArray[4] | (byteArray[5] << 8)); 
  
  //Scale to dps
  x *= 0.061037018;
  y *= 0.061037018;  //2000 dps/32767 magical rotation units;
  z *= 0.061037018;
  
  //Remove 0 rotation drift (just noise now)
  x -= X_OFF;
  y -= Y_OFF;
  z -= Z_OFF;
}


// Read Sequential Registers
void L3G4200D::readSequentialRegisters(unsigned char firstReg, unsigned char* byteArray, int n){
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(firstReg | ( 1 << 7 )); // read First Register | Auto-Increment
  Wire.endTransmission();

  Wire.requestFrom(DEVICE_ADDRESS, n);
  while(Wire.available() < n){}
  for (int i=0; i<n; i++){
    byteArray[i] = Wire.read();
  }
}


// Write to Register
int L3G4200D::writeRegister(int reg, unsigned char value) {
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission();
}


// Read Register
unsigned char L3G4200D::readRegister(int reg){
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(DEVICE_ADDRESS,1);
  while(Wire.available() < 1){}
  return Wire.read();
}


// Setup Gyro
void L3G4200D::setupGyro() {
  //            REG     bit# 76543210
  writeRegister(CTRL_REG1, 0b10111111);  // default, but with 200Hz bandwidth and 50Hz cut-off freq. (was 00001111)
  writeRegister(CTRL_REG2, 0b00000000);  // TODO: Configure this HPF, currently default
  writeRegister(CTRL_REG3, 0b00000000);  // default
  writeRegister(CTRL_REG4, 0b00110000);  // default, but with 2000dps Full scale, default was 250dps full scale
  writeRegister(CTRL_REG5, 0b00000000);  // default
}


// Print CSV
void L3G4200D::print_CSV(){
  Serial.print("GYRO:  ");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.println(z);
}
