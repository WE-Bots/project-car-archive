#include "hmc5883l.h"

// Get Magnetometer Data
void HMC5883L::getMagnetometerValues(int16_t* array){
  array[0] = x;
  array[1] = y;
  array[2] = z;
}
void HMC5883L::getMagnetometerValues(float* array){
  array[0] = x;
  array[1] = y;
  array[2] = z;
}

// Update Magnetometer Values
//void HMC5883L::updateMagValues(){
//  unsigned char x_l, y_l, z_l, x_h, y_h, z_h; 
//  x_l = readRegister(DATA_XL);
//  x_h = readRegister(DATA_XH);
//  y_l = readRegister(DATA_YL);
//  y_h = readRegister(DATA_YH);
//  z_l = readRegister(DATA_ZL);
//  z_h = readRegister(DATA_ZH);
//  
//  x = x_l | (x_h << 8);
//  y = y_l | (y_h << 8);
//  z = z_l | (z_h << 8);
//}


// Update Magnetometer Values with Repeated Start                                                                
void HMC5883L::updateMagValuesWithRepeatedStart(){
  unsigned char byteArray[6];
  readSequentialRegisters(DATA_XH, byteArray, 6); // 
  
  x = byteArray[1] | (byteArray[0] << 8);
  y = byteArray[5] | (byteArray[4] << 8);
  z = byteArray[3] | (byteArray[2] << 8); 
  
//  //SPARK_FUN
//  Wire.requestFrom(DATA_XH, 6);
//  if(6<=Wire.available()){
//    x =  Wire.receive()<<8; //X msb
//    x |= Wire.receive(); //X lsb
//    
//    z =  Wire.receive()<<8; //Z msb
//    z |= Wire.receive(); //Z lsb
//    
//    y =  Wire.receive()<<8; //Y msb
//    y |= Wire.receive(); //Y lsb
//  } 
  
  //Calibration
  x -= 144;
  y += 325;
  z -= 128;
//  x = 1;
//  y = 2;
//  z = 3;
}


// Read Sequential Registers
void HMC5883L::readSequentialRegisters(unsigned char firstReg, unsigned char* byteArray, int n){
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(firstReg); // read First Register | Auto-Increment
  Wire.endTransmission();

  Wire.requestFrom(DEVICE_ADDRESS, n);
  while(Wire.available() < n){}
  for (int i=0; i<n; i++){
    byteArray[i] = Wire.read();
  }
}


// Write to Register
int HMC5883L::writeRegister(int reg, unsigned char value) {
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission();
}


// Read Register
unsigned char HMC5883L::readRegister(int reg){
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(DEVICE_ADDRESS,1);
  while(Wire.available() < 1){}
  return Wire.read();
}


// Setup Magnetometer
void HMC5883L::setupMagnetometer() {
  //            REG      bit# 76543210
  writeRegister(MODE_REG  , 0b00000000);  // set to default i2c speed and continuous output mode
  writeRegister(CTRL_REG_A, 0b00010100);  // set 1 measurement average, 30Hz, normal measurement mode
  writeRegister(CTRL_REG_B, 0b00100000);  // set as default - 1.3 Ga range
//  char data[3] = {0b00010100,0b00100000,0b00000000};
//  Wire.beginTransmission(DEVICE_ADDRESS);
//  Wire.write(data,3);
//  Wire.endTransmission();
}


// Print CSV
void HMC5883L::print_CSV(){
  Serial.print("MAG:  ");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.println(z);
}
