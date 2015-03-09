/*============================================================================*/
// l3g4200d.h - Lawrence Cushman and Andrew Simpson, 2013.
//              - github: lawrencecushman
//  
//  This arduino sketch reads x, y, z-axis output data from the l3g4200d
//  gyroscope. There are two functions defined that provide updates to this
//  data: 
//    - updateGyroValues(), which iteratively reads the 6 output registers
//    AND
//    - updateGyroValuesWithRepeatedStart(), takes advantage of the repeated
//      start feature of I2C and allows for 3x faster read times, on average.
//  
//  Thus, updateGyroValuesWithRepeatedStart() should be used, if possible.
//  
/*============================================================================*/

#ifndef L3G4200D_h
#define L3G4200D_h

#include <Arduino.h>
#include <i2c_t3.h>

// Slave Address. In the documentation, this address is called SAD
#define DEVICE_ADDRESS 0x69

/*----------------------------------------------------------------------------*/
//  REGISTER MAPPING
//   The registers are further defined in the datasheet. The names are exactly 
//   the same. Addresses with two stars (**) in the comment are used in this
//   code. In the case of control registers, ** means they are modified from 
//   their default values.
/*----------------------------------------------------------------------------*/
#define WHO_AM_I      0x0F // Holds the address DEVICE_ADDRESS (default 0x69)
#define CTRL_REG1     0x20 // ** Control Register 1
#define CTRL_REG2     0x21 // Control Register 2
#define CTRL_REG3     0x22 // Control Register 3
#define CTRL_REG4     0x23 // ** Control Register 4
#define CTRL_REG5     0x24 // Control Register 5
#define REFERENCE_REG 0x25 // Reference
#define OUT_TEMP      0x26 // Temperature Data
#define STATUS_REG    0x27 // Contains 8 various status bits
#define OUT_X_L       0x28 // ** X-Axis LSB
#define OUT_X_H       0x29 // ** X-Axis MSB
#define OUT_Y_L       0x2A // ** Y-Axis LSB
#define OUT_Y_H       0x2B // ** Y-Axis MSB 
#define OUT_Z_L       0x2C // ** Z-Axis LSB
#define OUT_Z_H       0x2D // ** Z-Axis MSB
#define FIFO_CTRL_REG 0x2E // FIFO Control Register
#define FIFO_SRC_REG  0x2F // Contains various FIFO Status bits
#define INT1_CFG      0x30 // Interrupt control register
#define INT1_SRC      0x31 // Contains various interrupt status bits
#define INT1_TSH_XH   0x32 // X-Axis Interrupt threshold MSB
#define INT1_TSH_XL   0x33 // X-Axis Interrupt threshold LSB
#define INT1_TSH_YH   0x34 // Y-Axis Interrupt threshold MSB
#define INT1_TSH_YL   0x35 // Y-Axis Interrupt threshold LSB
#define INT1_TSH_ZH   0x36 // Z-Axis Interrupt threshold MSB
#define INT1_TSH_ZL   0x37 // Z-Axis Interrupt threshold LSB
#define INT1_DURATION 0x38 // Interrupt duration configuration
#define X_OFF         0.422 // 0 value for x axis
#define Y_OFF         0.116 // 0 value for y axis
#define Z_OFF         0.943 // 0 value for z axis

class L3G4200D {
private:
  float x, y, z;


public:
  L3G4200D();
  
  // Get Gyro Data
  // Stores x, y and z into array
  void getGyroValues(float* array);

  // Setup Gyro
  //  Sets up the gyros control registers.
  //  To take advantage of this function, toggle the bits based on your needs. The 
  //  default values for each control register are 0b00000000. 
  //  For more information, see pg.29 of the documentation.
  void setupGyro();

  // Update Gyro Values
  //  Reads the 6 gyro output registers using six separate calls to readRegister().
  //  Readings are stored in the x, y, and z integer variables. The performance of
  //  this function is crippled compared to the Repeated Start implementation.
  //void updateGyroValues();

  // Update Gyro Values with Repeated Start
  //  Reads the 6 gyro output registers with a single call to 
  //  readSequentialRegisters(). Readings are stored in the x, y, and z integer 
  //  variables.                                                                
  void updateGyroValuesWithRepeatedStart();

  // Read Sequential Registers
  //  Reads multiple registers with adjacent addresses. This function takes 
  //  advantage of I2C's repeated start mechanism, which avoids unnecessary start
  //  conditions and acknowklegements. The L3G4200D needs to be explicitly told to
  //  increment to the next register after each read. If the MSb of the address is
  //  1, the register address is automatically incremented.
  //  For more information, see pg. 22 of the L3G4200D documentation.
  // Arguments:
  //  firstReg  - the address of the first register to be read.
  //  byteArray - a pointer to the array the read values will be stored to
  //  n         - the size of byteArray
  void readSequentialRegisters(unsigned char firstReg, unsigned char* byteArray, int n);

  // Write to Register
  //  Writes a byte to a single register.
  // Arguments:
  //  reg   - the register's 7 bit address
  //  value - the value to be stored in the register
  // Returns:
  //  indicates the status of the transmission via endTransmission():
  //    0:success
  //    1:data too long to fit in transmit buffer
  //    2:received NACK on transmit of address
  //    3:received NACK on transmit of data
  //    4:other error
  int writeRegister(int reg, unsigned char value);

  // Read Register
  //  Reads data from a single register
  // Arguments:
  //  reg - the register to read from
  // Returns:
  //  The data stored in reg
  unsigned char readRegister(int reg);

  // Print CSV
  //  Prints x, y and z in CSV (Comma-Separated Values) format.
  void print_CSV();
};

#endif
