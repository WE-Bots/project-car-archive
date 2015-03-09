/*============================================================================*/
//  hmc5883l.h - Lawrence Cushman and Andrew Simpson, 2013.
//              - github: lawrencecushman
//  
//  This arduino sketch reads x, y, z-axis output data from the hmc5883l
//  magnetometer. There are two functions defined that provide updates to this
//  data: 
//    - updateMagValues(), which iteratively reads the 6 output registers
//    AND
//    - updateMagValuesWithRepeatedStart(), takes advantage of the repeated
//      start feature of I2C and allows for 3x faster read times, on average.
//  
//  Thus, updateMagValuesWithRepeatedStart() should be used, if possible.
//  
/*============================================================================*/

#ifndef HMC5883L_H
#define HMC5883L_H
#include <Arduino.h>
#include <i2c_t3.h>

// Slave Address. In the documentation, this address is called SAD
#define DEVICE_ADDRESS 0x1E

/*----------------------------------------------------------------------------*/
//  REGISTER MAPPING
//   The registers are further defined in the datasheet. Addresses with two 
//   stars (**) in the comment are used in this code. In the case of control
//   registers, ** means they are modified from their default values.
/*----------------------------------------------------------------------------*/
#define CTRL_REG_A 0x00 // ** Configuration Register A
#define CTRL_REG_B 0x01 // Configuration Register B
#define MODE_REG   0x02 // ** Control Register for the data mode
#define DATA_XH    0x03 // ** X-Axis MSB
#define DATA_XL    0x04 // ** X-Axis LSB
#define DATA_YH    0x05 // ** Y-Axis MSB
#define DATA_YL    0x06 // ** Y-Axis LSB
#define DATA_ZH    0x07 // ** Z-Axis MSB
#define DATA_ZL    0x08 // ** Z-Axis LSB
#define STATUS_REG 0x09 // Contains status bits LSb:Ready, 2LBSb:Locked
#define ID_REG_A   0x0A // Identification Register A
#define ID_REG_B   0x0B // Identification Register B 
#define ID_REG_C   0x0C // Identification Register C

class HMC5883L {
  private:
  int16_t x,y,z;                        // 16-bit magnetometer output values

  public:
  // Get Magnetometer Data
  // Stores x, y and z into array
  void getMagnetometerValues(int16_t* array);
  void getMagnetometerValues(float*   array);
  
  // Update Magnetometer Values
  //  Reads the 6 magnetometer output registers using six separate calls to 
  //  readRegister().  Readings are stored in the x, y, and z integer variables. 
  //  The performance of this function is crippled compared to the Repeated Start
  //  implementation.
  //void updateMagValues();


  // Update Magnetometer Values with Repeated Start
  //  Reads the 6 magnetometer output registers with a single call to 
  //  readSequentialRegisters(). Readings are stored in the x, y, and z integer 
  //  variables.                                                                
  void updateMagValuesWithRepeatedStart();


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

  // Setup Magnetometer
  //  Sets up the magnetometer's control registers.
  //  To take advantage of this function, toggle the bits based on your needs. The 
  //  default values for each control register are 0b00000000. 
  //  For more information, see pg.29 of the documentation.
  void setupMagnetometer();

  // Print CSV
  //  Prints x, y and z in CSV (Comma-Separated Values) format.
  void print_CSV();
};
#endif
