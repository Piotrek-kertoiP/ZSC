#include "Arduino.h"
#include <Wire.h>

#define LIS3DH_DEFAULT_ADDRESS   0x18		//dodane, Piotrek
#define LIS3DH_REG_WHOAMI        0x0F 
#define LIS3DH_REG_CTRL1         0x20  
#define LIS3DH_REG_CTRL4         0x23 
#define LIS3DH_REG_OUT_X_L       0x28 
#define LIS3DH_RANGE_4_G         0x01
#define dataRate  				 0x07

class Accelerometer{
 public:
  
  Accelerometer(void);
  bool begin();
  void read();
  void setRange(uint8_t range);
  uint8_t readRegister8(uint8_t reg);
  void    writeRegister8(uint8_t reg, uint8_t value);
    
  int16_t x, y, z;
  float x_g, y_g, z_g;
  
  TwoWire *I2Cinterface;  
};

