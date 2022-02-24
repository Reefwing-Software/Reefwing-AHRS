/******************************************************************
  @file       NexgenAHRS.cpp
  @brief      Attitude and Heading Reference System (AHRS) for the Nano 33 BLE.
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     1.0.0
  Date:        22/02/22

  1.0.0 Original Release.           12/02/22

******************************************************************/

#include <Arduino.h>
#include <Wire.h>

#include "NexgenAHRS.h"

/******************************************************************
 * 
 * LSM9DS1 Register Map
 * Reference: https://www.st.com/resource/en/datasheet/lsm9ds1.pdf
 *            Section 6, Page 38, Table 21.
 * 
 * Accelerometer and Gyroscope Registers
 * 
 ******************************************************************/

#define LSM9DS1XG_ACT_THS           0x04
#define LSM9DS1XG_ACT_DUR           0x05
#define LSM9DS1XG_INT_GEN_CFG_XL    0x06
#define LSM9DS1XG_INT_GEN_THS_X_XL  0x07
#define LSM9DS1XG_INT_GEN_THS_Y_XL  0x08
#define LSM9DS1XG_INT_GEN_THS_Z_XL  0x09
#define LSM9DS1XG_INT_GEN_DUR_XL    0x0A
#define LSM9DS1XG_REFERENCE_G       0x0B
#define LSM9DS1XG_INT1_CTRL         0x0C
#define LSM9DS1XG_INT2_CTRL         0x0D
#define LSM9DS1XG_WHO_AM_I          0x0F  // Returns 0x68 ref: 7.11
#define LSM9DS1XG_CTRL_REG1_G       0x10
#define LSM9DS1XG_CTRL_REG2_G       0x11
#define LSM9DS1XG_CTRL_REG3_G       0x12
#define LSM9DS1XG_ORIENT_CFG_G      0x13
#define LSM9DS1XG_INT_GEN_SRC_G     0x14
#define LSM9DS1XG_OUT_TEMP_L        0x15
#define LSM9DS1XG_OUT_TEMP_H        0x16
#define LSM9DS1XG_STATUS_REG        0x17
#define LSM9DS1XG_OUT_X_L_G         0x18
#define LSM9DS1XG_OUT_X_H_G         0x19
#define LSM9DS1XG_OUT_Y_L_G         0x1A
#define LSM9DS1XG_OUT_Y_H_G         0x1B
#define LSM9DS1XG_OUT_Z_L_G         0x1C
#define LSM9DS1XG_OUT_Z_H_G         0x1D
#define LSM9DS1XG_CTRL_REG4         0x1E
#define LSM9DS1XG_CTRL_REG5_XL      0x1F
#define LSM9DS1XG_CTRL_REG6_XL      0x20
#define LSM9DS1XG_CTRL_REG7_XL      0x21
#define LSM9DS1XG_CTRL_REG8         0x22
#define LSM9DS1XG_CTRL_REG9         0x23
#define LSM9DS1XG_CTRL_REG10        0x24
#define LSM9DS1XG_INT_GEN_SRC_XL    0x26
#define LSM9DS1XG_OUT_X_L_XL        0x28
#define LSM9DS1XG_OUT_X_H_XL        0x29
#define LSM9DS1XG_OUT_Y_L_XL        0x2A
#define LSM9DS1XG_OUT_Y_H_XL        0x2B
#define LSM9DS1XG_OUT_Z_L_XL        0x2C
#define LSM9DS1XG_OUT_Z_H_XL        0x2D
#define LSM9DS1XG_FIFO_CTRL         0x2E
#define LSM9DS1XG_FIFO_SRC          0x2F
#define LSM9DS1XG_INT_GEN_CFG_G     0x30
#define LSM9DS1XG_INT_GEN_THS_XH_G  0x31
#define LSM9DS1XG_INT_GEN_THS_XL_G  0x32
#define LSM9DS1XG_INT_GEN_THS_YH_G  0x33
#define LSM9DS1XG_INT_GEN_THS_YL_G  0x34
#define LSM9DS1XG_INT_GEN_THS_ZH_G  0x35
#define LSM9DS1XG_INT_GEN_THS_ZL_G  0x36
#define LSM9DS1XG_INT_GEN_DUR_G     0x37
#define LSM9DS1XG_WHO_AM_I_VALUE    0x68

/******************************************************************
 *
 * Magnetometer Registers 
 * 
 ******************************************************************/

#define LSM9DS1M_OFFSET_X_REG_L_M   0x05
#define LSM9DS1M_OFFSET_X_REG_H_M   0x06
#define LSM9DS1M_OFFSET_Y_REG_L_M   0x07
#define LSM9DS1M_OFFSET_Y_REG_H_M   0x08
#define LSM9DS1M_OFFSET_Z_REG_L_M   0x09
#define LSM9DS1M_OFFSET_Z_REG_H_M   0x0A
#define LSM9DS1M_WHO_AM_I           0x0F  // Returns 0x3D ref: 8.4
#define LSM9DS1M_CTRL_REG1_M        0x20
#define LSM9DS1M_CTRL_REG2_M        0x21
#define LSM9DS1M_CTRL_REG3_M        0x22
#define LSM9DS1M_CTRL_REG4_M        0x23
#define LSM9DS1M_CTRL_REG5_M        0x24
#define LSM9DS1M_STATUS_REG_M       0x27
#define LSM9DS1M_OUT_X_L_M          0x28
#define LSM9DS1M_OUT_X_H_M          0x29
#define LSM9DS1M_OUT_Y_L_M          0x2A
#define LSM9DS1M_OUT_Y_H_M          0x2B
#define LSM9DS1M_OUT_Z_L_M          0x2C
#define LSM9DS1M_OUT_Z_H_M          0x2D
#define LSM9DS1M_INT_CFG_M          0x30
#define LSM9DS1M_INT_SRC_M          0x31
#define LSM9DS1M_INT_THS_L_M        0x32
#define LSM9DS1M_INT_THS_H_M        0x33
#define LSM9DS1M_WHO_AM_I_VALUE     0x3D

/******************************************************************
 *
 * Device Addresses - 
 * ref: https://github.com/arduino-libraries/Arduino_LSM9DS1/blob/master/src/LSM9DS1.cpp
 * 
 ******************************************************************/

#define LSM9DS1XG_ADDRESS 0x6B  //  Address of gyroscope
#define LSM9DS1M_ADDRESS  0x1E  //  Address of magnetometer
#define LPS22HB_ADDRESS   0x5C  //  Address of barometer (on SENSE)  

/******************************************************************
 *
 * Define pressure and temperature conversion rates - 
 * 
 ******************************************************************/

#define ADC_256  0x00 
#define ADC_512  0x02
#define ADC_1024 0x04
#define ADC_2048 0x06
#define ADC_4096 0x08
#define ADC_D1   0x40
#define ADC_D2   0x50

/******************************************************************
 *
 * ENUM Type Definitions - 
 * 
 ******************************************************************/

enum Ascale {  // set of allowable accel full scale settings
  AFS_2G = 0,
  AFS_16G,
  AFS_4G,
  AFS_8G
};

enum Aodr {  // set of allowable gyro sample rates
  AODR_PowerDown = 0,
  AODR_10Hz,
  AODR_50Hz,
  AODR_119Hz,
  AODR_238Hz,
  AODR_476Hz,
  AODR_952Hz
};

enum Abw {  // set of allowable accewl bandwidths
  ABW_408Hz = 0,
  ABW_211Hz,
  ABW_105Hz,
  ABW_50Hz
};

enum Gscale {  // set of allowable gyro full scale settings
  GFS_245DPS = 0,
  GFS_500DPS,
  GFS_NoOp,
  GFS_2000DPS
};

enum Godr {  // set of allowable gyro sample rates
  GODR_PowerDown = 0,
  GODR_14_9Hz,
  GODR_59_5Hz,
  GODR_119Hz,
  GODR_238Hz,
  GODR_476Hz,
  GODR_952Hz
};

enum Gbw {   // set of allowable gyro data bandwidths
  GBW_low = 0,  // 14 Hz at Godr = 238 Hz,  33 Hz at Godr = 952 Hz
  GBW_med,      // 29 Hz at Godr = 238 Hz,  40 Hz at Godr = 952 Hz
  GBW_high,     // 63 Hz at Godr = 238 Hz,  58 Hz at Godr = 952 Hz
  GBW_highest   // 78 Hz at Godr = 238 Hz, 100 Hz at Godr = 952 Hz
};

enum Mscale {  // set of allowable mag full scale settings
  MFS_4G = 0,
  MFS_8G,
  MFS_12G,
  MFS_16G
};

enum Mmode {
  MMode_LowPower = 0, 
  MMode_MedPerformance,
  MMode_HighPerformance,
  MMode_UltraHighPerformance
};

enum Modr {  // set of allowable mag sample rates
  MODR_0_625Hz = 0,
  MODR_1_25Hz,
  MODR_2_5Hz,
  MODR_5Hz,
  MODR_10Hz,
  MODR_20Hz,
  MODR_80Hz
};

/******************************************************************
 *
 * LSM9DS1 Implementation - 
 * 
 ******************************************************************/

LSM9DS1::LSM9DS1() { }

void LSM9DS1::begin() {
    Wire1.begin();
    lps22hb.begin(LPS22HB_ADDRESS);
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x05);
    writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, 0x0c);

    OSR = ADC_4096;      // set pressure amd temperature oversample rate
    Gscale = GFS_245DPS; // gyro full scale
    Godr = GODR_238Hz;   // gyro data sample rate
    Gbw = GBW_med;       // gyro data bandwidth
    Ascale = AFS_2G;     // accel full scale
    Aodr = AODR_238Hz;   // accel data sample rate
    Abw = ABW_50Hz;      // accel data bandwidth
    Mscale = MFS_4G;     // mag full scale
    Modr = MODR_10Hz;    // mag data sample rate
    Mmode = MMode_HighPerformance;  // magnetometer operation mode
    aRes = 0;   // scale resolutions per LSB for the sensors
    gRes = 0;
    mRes = 0;      
    seaLevelPressure = 1018; //average sea level pressure is 1013.25
    Pressure = 0; // pressure in mbars
}

uint8_t LSM9DS1::whoAmIGyro() {
    // Read WHO_AM_I register for LSM9DS1 accel/gyro
    return readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_WHO_AM_I);  
}

uint8_t LSM9DS1::whoAmIMag() {
    // Read WHO_AM_I register for LSM9DS1 magnetometer
    return readByte(LSM9DS1M_ADDRESS, LSM9DS1M_WHO_AM_I);  
}

uint8_t LSM9DS1::whoAmIBaro() {
    return lps22hb.whoAmI();
}

float LSM9DS1::readGyroTemp() {
    // x/y/z gyro register data stored here
    uint8_t rawData[2];  
    // Read the two raw data registers sequentially into data array 
    readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_TEMP_L, 2, &rawData[0]);  
    // Turn the MSB and LSB into a 16-bit signed value
    int16_t rawTemp = (((int16_t)rawData[1] << 8) | rawData[0]);  
    // Gyro chip temperature in degrees Centigrade
    return ((float)rawTemp/256.0 + 25.0); 
}

float LSM9DS1::readBaroTemp() {
    return lps22hb.readTemperature();
}

/******************************************************************
 *
 * I2C Read/Write methods for the LSM9DS1 - 
 * 
 ******************************************************************/

void LSM9DS1::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire1.beginTransmission(address);  // Initialize the Tx buffer
  Wire1.write(subAddress);           // Put slave register address in Tx buffer
  Wire1.write(data);                 // Put data in Tx buffer
  Wire1.endTransmission();           // Send the Tx buffer
}

uint8_t LSM9DS1::readByte(uint8_t address, uint8_t subAddress) {
  uint8_t data; // `data` will store the register data  

  Wire1.beginTransmission(address);         // Initialize the Tx buffer
  Wire1.write(subAddress);                  // Put slave register address in Tx buffer
  Wire1.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire1.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
  data = Wire1.read();                      // Fill Rx buffer with result

  return data;                             // Return data read from slave register
}

void LSM9DS1::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {  
  uint8_t i = 0;

  Wire1.beginTransmission(address);   // Initialize the Tx buffer
  Wire1.write(subAddress);            // Put slave register address in Tx buffer
  Wire1.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  Wire1.requestFrom(address, count);  // Read bytes from slave register address 
  
  while (Wire1.available()) {
    dest[i++] = Wire1.read(); }         // Put read results in the Rx buffer
}