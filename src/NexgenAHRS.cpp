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
 * Barometer Registers 
 * 
 ******************************************************************/

#define LPS22HB_WHO_AM_I        0X0F //Who am I
#define LPS22HB_RES_CONF        0X1A //Normal (0) or Low current mode (1)
#define LPS22HB_CTRL_REG1       0X10 //output rate and filter settings
#define LPS22HB_CTRL_REG2       0X11 //BOOT FIFO_EN STOP_ON_FTH IF_ADD_INC I2C_DIS SWRESET One_Shot
#define LPS22HB_STATUS_REG      0X27 //Temp or Press data available bits
#define LPS22HB_PRES_OUT_XL     0X28 //XLSB
#define LPS22HB_PRES_OUT_L      0X29 //LSB
#define LPS22HB_PRES_OUT_H      0X2A //MSB
#define LPS22HB_TEMP_OUT_L      0X2B //LSB
#define LPS22HB_TEMP_OUT_H      0X2C //MSB
#define LPS22HB_WHO_AM_I_VALUE  0xB1 // Expected return value of WHO_AM_I register

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
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x05);
    writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, 0x0c);

    OSR = ADC_4096;      // set pressure amd temperature oversample rate
    Godr = GODR_238Hz;   // gyro data sample rate
    Gbw = GBW_med;       // gyro data bandwidth
    Aodr = AODR_238Hz;   // accel data sample rate
    Abw = ABW_50Hz;      // accel data bandwidth
    Modr = MODR_10Hz;    // mag data sample rate
    Mmode = MMode_HighPerformance;  // magnetometer operation mode

    //  Scale resolutions per LSB for each sensor
    //  sets aRes, gRes, mRes and aScale, gScale, mScale
    setAccResolution(Ascale::AFS_2G);
    setGyroResolution(Gscale::GFS_245DPS);
    setMagResolution(Mscale::MFS_4G);
}

uint8_t LSM9DS1::whoAmIGyro() {
    // Read WHO_AM_I register for LSM9DS1 accel/gyro
    return readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_WHO_AM_I);  
}

uint8_t LSM9DS1::whoAmIMag() {
    // Read WHO_AM_I register for LSM9DS1 magnetometer
    return readByte(LSM9DS1M_ADDRESS, LSM9DS1M_WHO_AM_I);  
}

bool LSM9DS1::connected() {
  return (whoAmIGyro() == LSM9DS1XG_WHO_AM_I_VALUE &&
          whoAmIMag() == LSM9DS1M_WHO_AM_I_VALUE);
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

void LSM9DS1::setAccResolution(Ascale ascale) {
  aScale = static_cast<uint8_t>(ascale);
  switch (ascale) {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 16 Gs (01), 4 Gs (10), and 8 Gs  (11). 
    case Ascale::AFS_2G:
      aRes = 2.0/32768.0;
      break;
    case Ascale::AFS_16G:
      aRes = 16.0/32768.0;
      break;
    case Ascale::AFS_4G:
      aRes = 4.0/32768.0;
      break;
    case Ascale::AFS_8G:
      aRes = 8.0/32768.0;
      break;
  }
}

void LSM9DS1::setGyroResolution(Gscale gscale) {
  gScale = static_cast<uint8_t>(gscale);
  switch (gscale) {
    // Possible gyro scales (and their register bit settings) are:
    // 245 DPS (00), 500 DPS (01), and 2000 DPS  (11). 
    case Gscale::GFS_245DPS:
      gRes = 245.0/32768.0;
      break;
    case Gscale::GFS_500DPS:
      gRes = 500.0/32768.0;
      break;
    case Gscale::GFS_2000DPS:
      gRes = 2000.0/32768.0;
      break;
  }
}

void LSM9DS1::setMagResolution(Mscale mscale) {
  mScale = static_cast<uint8_t>(mscale);
  switch (mscale) {
    // Possible magnetometer scales (and their register bit settings) are:
    // 4 Gauss (00), 8 Gauss (01), 12 Gauss (10) and 16 Gauss (11)
    case Mscale::MFS_4G:
      mRes = 4.0/32768.0;
      break;
    case Mscale::MFS_8G:
      mRes = 8.0/32768.0;
      break;
    case Mscale::MFS_12G:
      mRes = 12.0/32768.0;
      break;
    case Mscale::MFS_16G:
      mRes = 16.0/32768.0;
      break;
  }
}

float LSM9DS1::getAccResolution() {
  return (aRes * 1000.0);
}

float LSM9DS1::getGyroResolution() {
  return (gRes * 1000.0);
}

float LSM9DS1::getMagResolution() {
  return (mRes * 1000.0);
}

SelfTestResults LSM9DS1::selfTest() {
  SelfTestResults results;
  float accel_noST[3] = {0.0, 0.0, 0.0}, accel_ST[3] = {0.0, 0.0, 0.0};
  float gyro_noST[3] = {0.0, 0.0, 0.0}, gyro_ST[3] = {0.0, 0.0, 0.0};

  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG10,   0x00); // disable self test
  setBiasOffsets(gyro_noST, accel_noST);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG10,   0x05); // enable gyro/accel self test
  setBiasOffsets(gyro_ST, accel_ST);

  results.gyrodx = (gyro_ST[0] - gyro_noST[0]);
  results.gyrody = (gyro_ST[1] - gyro_noST[1]);
  results.gyrodz = (gyro_ST[2] - gyro_noST[2]);

  results.accdx = 1000.0 * (accel_ST[0] - accel_noST[0]);
  results.accdy = 1000.0 * (accel_ST[1] - accel_noST[1]);
  results.accdz = 1000.0 * (accel_ST[2] - accel_noST[2]);

  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG10,   0x00); // disable self test
  delay(200);

  return results;
}

void LSM9DS1::calibrateAccGyro() {
  setBiasOffsets(gyroBias, accelBias);
}

void LSM9DS1::calibrateMag() {
  setMagneticBias(magBias);
}

BiasOffsets LSM9DS1::getBiasOffsets() {
  BiasOffsets biasOffsets;

  biasOffsets.accelBias = accelBias;
  biasOffsets.gyroBias = gyroBias;
  biasOffsets.magBias = magBias;

  return biasOffsets;
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void LSM9DS1::setBiasOffsets(float* dest1, float* dest2) {  
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  uint16_t samples, index;

  // enable the 3-axes of the gyroscope
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG4, 0x38);
  // configure the gyroscope
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG1_G, Godr << 5 | gScale << 3 | Gbw);
  delay(200);
  // enable the three axes of the accelerometer 
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG5_XL, 0x38);
  // configure the accelerometer-specify bandwidth selection with Abw
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG6_XL, Aodr << 5 | aScale << 3 | 0x04 |Abw);
  delay(200);
  // enable block data update, allow auto-increment during multiple byte read
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x44);

  // First get gyro bias
  byte c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c | 0x02);     // Enable gyro FIFO  
  delay(50);                                                       // Wait for change to take effect
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
  delay(1000);  // delay 1000 milliseconds to collect FIFO samples

  samples = (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_SRC) & 0x2F); // Read number of stored samples

  for (index = 0; index < samples ; index++) {            // Read the gyro data stored in the FIFO
    int16_t gyro_temp[3] = {0, 0, 0};

    readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_G, 6, &data[0]);
    gyro_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
    gyro_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
    gyro_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

    gyro_bias[0] += (int32_t) gyro_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    gyro_bias[1] += (int32_t) gyro_temp[1]; 
    gyro_bias[2] += (int32_t) gyro_temp[2]; 
  }  

  gyro_bias[0] /= samples; // average the data
  gyro_bias[1] /= samples; 
  gyro_bias[2] /= samples; 

  dest1[0] = (float)gyro_bias[0] * gRes;  // Properly scale the data to get deg/s
  dest1[1] = (float)gyro_bias[1] * gRes;
  dest1[2] = (float)gyro_bias[2] * gRes;

  c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c & ~0x02);   //Disable gyro FIFO  
  delay(50);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x00);  // Enable gyro bypass mode

  // now get the accelerometer bias
  c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c | 0x02);     // Enable accel FIFO  
  delay(50);                                                       // Wait for change to take effect
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x20 | 0x1F);  // Enable accel FIFO stream mode and set watermark at 32 samples
  delay(1000);  // delay 1000 milliseconds to collect FIFO samples

  samples = (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_SRC) & 0x2F); // Read number of stored samples

  for (index = 0; index < samples ; index++) {            // Read the accel data stored in the FIFO
    int16_t accel_temp[3] = {0, 0, 0};

    readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_XL, 6, &data[0]);
    accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
    accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1]; 
    accel_bias[2] += (int32_t) accel_temp[2]; 
  }  

  accel_bias[0] /= samples; // average the data
  accel_bias[1] /= samples; 
  accel_bias[2] /= samples; 

  if (accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) (1.0/aRes);}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) (1.0/aRes);}

  dest2[0] = (float)accel_bias[0] * aRes;  // Properly scale the data to get g
  dest2[1] = (float)accel_bias[1] * aRes;
  dest2[2] = (float)accel_bias[2] * aRes;

  c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c & ~0x02);   //Disable accel FIFO  
  delay(50);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x00);  // Enable accel bypass mode
}

void setMagneticBias(float *dest1) {
  uint8_t data[6]; // data array to hold mag x, y, z, data
  uint16_t index = 0, sample_count = 128;
  int32_t mag_bias[3] = {0, 0, 0};
  int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};

  // configure the magnetometer-enable temperature compensation of mag data
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M, 0x80 | Mmode << 5 | Modr << 2); // select x,y-axis mode
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, mScale << 5 ); // select mag full scale
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG3_M, 0x00 ); // continuous conversion mode
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG4_M, Mmode << 2 ); // select z-axis mode
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG5_M, 0x40 ); // select block update mode

  delay(4000);

  for (index = 0; index < sample_count; index++) {
    int16_t mag_temp[3] = {0, 0, 0};
    
    readBytes(LSM9DS1M_ADDRESS, LSM9DS1M_OUT_X_L_M, 6, &data[0]);  // Read the six raw data registers into data array
    mag_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;   // Form signed 16-bit integer for each sample in FIFO
    mag_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
    mag_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(105);  // at 10 Hz ODR, new mag data is available every 100 ms
  }

  mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

  dest1[0] = (float) mag_bias[0] * mRes;  // save mag biases in G for main program
  dest1[1] = (float) mag_bias[1] * mRes;   
  dest1[2] = (float) mag_bias[2] * mRes;          

  //write biases to accelerometermagnetometer offset registers as counts);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_X_REG_L_M, (int16_t) mag_bias[0]  & 0xFF);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_X_REG_H_M, ((int16_t)mag_bias[0] >> 8) & 0xFF);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Y_REG_L_M, (int16_t) mag_bias[1] & 0xFF);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Y_REG_H_M, ((int16_t)mag_bias[1] >> 8) & 0xFF);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Z_REG_L_M, (int16_t) mag_bias[2] & 0xFF);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Z_REG_H_M, ((int16_t)mag_bias[2] >> 8) & 0xFF);
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

/******************************************************************
 *
 * LPS22HB Implementation - 
 * 
 ******************************************************************/

LPS22HB::LPS22HB() { }

void LPS22HB::begin() {
  _address = LPS22HB_ADDRESS;
  Wire1.begin();
  write(LPS22HB_RES_CONF, 0x0); // resolution: temp=32, pressure=128
  write(LPS22HB_CTRL_REG1, 0x00); // one-shot mode
}

byte LPS22HB::whoAmI() {
  Wire1.beginTransmission(_address);
  Wire1.write(LPS22HB_WHO_AM_I);
  Wire1.endTransmission();
  Wire1.requestFrom(_address, 1);
  return Wire1.read();
}

bool LPS22HB::connected() {
  return (whoAmI() == LPS22HB_WHO_AM_I_VALUE);
}

float LPS22HB::readPressure() {
  write(LPS22HB_CTRL_REG2, 0x1);

  if (status(0x1) < 0)
    return 1.23;
  //delay(50);
  uint8_t pressOutH = read(LPS22HB_PRES_OUT_H);
  uint8_t pressOutL = read(LPS22HB_PRES_OUT_L);
  uint8_t pressOutXL = read(LPS22HB_PRES_OUT_XL);

  long val = ( ((long)pressOutH << 16) | ((long)pressOutL << 8) | (long)pressOutXL );
  //if (val == 1.00) readPressure();
  return val/4096.0f;
}

uint32_t LPS22HB::readPressureRAW() {
  write(LPS22HB_CTRL_REG2, 0x1);

  if (status(0x1) < 0)
    return 123;
  //delay(50);
  uint8_t pressOutH = read(LPS22HB_PRES_OUT_H);
  uint8_t pressOutL = read(LPS22HB_PRES_OUT_L);
  uint8_t pressOutXL = read(LPS22HB_PRES_OUT_XL);

  int32_t val = ( (pressOutH << 16) | (pressOutL << 8) | pressOutXL );
  val=val+0x400000;
  //if (val == 1.00) readPressure();
  return (uint32_t)val;
}

uint32_t LPS22HB::readPressureUI() {
  write(LPS22HB_CTRL_REG2, 0x1);

  if (status(0x1) < 0)
    return 1.23;
  //delay(50);
  uint8_t pressOutH = read(LPS22HB_PRES_OUT_H);
  uint8_t pressOutL = read(LPS22HB_PRES_OUT_L);
  uint8_t pressOutXL = read(LPS22HB_PRES_OUT_XL);

  uint32_t val = ((pressOutH << 16) | (pressOutL << 8) | pressOutXL );
  //if (val == 1.00) readPressure();
  return val/4096;
}

float LPS22HB::readTemperature() {
  write(LPS22HB_CTRL_REG2, 0x1);
  if (status(0x2) < 0)
    return 4.56;

  uint8_t tempOutH = read(LPS22HB_TEMP_OUT_H);
  uint8_t tempOutL = read(LPS22HB_TEMP_OUT_L);

  int16_t val = (tempOutH << 8) | (tempOutL & 0xff);
  return ((float)val)/100.0f;
}


uint8_t LPS22HB::status(uint8_t status) {
  int count = 1000;
  uint8_t data = 0xff;
  do {
    data = read(LPS22HB_STATUS_REG);
    --count;
    if (count < 0)
      break;
  } while ((data & status) == 0);

  if (count < 0)
    return -1;
  else
    return 0;
}

uint8_t LPS22HB::read(uint8_t reg) {
  Wire1.beginTransmission(_address);
  Wire1.write(reg);
  Wire1.endTransmission();
  Wire1.requestFrom(_address, 1);
  return Wire1.read();
}

void LPS22HB::write(uint8_t reg, uint8_t data) {
  Wire1.beginTransmission(_address);
  Wire1.write(reg);
  Wire1.write(data);
  Wire1.endTransmission();
}
