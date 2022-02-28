/******************************************************************
  @file       NexgenAHRS.h
  @brief      Attitude and Heading Reference System (AHRS) for the Nano 33 BLE.
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     1.0.0
  Date:        22/02/22

  1.0.0 Original Release.           22/02/22

  Credit - LPS22HB Absolute Digital Barometer class 
           based on work by Adrien Chapelet for IoThings.
           ref: https://github.com/adrien3d/IO_LPS22HB

******************************************************************/

#ifndef NexgenAHRS_h
#define NexgenAHRS_h

#include <Arduino.h>

/******************************************************************
 *
 * ENUM Class & Struct Definitions - 
 * 
 ******************************************************************/

enum class Ascale {  // set of allowable accel full scale settings
  AFS_2G = 0,
  AFS_16G,
  AFS_4G,
  AFS_8G
};

enum class Gscale {  // set of allowable gyro full scale settings
  GFS_245DPS = 0,
  GFS_500DPS,
  GFS_NoOp,
  GFS_2000DPS
};

enum class Mscale {  // set of allowable mag full scale settings
  MFS_4G = 0,
  MFS_8G,
  MFS_12G,
  MFS_16G
};

struct SelfTestResults {
  float gyrodx;
  float gyrody;
  float gyrodz;
  float accdx;
  float accdy;
  float accdz;
};

struct BiasOffsets {
  float gyroBias[3]; 
  float accelBias[3];  
  float magBias[3]; 
};

/******************************************************************
 *
 * LSM9DS1 Class Definition - 
 * 
 ******************************************************************/

class LSM9DS1 {
    public:
        LSM9DS1();

        void begin();
        bool connected();
        uint8_t whoAmIGyro();
        uint8_t whoAmIMag();
        float readGyroTemp();
        void setAccResolution(Ascale ascale);
        void setGyroResolution(Gscale gscale);
        void setMagResolution(Mscale mscale);
        float getAccResolution();
        float getGyroResolution();
        float getMagResolution();
        void calibrateAccGyro();
        void calibrateMag();
        
        SelfTestResults selfTest();
        BiasOffsets getBiasOffsets();


    private:
        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
        uint8_t readByte(uint8_t address, uint8_t subAddress);
        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
        void setBiasOffsets(float *dest1, float *dest2);
        void setMagneticBias(float *dest1);

        uint8_t OSR, Godr, Gbw, Aodr, Abw, Modr, Mmode;  
        uint8_t aScale, gScale, mScale;
        float aRes, gRes, mRes; 
        float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0},  magBias[3] = {0, 0, 0}; 
};

/******************************************************************
 *
 * LPS22HB Class Definition - 
 * 
 ******************************************************************/

class LPS22HB {
public:
  LPS22HB();

  void begin();
  bool connected();

  uint8_t whoAmI();
  float readTemperature();

  float readPressure();
  uint32_t readPressureUI();
  uint32_t readPressureRAW();

private:
  uint8_t _address;
  uint8_t read(uint8_t reg);
  void write(uint8_t reg, uint8_t data);
  uint8_t status(uint8_t data);
};

#endif