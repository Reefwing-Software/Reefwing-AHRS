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

class LSM9DS1 {
    public:
        LSM9DS1();

        void begin();
        bool connected();
        uint8_t whoAmIGyro();
        uint8_t whoAmIMag();
        float readGyroTemp();

    private:
        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
        uint8_t readByte(uint8_t address, uint8_t subAddress);
        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);

        uint8_t OSR;
        uint8_t Gscale;
        uint8_t Godr;
        uint8_t Gbw;
        uint8_t Ascale;
        uint8_t Aodr;
        uint8_t Abw;
        uint8_t Mscale;
        uint8_t Modr;
        uint8_t Mmode;  
        float aRes, gRes, mRes; 
};

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