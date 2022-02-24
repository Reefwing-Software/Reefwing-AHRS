/******************************************************************
  @file       NexgenAHRS.h
  @brief      Attitude and Heading Reference System (AHRS) for the Nano 33 BLE.
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     1.0.0
  Date:        22/02/22

  1.0.0 Original Release.           22/02/22

******************************************************************/

#ifndef NexgenAHRS_h
#define NexgenAHRS_h

#include <Arduino.h>

#include "LPS22HB.h"

class LSM9DS1 {
    public:
        LSM9DS1();

        void begin();
        uint8_t whoAmIGyro();
        uint8_t whoAmIMag();
        uint8_t whoAmIBaro();
        float readTemperature();

    private:
        LPS22HB lps22hb;
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
        float seaLevelPressure; 
        float Pressure;
        float pressureArray[10];
        

};

#endif