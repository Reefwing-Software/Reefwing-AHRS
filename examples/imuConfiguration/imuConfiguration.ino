/******************************************************************
  @file       imuConfiguration.ino
  @brief      Read IMU configuration for the Nano 33 BLE.
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     1.0.0
  Date:        22/02/22

  1.0.0 Original Release.           22/02/22

******************************************************************/

#include <NexgenAHRS.h>

LSM9DS1 imu;
LPS22HB barometer;

void setup() {
  // Initialise the LSM9DS1 IMU
  imu.begin();
  barometer.begin();

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  if (imu.connected()) {
    Serial.println("LSM9DS1 Accelerometer, Magnetometer and Gyroscope are connected.");
    Serial.print("Gyro chip temperature is ");
    Serial.print(imu.readGyroTemp(), 1);  
    Serial.println(" degrees C.");
  }
  else {
    Serial.println("LSM9DS1 Accelerometer, Magnetometer and Gyroscope not found.");
  }

  if (barometer.connected()) {
    barometer.readTemperature();  //  discard 1st reading, normally zero.
    Serial.print("LSM9DS1 Barometer is connected. Barometer temperature is ");
    Serial.print(barometer.readTemperature(), 1);  
    Serial.println(" degrees C.");
  }
  else {
    Serial.println("LSM9DS1 Barometer not found.");
  }

  
}

void loop() {
    // put your main code here, to run repeatedly:
    
}