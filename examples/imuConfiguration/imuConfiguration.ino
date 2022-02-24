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

void setup() {
  // Initialise the LSM9DS1 IMU
  imu.begin();

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  if (imu.whoAmIGyro() == LSM9DS1XG_WHO_AM_I_VALUE) {
    Serial.print("LSM9DS1 Accelerometer and Gyroscope is available.");
    Serial.print("Gyro temperature is ");  
    Serial.print(imu.readTemperature(), 1);  
    Serial.println(" degrees C");
  }
  else {
    Serial.println("LSM9DS1 Accelerometer and Gyroscope not found.");
  }

  if (imu.whoAmIMag() == LSM9DS1M_WHO_AM_I_VALUE) {
    Serial.println("LSM9DS1 Magnetometer is available.");
  }
  else {
    Serial.println("LSM9DS1 Magnetometer not found.");
  }

  if (imu.whoAmIBaro() == LPS22HB_WHO_AM_I_VALUE) {
    Serial.println("LSM9DS1 Barometer is available.");
  }
  else {
    Serial.println("LSM9DS1 Barometer not found.");
  }

  
}

void loop() {
    // put your main code here, to run repeatedly:
    
}