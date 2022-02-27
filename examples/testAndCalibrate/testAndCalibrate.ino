/******************************************************************
  @file       testAndCalibrate.ino
  @brief      Perform self test and calibration on the LSM9DS1 IMU.
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     1.0.0
  Date:        22/02/22

  1.0.0 Original Release.           22/02/22

******************************************************************/

#include <NexgenAHRS.h>

LSM9DS1 imu;
SelfTestResults results;

void setup() {
  // Initialise the LSM9DS1 IMU
  imu.begin();

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  if (imu.connected()) {
      results = imu.selfTest();

      Serial.println("Gyro self-test results: ");
      Serial.print("x-axis = "); 
      Serial.print(results.gyrodx); 
      Serial.print(" dps"); 
      Serial.println(" should be between 20 and 250 dps");

      Serial.print("y-axis = "); 
      Serial.print(results.gyrody); 
      Serial.print(" dps"); 
      Serial.println(" should be between 20 and 250 dps");

      Serial.print("z-axis = "); 
      Serial.print(results.gyrodz); 
      Serial.print(" dps"); 
      Serial.println(" should be between 20 and 250 dps");

      Serial.println("Accelerometer self-test results: ");
      Serial.print("x-axis = "); 
      Serial.print(results.accdx); 
      Serial.print(" mg"); 
      Serial.println(" should be between 60 and 1700 mg");

      Serial.print("y-axis = "); 
      Serial.print(results.accdy); 
      Serial.print(" mg"); 
      Serial.println(" should be between 60 and 1700 mg");

      Serial.print("z-axis = "); 
      Serial.print(results.accdz); 
      Serial.print(" mg"); 
      Serial.println(" should be between 60 and 1700 mg");
  }
  else {
    Serial.println("LSM9DS1 Accelerometer, Magnetometer and Gyroscope not found.");
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
