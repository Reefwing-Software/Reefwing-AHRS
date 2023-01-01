/******************************************************************
  @file       imuConfiguration.ino
  @brief      Read IMU configuration for the Nano 33 BLE.
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     2.1.0
  Date:        15/12/22

  1.0.0 Original Release.           22/02/22
  1.1.0 Added NONE fusion option.   25/05/22
  2.0.0 Changed Repo & Branding     15/12/22
  2.0.1 Invert Gyro Values PR       24/12/22
  2.1.0 Updated Fusion Library      30/12/22

******************************************************************/

#include <ReefwingAHRS.h>

LSM9DS1 imu;
LPS22HB barometer;

void setup() {
  // Initialise the LSM9DS1 IMU and LPS22HB Barometer
  imu.begin();
  barometer.begin();

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  if (imu.connected()) {
    Serial.println("LSM9DS1 Accelerometer, Magnetometer and Gyroscope are connected.");
    Serial.print("Gyro chip temperature: ");
    Serial.print(imu.readGyroTemp(), 1);  
    Serial.println("°C.");

    Serial.print("Accelerometer sensitivity: "); 
    Serial.print(imu.getAccResolution()); 
    Serial.println(" mg/LSB");

    Serial.print("Gyroscope sensitivity: "); 
    Serial.print(imu.getGyroResolution()); 
    Serial.println(" mdps/LSB");

    Serial.print("Magnetometer sensitivity: "); 
    Serial.print(imu.getMagResolution()); 
    Serial.println(" mGauss/LSB");
  }
  else {
    Serial.println("LSM9DS1 Accelerometer, Magnetometer and Gyroscope not found.");
  }

  if (barometer.connected()) {
    barometer.readTemperature();  //  discard 1st reading, normally zero.
    Serial.print("\nLSM9DS1 Barometer is connected. Barometer temperature: ");
    Serial.print(barometer.readTemperature(), 1);  
    Serial.println("°C.");
  }
  else {
    Serial.println("LSM9DS1 Barometer not found.");
  }

  
}

void loop() {
    // put your main code here, to run repeatedly:
    
}