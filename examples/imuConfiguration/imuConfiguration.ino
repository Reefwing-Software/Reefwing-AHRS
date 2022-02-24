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

  Serial.print("LSM9DS1 Accelerometer/gyroscope "); 
  Serial.print("I AM "); 
  Serial.print(imu.whoAmIGyro(), HEX); 
  Serial.print(" I should be "); 
  Serial.println(0x68, HEX);

  Serial.print("LSM9DS1 Magnetometer "); 
  Serial.print("I AM "); 
  Serial.print(imu.whoAmIMag(), HEX); 
  Serial.print(" I should be "); 
  Serial.println(0x3D, HEX);

  Serial.print("LPS22HB Barometer "); 
  Serial.print("I AM "); 
  Serial.print(imu.whoAmIBaro(), HEX); 
  Serial.print(" I should be "); 
  Serial.println(0xB1, HEX);
}

void loop() {
    // put your main code here, to run repeatedly:
    Serial.print("Gyro temperature is ");  
    Serial.print(imu.readTemperature(), 1);  
    Serial.println(" degrees C");
    delay(1000);
}
