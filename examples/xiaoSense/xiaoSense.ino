/******************************************************************
  @file       xiaoSense.ino
  @brief      Print roll, pitch, yaw and heading angles using the
              LSM6DS3 IMU on the Seeed Xiao Sense
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     2.2.0
  Date:        10/02/23

  1.0.0 Original Release.                         22/02/22
  1.1.0 Added NONE fusion option.                 25/05/22
  2.0.0 Changed Repo & Branding                   15/12/22
  2.0.1 Invert Gyro Values PR                     24/12/22
  2.1.0 Updated Fusion Library                    30/12/22
  2.2.0 Add support for Nano 33 BLE Sense Rev. 2  10/02/23

  This sketch is configured to work with the CLASSIC, KALMAN & NONE 
  Sensor Fusion options. Set the algorithm that you wish to use 
  with:

  ahrs.setFusionAlgorithm(SensorFusion::CLASSIC);

  The other Sensor Fusion algorithms require a 9 DOF IMU (i.e., 
  a magnetometer).

******************************************************************/


#include <Wire.h>
#include <LSM6DS3.h>
#include <ReefwingAHRS.h>


ReefwingAHRS ahrs;
SensorData data;
LSM6DS3 imu(I2C_MODE, LSM6DS3_ADDRESS);

int loopFrequency = 0;
const long displayPeriod = 1000;
unsigned long previousMillis = 0;

void setup() {
  //  Initialise the AHRS
  //  Use default fusion algo and parameters
  ahrs.begin();
  
  ahrs.setFusionAlgorithm(SensorFusion::CLASSIC);
  ahrs.setDeclination(12.717);                      //  Sydney, Australia

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  Serial.print("Detected Board - ");
  Serial.println(ahrs.getBoardTypeString());

  if (imu.begin() == 0) {
    Serial.println("LSM6DS3 IMU Connected."); 
    Serial.println("Gyro/Accelerometer sample rate = 416 Hz");
  } 
  else {
    Serial.println("LSM6DS3 IMU Not Detected.");
    while(1);
  }
}

void loop() {
  data.gx = imu.readFloatGyroX();
  data.gy = imu.readFloatGyroY();
  data.gz = imu.readFloatGyroZ();
  data.ax = imu.readFloatAccelX();
  data.ay = imu.readFloatAccelY();
  data.az = imu.readFloatAccelZ();

  ahrs.setData(data);
  ahrs.update();

  if (millis() - previousMillis >= displayPeriod) {
    //  Display sensor data every displayPeriod, non-blocking.
    Serial.print("--> Roll: ");
    Serial.print(ahrs.angles.roll, 2);
    Serial.print("\tPitch: ");
    Serial.print(ahrs.angles.pitch, 2);
    Serial.print("\tYaw: ");
    Serial.print(ahrs.angles.yaw, 2);
    Serial.print("\tHeading: ");
    Serial.print(ahrs.angles.heading, 2);
    Serial.print("\tLoop Frequency: ");
    Serial.print(loopFrequency);
    Serial.println(" Hz");

    loopFrequency = 0;
    previousMillis = millis();
  }

  loopFrequency++;
}