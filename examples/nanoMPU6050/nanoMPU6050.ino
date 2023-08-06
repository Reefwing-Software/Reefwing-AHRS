/******************************************************************
  @file       nanoMPU6050.ino
  @brief      Print roll, pitch, yaw and heading angles using the
              MPU6050 IMU on the Arduino AVR Nano.
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

#include <ReefwingAHRS.h>
#include <ReefwingMPU6050.h>

ReefwingMPU6050 imu;
ReefwingAHRS ahrs;

//  Display and Loop Frequency
int loopFrequency = 0;
const long displayPeriod = 1000;
unsigned long previousMillis = 0;

void setup() {
  //  Initialise the MPU6050 IMU & AHRS
  //  Use default fusion algo and parameters
  imu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
  ahrs.begin();

  //  Nano will be detected but could use any IMU
  //  If your IMU isn't autodetected and has a mag you need
  //  to add: ahrs.setDOF(DOF::DOF_9);
  ahrs.setDOF(DOF::DOF_6);
  ahrs.setImuType(ImuType::MPU6050);
  ahrs.setFusionAlgorithm(SensorFusion::KALMAN);
  ahrs.setDeclination(12.717);                      //  Sydney, Australia

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  if (imu.connected()) {
    Serial.println("MPU6050 IMU Connected."); 

    imu.calibrateGyro();
    delay(20);

    //  Flush the first reading - this is important!
    //  Particularly after changing the configuration.
    imu.readRawGyro();
    imu.readRawAccel();
  } else {
    Serial.println("MPU6050 IMU Not Detected.");
    while(1);
  }
}

void loop() {
  imu.updateSensorData();
  ahrs.setData(imu.data);
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