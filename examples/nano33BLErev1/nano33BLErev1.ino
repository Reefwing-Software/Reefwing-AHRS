/******************************************************************
  @file       nano33BLErev1.ino
  @brief      Print roll, pitch, yaw and heading angles using the
              LSM9DS1 IMU on the Nano 33 BLE rev1
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

  This sketch is congigured to work with the MADGWICK, MAHONY,
  CLASSIC and COMPLEMENTARY Sensor Fusion options. Set the 
  algorithm that you wish to use with:

  ahrs.setFusionAlgorithm(SensorFusion::MADGWICK);

  If you want to test the FUSION (Madgwick v2) algoritm, then use
  the fusionAHRS example sketch or add the FUSION specific
  configuration to this sketch.

******************************************************************/

#include <ReefwingAHRS.h>
#include <ReefwingLSM9DS1.h>

ReefwingLSM9DS1 imu;
ReefwingAHRS ahrs;

int loopFrequency = 0;
const long displayPeriod = 1000;
unsigned long previousMillis = 0;

void setup() {
  //  Initialise the LSM9DS1 IMU & AHRS
  //  Use default fusion algo and parameters
  imu.begin();
  ahrs.begin();
  
  ahrs.setFusionAlgorithm(SensorFusion::MADGWICK);
  ahrs.setDeclination(12.717);                      //  Sydney, Australia

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  if (imu.connected()) {
    Serial.println("LSM9DS1 IMU Connected."); 
    Serial.println("Calibrating IMU...\n"); 
    imu.start();
    imu.calibrateGyro();
    imu.calibrateAccel();
    imu.calibrateMag();

    delay(20);
    //  Flush the first reading - this is important!
    //  Particularly after changing the configuration.
    imu.readGyro();
    imu.readAccel();
    imu.readMag();
  } else {
    Serial.println("LSM9DS1 IMU Not Detected.");
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
    Serial.print(ahrs.angles.roll);
    Serial.print("\tPitch: ");
    Serial.print(ahrs.angles.pitch);
    Serial.print("\tYaw: ");
    Serial.print(ahrs.angles.yaw);
    Serial.print("\tHeading: ");
    Serial.print(ahrs.angles.heading);
    Serial.print("\tLoop Frequency: ");
    Serial.print(loopFrequency);
    Serial.println(" Hz");

    loopFrequency = 0;
    previousMillis = millis();
  }

  loopFrequency++;
}