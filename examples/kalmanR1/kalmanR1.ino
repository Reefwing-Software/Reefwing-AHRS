/******************************************************************
  @file       kalmanR1.ino
  @brief      Print roll, pitch, yaw and heading angles using the
              LSM9DS1 IMU and Kalman Filter on the Nano 33 BLE 
              and Nano 33 BLE Sense rev1.
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     1.0.0
  Date:        23/11/24

  1.0.0 Original Release.                         23/11/24

******************************************************************/

#include <ReefwingAHRS.h>
#include <ReefwingLSM9DS1.h>

ReefwingLSM9DS1 imu;
ReefwingAHRS ahrs; 
SensorData data;   

unsigned long previousMillis = 0; // Timer for 1-second updates
const unsigned long interval = 1000; // 1-second interval

void setup() {
  imu.begin();
  ahrs.begin();
  ahrs.setFusionAlgorithm(SensorFusion::EXTENDED_KALMAN);
  ahrs.setDeclination(12.717);                      //  Sydney, Australia

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Reefwing AHRS with Extended Kalman Filter");
  Serial.print("Detected Board - ");
  Serial.println(ahrs.getBoardTypeString());

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
  } 
  else {
    Serial.println("LSM9DS1 IMU Not Detected.");
    while(1);
  }
}

void loop() {
  imu.updateSensorData();
  ahrs.setData(imu.data);
  ahrs.update();

  // Print roll, pitch, and yaw every second
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    Serial.print("Roll: ");
    Serial.print(ahrs.angles.roll, 2);
    Serial.print("°, Pitch: ");
    Serial.print(ahrs.angles.pitch, 2);
    Serial.print("°, Yaw: ");
    Serial.print(ahrs.angles.yaw, 2);
    Serial.println("°");

    previousMillis = currentMillis;
  }
}