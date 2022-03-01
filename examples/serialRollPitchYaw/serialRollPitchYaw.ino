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
SensorData data;

int loopFrequency = 0;
const long displayPeriod = 1000;
unsigned long previousMillis = 0;

void setup() {
  // Initialise the LSM9DS1 IMU
  imu.begin();

  //  Positive magnetic declination - Sydney, AUSTRALIA
  imu.setDeclination(12.717);
  imu.setFusionAlgorithm(SensorFusion::MADGWICK);

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  if (imu.connected()) {
      //    This sketch assumes that the LSM9DS1 is already calibrated, 
      //    If so, start processing IMU data. If not, run the testAndCalibrate 
      //    sketch first.
      imu.start();
  }
}

void loop() {
    //  Check for new IMU data and update angles
    data = imu.update();

    //  Display sensor data every displayPeriod, non-blocking.
    if (millis() - previousMillis >= displayPeriod) {
      Serial.print("Roll:\t");
      Serial.print(data.roll);
      Serial.print(" Pitch:\t");
      Serial.print(data.pitch);
      Serial.print(" Yaw:\t");
      Serial.print(data.yaw);
      Serial.print(" Heading:\t");
      Serial.print(data.heading);
      Serial.print(" Loop Frequency:\t");
      Serial.print(loopFrequency);
      Serial.println(" Hz");

      loopFrequency = 0;
      previousMillis = millis();
    }

    loopFrequency++;
}