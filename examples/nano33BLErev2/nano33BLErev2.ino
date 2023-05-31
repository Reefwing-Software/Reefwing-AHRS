/******************************************************************
  @file       nano33BLErev2.ino
  @brief      Print roll, pitch, yaw and heading angles using the
              BMI270/BMM150 IMUs on the Nano 33 BLE rev2
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

  imu.setFusionAlgorithm(SensorFusion::MADGWICK);

  If you want to test the FUSION (Madgwick v2) algoritm, then use
  the fusionAHRS example sketch or add the FUSION specific
  configuration to this sketch.

******************************************************************/

#include <ReefwingAHRS.h>
#include <Arduino_BMI270_BMM150.h>

EulerAngles angles;
SensorData data;

int loopFrequency = 0;
const long displayPeriod = 1000;
unsigned long previousMillis = 0;

void setup() {
  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize BMI270 IMU!");
    while (1);
  }

  Serial.println("BMI270 IMU Connected."); 
}

void loop() {
  //  Check for new IMU sensor data and update angles
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(data.ax, data.ay, data.az);    //  Acceleration in G's
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(data.gx, data.gy, data.gz);       //  Angular rate in degrees/second
  }

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(data.mx, data.my, data.mz);   //  Magnetic Field in uT
  }



  angles = imu.update();

  //  Display sensor data every displayPeriod, non-blocking.
  if (millis() - previousMillis >= displayPeriod) {

    //  Uncomment to DEBUG raw sensor data:
    //  SensorData data = imu.rawData();
    //  Serial.print("ax = "); Serial.print(1000*data.ax);  
    //  Serial.print(" ay = "); Serial.print(1000*data.ay); 
    //  Serial.print(" az = "); Serial.print(1000*data.az); Serial.println(" mg");
    //  Serial.print("gx = "); Serial.print( data.gx, 2); 
    //  Serial.print(" gy = "); Serial.print( data.gy, 2); 
    //  Serial.print(" gz = "); Serial.print( data.gz, 2); Serial.println(" deg/s");
    //  Serial.print("mx = "); Serial.print(1000*data.mx ); 
    //  Serial.print(" my = "); Serial.print(1000*data.my ); 
    //  Serial.print(" mz = "); Serial.print(1000*data.mz ); Serial.println(" mG");
      
    Serial.print("Roll: ");
    Serial.print(angles.roll);
    Serial.print("\tPitch: ");
    Serial.print(angles.pitch);
    Serial.print("\tYaw: ");
    Serial.print(angles.yaw);
    Serial.print("\tHeading: ");
    Serial.print(angles.heading);
    Serial.print("\tLoop Frequency: ");
    Serial.print(loopFrequency);
    Serial.println(" Hz");

    loopFrequency = 0;
    previousMillis = millis();
  }

  loopFrequency++;
}