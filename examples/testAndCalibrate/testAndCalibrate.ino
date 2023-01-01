/******************************************************************
  @file       testAndCalibrate.ino
  @brief      Perform self test and calibration on the LSM9DS1 IMU.
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
SelfTestResults results;
BiasOffsets biasOffsets;

bool sensorInRange(float x, float y, float z, float min, float max) {
  return ((min <= x && x <= max) &&
          (min <= y && y <= max) &&
          (min <= z && z <= max)
         );
}

void printCode(char method[], float bias[3]) {   
  Serial.print(method);
  Serial.print("(");
  Serial.print(bias[0], 6); Serial.print(", ");
  Serial.print(bias[1], 6); Serial.print(", ");
  Serial.print(bias[2], 6); Serial.println(");");
}

void setup() {
  // Initialise the LSM9DS1 IMU
  imu.begin();

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  if (imu.connected()) {
    Serial.println("Gyroscope SELF TEST");
    Serial.println("Place the Nano 33 BLE on a flat surface and don't move it.");
    
    results = imu.selfTest();

    Serial.print("x-axis: "); 
    Serial.print(results.gyrodx); 
    Serial.println(" dps"); 

    Serial.print("y-axis: "); 
    Serial.print(results.gyrody); 
    Serial.println(" dps"); 

    Serial.print("z-axis: "); 
    Serial.print(results.gyrodz); 
    Serial.println(" dps"); 

    Serial.println("Gyroscope x, y and z axis expected range: 20 - 250 dps");

    if (sensorInRange(results.gyrodx, results.gyrody, results.gyrodz, 20.0, 250.0)) {
      Serial.println("GYROSCOPE PASSED SELF-TEST");
    }
    else {
      Serial.println("GYROSCOPE FAILED SELF-TEST");
    }

    Serial.println("\nAccelerometer SELF TEST");
    Serial.print("x-axis: "); 
    Serial.print(results.accdx); 
    Serial.println(" mg"); 

    Serial.print("y-axis = "); 
    Serial.print(results.accdy); 
    Serial.println(" mg"); 

    Serial.print("z-axis = "); 
    Serial.print(results.accdz); 
    Serial.println(" mg"); 

    Serial.println("Accelerometer x, y, and z axis expected range: 60 - 1700 mg");

    if (sensorInRange(results.accdx, results.accdy, results.accdz, 60.0, 1700.0)) {
      Serial.println("ACCELEROMETER PASSED SELF-TEST");
    }
    else {
      Serial.println("ACCELEROMETER FAILED SELF-TEST");
    }

    Serial.println("\nIMU CALIBRATION");

    // Calibrate gyroscope and accelerometer. Load offset biases into the bias registers
    imu.calibrateAccGyro();

    Serial.println("\nACCELEROMETER & GYROSCOPE CALIBRATION COMPLETE.");

    Serial.println("\nMAGNETOMETER CALIBRATION:");
    Serial.println("Wave the Nano 33 BLE in a vertical figure of eight until calibration is complete.");
    imu.calibrateMag();
    Serial.println("\nMAGNETOMETER CALIBRATION COMPLETE.");

    biasOffsets = imu.getBiasOffsets();

    Serial.println("\nAccelerometer Offset Bias (mg):"); 
    Serial.println(1000.0 * biasOffsets.accelBias[0]); 
    Serial.println(1000.0 * biasOffsets.accelBias[1]); 
    Serial.println(1000.0 * biasOffsets.accelBias[2]);

    Serial.println("\nGyroscope Offset Bias (dps):"); 
    Serial.println(biasOffsets.gyroBias[0]); 
    Serial.println(biasOffsets.gyroBias[1]); 
    Serial.println(biasOffsets.gyroBias[2]);

    Serial.println("\nMagnetometer Offset Bias (mG):"); 
    Serial.println(1000.0 * biasOffsets.magBias[0]); 
    Serial.println(1000.0 * biasOffsets.magBias[1]); 
    Serial.println(1000.0 * biasOffsets.magBias[2]); 

    Serial.println("\nLSM9DS1 IMU CALIBRATION COMPLETE.");

    Serial.println("\n\n  Code to copy and paste into your sketch:\n");

    printCode("\timu.loadAccBias", biasOffsets.accelBias);
    printCode("\timu.loadGyroBias", biasOffsets.gyroBias);
    printCode("\timu.loadMagBias", biasOffsets.magBias);
  }
  else {
    Serial.println("LSM9DS1 IMU not found.");
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
