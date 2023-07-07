/******************************************************************
  @file       portentaH7.ino
  @brief      Print roll, pitch, yaw and heading angles using the
              MPU6500 IMU and the Portenta H7.
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

  This sketch is configured to work with the CLASSIC, KALMAN & NONE Sensor 
  Fusion options. Set the algorithm that you wish to use with:

  imu.setFusionAlgorithm(SensorFusion::MADGWICK);

  We are using the FastIMU MPU6500 library. Substitute the
  appropriate library and setup/calibration for your IMU

******************************************************************/

#include <ReefwingAHRS.h>
#include <FastIMU.h>

ReefwingAHRS ahrs;
SensorData data;

#define IMU_ADDRESS 0x68    //  Change to the address of the IMU
#define PERFORM_CALIBRATION //  Comment to disable startup calibration
MPU6500 IMU;                //  Change to the name of any supported IMU! 

//  FastIMU - Currently supported IMUs: MPU9255 MPU9250 MPU6886 MPU6500 MPU6050 ICM20689 
//  ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL.

calData calib = { 0 };  //  Calibration data
AccelData accelData;    //  Sensor data
GyroData gyroData;
MagData magData;

int loopFrequency = 0;
const long displayPeriod = 1000;
unsigned long previousMillis = 0;

void setup() {
  //  Initialise the AHRS
  //  begin() will detect the Portenta H7 but this has no default IMU
  ahrs.begin();
  ahrs.setImuType(ImuType::MPU6500);
  ahrs.setDOF(DOF::DOF_6);
  
  ahrs.setFusionAlgorithm(SensorFusion::CLASSIC);
  ahrs.setDeclination(12.717);                      //  Sydney, Australia

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  Serial.print("Detected Board - ");
  Serial.println(ahrs.getBoardTypeString());

  int err = IMU.init(calib, IMU_ADDRESS);

  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (1);
  }
  else {
    Serial.println("MPU6500 IMU Connected."); 
  }

  #ifdef PERFORM_CALIBRATION
  Serial.println("FastIMU calibration & data example");
  if (IMU.hasMagnetometer()) {
    delay(1000);
    Serial.println("Move IMU in figure 8 pattern until done.");
    delay(3000);
    IMU.calibrateMag(&calib);
    Serial.println("Magnetic calibration done!");
  }
  else {
    delay(5000);
  }

  delay(5000);
  Serial.println("Keep IMU level.");
  delay(5000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  if (IMU.hasMagnetometer()) {
    Serial.println("Mag biases X/Y/Z: ");
    Serial.print(calib.magBias[0]);
    Serial.print(", ");
    Serial.print(calib.magBias[1]);
    Serial.print(", ");
    Serial.println(calib.magBias[2]);
    Serial.println("Mag Scale X/Y/Z: ");
    Serial.print(calib.magScale[0]);
    Serial.print(", ");
    Serial.print(calib.magScale[1]);
    Serial.print(", ");
    Serial.println(calib.magScale[2]);
  }
  delay(5000);
  IMU.init(calib, IMU_ADDRESS);
#endif

  //err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  //err = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO ±500 DPS AND THE ACCELEROMETER RANGE TO ±2g
  
  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
}

void loop() {
  IMU.update();
  IMU.getAccel(&accelData);

  data.ax = accelData.accelX;
  data.ay = accelData.accelY;
  data.az = accelData.accelZ;

  IMU.getGyro(&gyroData);
  data.gx = gyroData.gyroX;
  data.gy = gyroData.gyroY;
  data.gz = gyroData.gyroZ;
  
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