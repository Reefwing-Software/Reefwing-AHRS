/******************************************************************
  @file       ReefwingAHRS.h
  @brief      Attitude and Heading Reference System (AHRS)
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     2.3.2
  Date:        31/12/24

  1.0.0 Original Release.                         22/02/22
  1.1.0 Added NONE fusion option.                 25/05/22
  2.0.0 Changed Repo & Branding                   15/12/22
  2.0.1 Invert Gyro Values PR                     24/12/22
  2.1.0 Updated Fusion Library                    30/12/22
  2.2.0 Add support for Nano 33 BLE Sense Rev. 2  10/02/23
  2.3.0 Extended Kalman Filter added              20/11/24
  2.3.1 Madgwick filter bug fixed                 30/12/24
  2.3.2 Improved normalization for Madgwick       31/12/24

  Credits: - The C++ code for our quaternion position update 
             using the Madgwick Filter is based on the paper, 
             "An efficient orientation filter for inertial and 
             inertial/magnetic sensor arrays" written by Sebastian 
             O.H. Madgwick in April 30, 2010.
         
******************************************************************/

#ifndef REEFWING_AHRS_H
#define REEFWING_AHRS_H

#include <Arduino.h>
#include <Reefwing_imuTypes.h>

#include "ExtendedKalmanFilter.h"

/******************************************************************
 *
 * Definitions - 
 * 
 ******************************************************************/

#define DEG_TO_RAD 0.01745329251994   //  PI/180 * degrees
#define RAD_TO_DEG 57.2957795130823   //  180/PI * radians

#ifndef BOARD_NAME
#define BOARD_NAME 
#endif

/******************************************************************
 *
 * ENUM Class & Struct Definitions - 
 * 
 ******************************************************************/

enum class BoardType {
  NANO = 0,
  NANO33BLE,
  NANO33BLE_SENSE_R1,
  NANO33BLE_SENSE_R2,
  XIAO_SENSE,
  PORTENTA_H7,
  VIDOR_4000,
  NANO33IOT,
  NOT_DEFINED
};

enum class DOF {
  DOF_6 = 0,
  DOF_9
};

enum class ImuType {
  LSM9DS1 = 0,
  LSM6DS3,
  BMI270_BMM150,
  MPU6050,
  MPU6500,
  UNKNOWN
};

enum class SensorFusion { // Sensor fusion algorithm options
  MADGWICK = 0,
  MAHONY,
  COMPLEMENTARY,
  CLASSIC,
  EXTENDED_KALMAN,
  NONE
};

/******************************************************************
 *
 * ReefwingAHRS Class Definition - 
 * 
 ******************************************************************/

class ReefwingAHRS {
  public:
    ReefwingAHRS();

    void begin();
    void update();
    void reset();

    void setFusionAlgorithm(SensorFusion algo);
    void setAlpha(float a);
    void setBeta(float b);
    void setGyroMeasError(float gme);
    void setKp(float p);
    void setKi(float i);
    void setDeclination(float dec);
    void setData(SensorData d, bool axisAlign = true);
    void setDOF(DOF d);
    void setImuType(ImuType i);
    void setBoardType(BoardType b);

    void updateEulerAngles(float deltaT);
    void classicUpdate();
    void tiltCompensatedYaw();
    void madgwickUpdate(SensorData d, float deltaT); 
    void mahoneyUpdate(SensorData d, float deltaT);
    void extendedKalmanUpdate(SensorData d, float deltaT); 
    void complementaryUpdate(SensorData d, float deltaT);
    
    SensorData gyroToRadians();
    BoardType getBoardType();
    const char* getBoardTypeString();

    void formatAnglesForConfigurator();
    Quaternion getQuaternion();
    EulerAngles angles, configAngles;

  private:
    long _lastUpdate;                                //  Time since last update in micro-seconds (us)
    float _declination;
    float _gyroMeasError, _alpha, _beta, _Kp, _Ki;   //  Sensor Fusion free parameters
    float _eInt[3] = {0.0f, 0.0f, 0.0f};             //  Vector to hold integral error for Mahony filter
    float _att[4] = {1.0f, 0.0f, 0.0f, 0.0f};        //  Attitude quaternion for complementary filter

    DOF _dof;
    ImuType _imuType;
    BoardType _boardType;
    SensorFusion _fusion;
    
    SensorData _data;   //  Sensor Data: gyro - DPS, accel - g's, mag - gauss
    Quaternion _q;      //  Quaternion used for AHRS update

    const char* _boardTypeStr[9];

    // Extended Kalman Filter specific variables
    ExtendedKalmanFilter ekf; // EKF instance
    float state[2];           // State vector [roll, pitch]
    float P[2][2];            // Error covariance matrix
    float Q[2][2];            // Process noise covariance
    float R[2][2];            // Measurement noise covariance

};

#endif