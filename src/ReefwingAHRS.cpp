/******************************************************************
  @file       ReefwingAHRS.cpp
  @brief      Attitude and Heading Reference System (AHRS) for the 
              Arduino Nano 33 BLE and XIAO Sense
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

  Credits: - The C++ code for our quaternion position update 
             using the Madgwick Filter is based on the paper, 
             "An efficient orientation filter for inertial and 
             inertial/magnetic sensor arrays" written by Sebastian 
             O.H. Madgwick in April 30, 2010.
           - Kalman filter code is forked from KalmanFilter (3e5d060)
             v1.0.2 by Kristian Lauszus. 
             Ref: https://github.com/TKJElectronics/KalmanFilter/tree/master

******************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "ReefwingAHRS.h"

/******************************************************************
 *
 * ReefwingAHRS Implementation - 
 * 
 ******************************************************************/

ReefwingAHRS::ReefwingAHRS() { 
  _boardTypeStr[0] = "Nano";
  _boardTypeStr[1] = "Nano 33 BLE";
  _boardTypeStr[2] = "Nano 33 BLE Sense";
  _boardTypeStr[3] = "Nano 33 BLE Sense Rev 2";
  _boardTypeStr[4] = "Seeed XIAO nRF52840 Sense";
  _boardTypeStr[5] = "MKR Portenta H7";
  _boardTypeStr[6] = "MKR Vidor 4000";
  _boardTypeStr[7] = "Nano 33 IoT";
  _boardTypeStr[8] = "Undefined Board Type";
}

void ReefwingAHRS::begin() {
  //  Detect Board Hardware & Associated IMU
  setBoardType(BoardType::NOT_DEFINED);
  setImuType(ImuType::UNKNOWN);
  setDOF(DOF::DOF_6);

  #if defined(ARDUINO_ARDUINO_NANO33BLE)    //  Nano 33 BLE found
  
    byte error;
    
    Wire1.begin();
    Wire1.beginTransmission(HTS221_ADDRESS);
    error = Wire1.endTransmission();

    if (error == 0) {
      setBoardType(BoardType::NANO33BLE_SENSE_R1);
      setImuType(ImuType::LSM9DS1);
      setDOF(DOF::DOF_9);
    }
    else {
      Wire1.beginTransmission(HS3003_ADDRESS);
      error = Wire1.endTransmission();

      if (error == 0) {
        setBoardType(BoardType::NANO33BLE_SENSE_R2);
        setImuType(ImuType::BMI270_BMM150);
        setDOF(DOF::DOF_9);
      }
      else {
        setBoardType(BoardType::NANO33BLE);
        setImuType(ImuType::LSM9DS1);
        setDOF(DOF::DOF_9);
      }
    }
  #elif defined(ARDUINO_AVR_NANO)   
    setBoardType(BoardType::NANO);
  #elif defined(ARDUINO_PORTENTA_H7_M7) 
    setBoardType(BoardType::PORTENTA_H7);
  #elif defined(ARDUINO_SAMD_MKRVIDOR4000)
    setBoardType(BoardType::VIDOR_4000);
  #elif defined(ARDUINO_SAMD_NANO_33_IOT)
    setBoardType(BoardType::NANO33IOT);
  #elif defined(BOARD_NAME)
    if (strncmp(BOARD_NAME, _boardTypeStr[4], 25) == 0) {
      setBoardType(BoardType::XIAO_SENSE);
      setImuType(ImuType::LSM6DS3);
      setDOF(DOF::DOF_6);
    }
  #endif

  //  Default Sensor Fusion Co-Efficients - see README.md
  _gyroMeasError = 40.0f * DEG_TO_RAD;   // gyroscope measurement error in rads/s (start at 40 deg/s)
  _alpha = 0.98; // default complementary filter coefficient
  _beta = sqrt(3.0f / 4.0f) * _gyroMeasError;   // compute beta
  _Kp = 2.0f * 5.0f; // These are the free parameters in the Mahony filter and fusion scheme, _Kp for proportional feedback, _Ki for integral
  _Ki = 0.0f;

  //  Set default sensor fusion algorithm
  setFusionAlgorithm(SensorFusion::MADGWICK);

  //  Set default magnetic declination - Sydney, NSW, AUSTRALIA
  setDeclination(12.717);

  //  Init Kalman Filter
  kalmanX.setAngle(0);     // Set starting angle - roll
  kalmanY.setAngle(0);     // Set starting angle - pitch
}

void ReefwingAHRS::update() {
  long now = micros();
  float deltaT = ((now - _lastUpdate)/1000000.0f); // Time elapsed since last update in seconds

  _lastUpdate = now;

  switch(_fusion) {
    case SensorFusion::MADGWICK:
      madgwickUpdate(gyroToRadians(), deltaT);
      angles = _q.toEulerAngles(_declination);
    break;
    case SensorFusion::MAHONY:
      mahoneyUpdate(gyroToRadians(), deltaT);
      angles = _q.toEulerAngles(_declination);
    break;
    case SensorFusion::COMPLEMENTARY:
      complementaryUpdate(gyroToRadians(), deltaT);
      angles = _q.toEulerAngles(_declination);
    break;
    case SensorFusion::CLASSIC:
      updateEulerAngles(deltaT);
      classicUpdate();
      if (_dof == DOF::DOF_9) {
        tiltCompensatedYaw();
      }
    break;
    case SensorFusion::KALMAN:
      kalmanUpdate(deltaT);
      if (_dof == DOF::DOF_9) {
        tiltCompensatedYaw();
      }
    break;
    case SensorFusion::NONE:
      updateEulerAngles(deltaT);
      if (_dof == DOF::DOF_9) {
        tiltCompensatedYaw();
      }
    break;
  }
}

SensorData ReefwingAHRS::gyroToRadians() {
  //  Convert gyro data from DPS to radians/sec.
  SensorData filterData = _data;

  filterData.gx = _data.gx * DEG_TO_RAD;
  filterData.gy = _data.gy * DEG_TO_RAD;
  filterData.gz = _data.gz * DEG_TO_RAD;

  return filterData;
}

void ReefwingAHRS::formatAnglesForConfigurator() {
  //  Adjust angle signs for consistent display
  //  in the Reefwing Configurator
  configAngles = angles;
  
  switch(_fusion) {
    case SensorFusion::MADGWICK:
      configAngles.roll = -angles.roll;
      configAngles.pitch = -angles.pitch;
      configAngles.pitchRadians = -angles.pitchRadians;
      configAngles.rollRadians = -angles.rollRadians;
    break;
    case SensorFusion::MAHONY:
      configAngles.roll = -angles.roll;
      configAngles.pitch = -angles.pitch;
      configAngles.pitchRadians = -angles.pitchRadians;
      configAngles.rollRadians = -angles.rollRadians;
      break;
    case SensorFusion::COMPLEMENTARY:
      configAngles.yaw = -angles.yaw;
      configAngles.yawRadians = -angles.yawRadians;
      break;
    case SensorFusion::CLASSIC:
      break;
    case SensorFusion::KALMAN:
      break;
    case SensorFusion::NONE:
      break;
    default:
      break;
  }
}

BoardType ReefwingAHRS::getBoardType() {
  return _boardType;
}

const char* ReefwingAHRS::getBoardTypeString() {
  uint8_t index = (uint8_t)_boardType;

  return _boardTypeStr[index];
}

void ReefwingAHRS::setFusionAlgorithm(SensorFusion algo) {
  _fusion = algo;
}

void ReefwingAHRS::setAlpha(float a) {
  _alpha = a;
}

void ReefwingAHRS::setBeta(float b) {
  _beta = b;
}

void ReefwingAHRS::setGyroMeasError(float gme) {
  _gyroMeasError = gme;
}

void ReefwingAHRS::setKp(float p) {
  _Kp = p;
}

void ReefwingAHRS::setKi(float i) {
  _Ki = i;
}

void ReefwingAHRS::setDeclination(float dec) {
  _declination = dec;
}

void ReefwingAHRS::setData(SensorData d, bool axisAlign) {
  //  If required, convert IMU data to a 
  //  consistent Reference Frame.
  _data = d;

  if (axisAlign) {
    switch(_imuType) {
      case ImuType::LSM9DS1:
        _data.mx = -d.mx;
      break;
      case ImuType::LSM6DS3:
      break;
      case ImuType::BMI270_BMM150:
        _data.my = -d.my;
      break;
      case ImuType::MPU6050:
      break;
      case ImuType::MPU6500:
      break;
      case ImuType::UNKNOWN:
      break;
      default:
      break;
    }
  }
}

void ReefwingAHRS::setDOF(DOF d) {
  _dof = d;
}

void ReefwingAHRS::setBoardType(BoardType b) {
  _boardType = b;
}

void ReefwingAHRS::setImuType(ImuType i) {
  _imuType = i;
}

Quaternion ReefwingAHRS::getQuaternion() {
  return _q;
}

/******************************************************************
 *
 * ReefwingAHRS - Update Methods 
 * 
 ******************************************************************/

void ReefwingAHRS::kalmanUpdate(float deltaT) {
  //  The formulas for the roll φ and pitch θ angles have an infinite number of solutions at multiples of 360°.
  //  The solution is to restrict either the roll or the pitch angle (but not both) to lie between -90° and +90°. 
  //  The convention used in the aerospace sequence is that the roll angle can range between -180° to +180° but 
  //  the pitch angle is restricted to -90° to +90°.
  //  Ref: Tilt Sensing Using a Three-Axis Accelerometer - NXP AN3461
  //  URL: https://www.nxp.com/docs/en/application-note/AN3461.pdf
  float accRollAngle  = atan2(_data.ay, _data.az) * RAD_TO_DEG;
  float accPitchAngle = atan(-_data.ax / sqrt(_data.ay * _data.ay + _data.az * _data.az)) * RAD_TO_DEG;

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((accRollAngle < -90 && _kalAngleX > 90) || (accRollAngle > 90 && _kalAngleX < -90)) {
    kalmanX.setAngle(accRollAngle);
    _kalAngleX = accRollAngle;
  } 
  else {
    _kalAngleX = kalmanX.getAngle(accRollAngle, _data.gx, deltaT); // Calculate the angle using a Kalman filter
  }

  if (abs(_kalAngleX) > 90) {
    _data.gy = -_data.gy; // Invert rate, so it fits the restriced accelerometer reading
  }
  _kalAngleY = kalmanY.getAngle(accPitchAngle, _data.gy, deltaT);

  //  Assign Kalman Filtered results
  angles.pitch = _kalAngleY;
  angles.roll = _kalAngleX;
  angles.pitchRadians = angles.pitch * DEG_TO_RAD;
  angles.rollRadians = angles.roll * DEG_TO_RAD;
}

void ReefwingAHRS::classicUpdate() {
  // Convert from force vector to angle using 3 axis formula - result in radians
  float accRollAngle  =  atan(-1 * _data.ay / sqrt(pow(_data.ax, 2) + pow(_data.az, 2)));
  float accPitchAngle =  -atan(-1 * _data.ax / sqrt(pow(_data.ay, 2) + pow(_data.az, 2)));

  //  Combine gyro and acc angles using a complementary filter
  angles.pitch = _alpha * angles.pitch + (1.0f - _alpha) * accPitchAngle * RAD_TO_DEG;
  angles.roll = _alpha * angles.roll + (1.0f - _alpha) * accRollAngle * RAD_TO_DEG;
  angles.pitchRadians = angles.pitch * DEG_TO_RAD;
  angles.rollRadians = angles.roll * DEG_TO_RAD;
}

void ReefwingAHRS::tiltCompensatedYaw() {
  //  Calculate yaw using magnetometer & derived roll and pitch
  float mag_x_compensated = _data.mx * cos(angles.pitchRadians) + _data.mz * sin(angles.pitchRadians);
  float mag_y_compensated = _data.mx * sin(angles.rollRadians) * sin(angles.pitchRadians) + _data.my * cos(angles.rollRadians) - _data.mz * sin(angles.rollRadians) * cos(angles.pitchRadians);

  angles.yawRadians = -atan2(mag_x_compensated, mag_y_compensated);
  angles.yaw =  angles.yawRadians * RAD_TO_DEG;    //  Yaw compensated for tilt
  //  float magYawAngle = atan2(_data.mx, _data.my) * RAD_TO_DEG;      //  Raw yaw from magnetometer, uncompensated for tilt - alternative yaw value
  
  angles.heading = angles.yaw - _declination;
}

void ReefwingAHRS::updateEulerAngles(float deltaT) {
  // Auxiliary variables to avoid repeated arithmetic
  float sinPHI = sin(angles.rollRadians);
  float cosPHI = cos(angles.rollRadians);
  float cosTHETA = cos(angles.pitchRadians);
  float tanTHETA = tan(angles.pitchRadians);

  //  Convert gyro rates to Euler rates (ground reference frame)
  //  Euler Roll Rate, ϕ ̇= p + sin(ϕ)tan(θ) × q + cos(ϕ)tan(θ) × r
  //  Euler Pitch Rate, θ ̇= cos(ϕ) × q - sin(ϕ) × r
  //  Euler Yaw Rate, ψ ̇= [sin(ϕ) × q]/cos(θ) + cos(ϕ)cos(θ) × r

  float eulerRollRate = _data.gy + sinPHI * tanTHETA * _data.gx + cosPHI * tanTHETA * _data.gz;
  float eulerPitchRate = cosPHI * _data.gx - sinPHI * _data.gz;
  float eulerYawRate = (sinPHI * _data.gx) / cosTHETA + cosPHI * cosTHETA * _data.gz;

  angles.rollRadians  += eulerRollRate * deltaT;    // Angle around the X-axis
  angles.pitchRadians += eulerPitchRate * deltaT;   // Angle around the Y-axis
  angles.yawRadians   += eulerYawRate * deltaT;     // Angle around the Z-axis    
  angles.roll = angles.rollRadians * RAD_TO_DEG;
  angles.pitch = angles.pitchRadians * RAD_TO_DEG;
  angles.yaw = angles.yawRadians * RAD_TO_DEG;
}

void ReefwingAHRS::complementaryUpdate(SensorData d, float deltaT) {
  //  Roll (Theta) and Pitch (Phi) from accelerometer
  float rollAcc = atan2(d.ay, d.az);
  float pitchAcc = atan2(-d.ax, sqrt(pow(d.ay, 2) + pow(d.az, 2)));

  // Auxiliary variables to avoid repeated arithmetic
  float _halfdT = deltaT * 0.5f;
  float _cosTheta = cos(rollAcc);
  float _cosPhi = cos(pitchAcc);
  float _sinTheta = sin(rollAcc);
  float _sinPhi = sin(pitchAcc);
  float _halfTheta = rollAcc * 0.5f;
  float _halfPhi = pitchAcc * 0.5f;
  float _cosHalfTheta = cos(_halfTheta);
  float _cosHalfPhi = cos(_halfPhi);
  float _sinHalfTheta = sin(_halfTheta);
  float _sinHalfPhi = sin(_halfPhi);

  //  Calculate Attitude Quaternion
  //  ref: https://ahrs.readthedocs.io/en/latest/filters/complementary.html
  _att[0] = _att[0] - _halfdT * d.gx * _att[1] - _halfdT * d.gy * _att[2] - _halfdT * d.gz * _att[3];
  _att[1] = _att[1] + _halfdT * d.gx * _att[0] - _halfdT * d.gy * _att[3] + _halfdT * d.gz * _att[2];
  _att[2] = _att[2] + _halfdT * d.gx * _att[3] + _halfdT * d.gy * _att[0] - _halfdT * d.gz * _att[1];
  _att[3] = _att[3] - _halfdT * d.gx * _att[2] + _halfdT * d.gy * _att[1] + _halfdT * d.gz * _att[0];

  //  Calculate Tilt Vector [bx by bz] and tilt adjusted yaw (Psi) using accelerometer data
  float bx = d.mx * _cosTheta + d.my * _sinTheta * _sinPhi + d.mz * _sinTheta * _cosPhi;
  float by = d.my * _cosPhi - d.mz * _sinPhi;
  float bz = -d.mx * _sinTheta + d.my * _cosTheta * _sinPhi + d.mz * _cosTheta * _cosPhi;

  float yaw = atan2(-by, bx);

  // More auxiliary variables to avoid repeated arithmetic
  float _halfPsi = yaw * 0.5f;
  float _cosHalfPsi = cos(_halfPsi);
  float _sinHalfPsi = sin(_halfPsi);

  //  Convert Accelerometer & Magnetometer roll, pitch & yaw to quaternion (qam)
  float qam[4];  

  qam[0] = _cosHalfPhi * _cosHalfTheta * _cosHalfPsi + _sinHalfPhi * _sinHalfTheta * _sinHalfPsi;
  qam[1] = _sinHalfPhi * _cosHalfTheta * _cosHalfPsi - _cosHalfPhi * _sinHalfTheta * _sinHalfPsi;
  qam[2] = _cosHalfPhi * _sinHalfTheta * _cosHalfPsi + _sinHalfPhi * _cosHalfTheta * _sinHalfPsi;
  qam[3] = _cosHalfPhi * _cosHalfTheta * _sinHalfPsi - _sinHalfPsi * _sinHalfTheta * _cosHalfPsi;

  //  Fuse attitude quaternion (_att) with qam using complementary filter
  _q.q0 = _alpha * _att[0] + (1 - _alpha) * qam[0];
  _q.q1 = _alpha * _att[1] + (1 - _alpha) * qam[1];
  _q.q2 = _alpha * _att[2] + (1 - _alpha) * qam[2];
  _q.q3 = _alpha * _att[3] + (1 - _alpha) * qam[3];
}

void ReefwingAHRS::madgwickUpdate(SensorData d, float deltaT) {
  float norm;
  float hx, hy, _2bx, _2bz;
  float s0, s1, s2, s3;
  float qDot0, qDot1, qDot2, qDot3;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q0mx;
  float _2q0my;
  float _2q0mz;
  float _2q1mx;
  float _4bx;
  float _4bz;
  float _2q0 = 2.0f * _q.q0;
  float _2q1 = 2.0f * _q.q1;
  float _2q2 = 2.0f * _q.q2;
  float _2q3 = 2.0f * _q.q3;
  float _2q0q2 = 2.0f * _q.q0 * _q.q2;
  float _2q2q3 = 2.0f * _q.q2 * _q.q3;
  float q0q0 = _q.q0 * _q.q0;
  float q0q1 = _q.q0 * _q.q1;
  float q0q2 = _q.q0 * _q.q2;
  float q0q3 = _q.q0 * _q.q3;
  float q1q1 = _q.q1 * _q.q1;
  float q1q2 = _q.q1 * _q.q2;
  float q1q3 = _q.q1 * _q.q3;
  float q2q2 = _q.q2 * _q.q2;
  float q2q3 = _q.q2 * _q.q3;
  float q3q3 = _q.q3 * _q.q3;

  // Normalise accelerometer measurement
  norm = sqrt(d.ax * d.ax + d.ay * d.ay + d.az * d.az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  d.ax *= norm;
  d.ay *= norm;
  d.az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(d.mx * d.mx + d.my * d.my + d.mz * d.mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  d.mx *= norm;
  d.my *= norm;
  d.mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q0mx = 2.0f * _q.q0 * d.mx;
  _2q0my = 2.0f * _q.q0 * d.my;
  _2q0mz = 2.0f * _q.q0 * d.mz;
  _2q1mx = 2.0f * _q.q1 * d.mx;

  hx = d.mx * q0q0 - _2q0my * _q.q3 + _2q0mz * _q.q2 + d.mx * q1q1 + _2q1 * d.my * _q.q2 + _2q1 * d.mz * _q.q3 - d.mx * q2q2 - d.mx * q3q3;
  hy = _2q0mx * _q.q3 + d.my * q0q0 - _2q0mz * _q.q1 + _2q1mx * _q.q2 - d.my * q1q1 + d.my * q2q2 + _2q2 * d.mz * _q.q3 - d.my * q3q3;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q0mx * _q.q2 + _2q0my * _q.q1 + d.mz * q0q0 + _2q1mx * _q.q3 - d.mz * q1q1 + _2q2 * d.my * _q.q3 - d.mz * q2q2 + d.mz * q3q3;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - d.ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - d.ay) - _2bz * _q.q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - d.mx) + (-_2bx * _q.q3 + _2bz * _q.q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - d.my) + _2bx * _q.q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - d.mz);
  s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - d.ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - d.ay) - 4.0f * _q.q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - d.az) + _2bz * _q.q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - d.mx) + (_2bx * _q.q2 + _2bz * _q.q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - d.my) + (_2bx * _q.q3 - _4bz * _q.q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - d.mz);
  s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - d.ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - d.ay) - 4.0f * _q.q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - d.az) + (-_4bx * _q.q2 - _2bz * _q.q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - d.mx) + (_2bx * _q.q1 + _2bz * _q.q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - d.my) + (_2bx * _q.q0 - _4bz * _q.q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - d.mz);
  s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - d.ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - d.ay) + (-_4bx * _q.q3 + _2bz * _q.q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - d.mx) + (-_2bx * _q.q0 + _2bz * _q.q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - d.my) + _2bx * _q.q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - d.mz);
  
  norm = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);    // normalise step magnitude
  norm = 1.0f/norm;
  s0 *= norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;

  // Compute rate of change of quaternion
  qDot0 = 0.5f * (-_q.q1 * d.gx - _q.q2 * d.gy - _q.q3 * d.gz) - _beta * s0;
  qDot1 = 0.5f * (_q.q0 * d.gx + _q.q2 * d.gz - _q.q3 * d.gy) - _beta * s1;
  qDot2 = 0.5f * (_q.q0 * d.gy - _q.q1 * d.gz + _q.q3 * d.gx) - _beta * s2;
  qDot3 = 0.5f * (_q.q0 * d.gz + _q.q1 * d.gy - _q.q2 * d.gx) - _beta * s3;

  // Integrate to yield quaternion
  _q.q0 += qDot0 * deltaT;
  _q.q1 += qDot1 * deltaT;
  _q.q2 += qDot2 * deltaT;
  _q.q3 += qDot3 * deltaT;

  norm = sqrt(_q.q0 * _q.q0 + _q.q1 * _q.q1 + _q.q2 * _q.q2 + _q.q3 * _q.q3);    // normalise quaternion
  norm = 1.0f/norm;

  _q.q0 = _q.q0 * norm;
  _q.q1 = _q.q1 * norm;
  _q.q2 = _q.q2 * norm;
  _q.q3 = _q.q3 * norm;
}

void ReefwingAHRS::mahoneyUpdate(SensorData d, float deltaT) {
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q0q0 = _q.q0 * _q.q0;
  float q0q1 = _q.q0 * _q.q1;
  float q0q2 = _q.q0 * _q.q2;
  float q0q3 = _q.q0 * _q.q3;
  float q1q1 = _q.q1 * _q.q1;
  float q1q2 = _q.q1 * _q.q2;
  float q1q3 = _q.q1 * _q.q3;
  float q2q2 = _q.q2 * _q.q2;
  float q2q3 = _q.q2 * _q.q3;
  float q3q3 = _q.q3 * _q.q3;   

  // Normalise accelerometer measurement
  norm = sqrtf(d.ax * d.ax + d.ay * d.ay + d.az * d.az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  d.ax *= norm;
  d.ay *= norm;
  d.az *= norm;

  // Normalise magnetometer measurement
  norm = sqrtf(d.mx * d.mx + d.my * d.my + d.mz * d.mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  d.mx *= norm;
  d.my *= norm;
  d.mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * d.mx * (0.5f - q2q2 - q3q3) + 2.0f * d.my * (q1q2 - q0q3) + 2.0f * d.mz * (q1q3 + q0q2);
  hy = 2.0f * d.mx * (q1q2 + q0q3) + 2.0f * d.my * (0.5f - q1q1 - q3q3) + 2.0f * d.mz * (q2q3 - q0q1);
  bx = sqrtf((hx * hx) + (hy * hy));
  bz = 2.0f * d.mx * (q1q3 - q0q2) + 2.0f * d.my * (q2q3 + q0q1) + 2.0f * d.mz * (0.5f - q1q1 - q2q2);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q1q3 - q0q2);
  vy = 2.0f * (q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
  wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
  wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);  

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (d.ay * vz - d.az * vy) + (d.my * wz - d.mz * wy);
  ey = (d.az * vx - d.ax * vz) + (d.mz * wx - d.mx * wz);
  ez = (d.ax * vy - d.ay * vx) + (d.mx * wy - d.my * wx);
  if (_Ki > 0.0f) {
    _eInt[0] += ex;      // accumulate integral error
    _eInt[1] += ey;
    _eInt[2] += ez;
  }
  else {
    _eInt[0] = 0.0f;     // prevent integral wind up
    _eInt[1] = 0.0f;
    _eInt[2] = 0.0f;
  }

  // Apply feedback terms
  d.gx = d.gx + _Kp * ex + _Ki * _eInt[0];
  d.gy = d.gy + _Kp * ey + _Ki * _eInt[1];
  d.gz = d.gz + _Kp * ez + _Ki * _eInt[2];

  // Integrate rate of change of quaternion
  pa = _q.q1;
  pb = _q.q2;
  pc = _q.q3;
  _q.q0 = _q.q0 + (-_q.q1 * d.gx - _q.q2 * d.gy - _q.q3 * d.gz) * (0.5f * deltaT);
  _q.q1 = pa + (_q.q0 * d.gx + pb * d.gz - pc * d.gy) * (0.5f * deltaT);
  _q.q2 = pb + (_q.q0 * d.gy - pa * d.gz + pc * d.gx) * (0.5f * deltaT);
  _q.q3 = pc + (_q.q0 * d.gz + pa * d.gy - pb * d.gx) * (0.5f * deltaT);

  // Normalise quaternion
  norm = sqrtf(_q.q0 * _q.q0 + _q.q1 * _q.q1 + _q.q2 * _q.q2 + _q.q3 * _q.q3);
  norm = 1.0f / norm;
  _q.q0 = _q.q0 * norm;
  _q.q1 = _q.q1 * norm;
  _q.q2 = _q.q2 * norm;
  _q.q3 = _q.q3 * norm;
}
