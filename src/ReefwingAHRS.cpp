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
           - Fusion Library is based on the revised AHRS algorithm 
             presented in chapter 7 of Madgwick's PhD thesis.
             ref: https://github.com/xioTechnologies/Fusion

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
  _boardTypeStr[0] = "Nano 33 BLE";
  _boardTypeStr[1] = "Nano 33 BLE Sense";
  _boardTypeStr[2] = "Nano 33 BLE Sense Rev 2";
  _boardTypeStr[3] = "Seeed XIAO nRF52840 Sense";
  _boardTypeStr[4] = "Unsupported Board Type";
}

void ReefwingAHRS::begin() {
  //  Detect Board Hardware
  _boardType = BoardType::NOT_SUPPORTED;

  #if defined(ARDUINO_ARDUINO_NANO33BLE)    //  Nano 33 BLE found
  
    byte error;
    
    Wire1.begin();
    Wire1.beginTransmission(HTS221_ADDRESS);
    error = Wire1.endTransmission();

    if (error == 0) {
      _boardType = BoardType::NANO33BLE_SENSE_R1;
    }
    else {
      Wire1.beginTransmission(HS3003_ADDRESS);
      error = Wire1.endTransmission();

      if (error == 0) {
        _boardType = BoardType::NANO33BLE_SENSE_R2;
      }
      else {
        _boardType = BoardType::NANO33BLE;
      }
    }
  #else
    if (strncmp(BOARD_NAME, _boardTypeStr[3], 25) == 0) {
      _boardType = BoardType::XIAO_SENSE;
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
}

void ReefwingAHRS::update() {
  long now = micros();
  float deltaT = ((now - _lastUpdate)/1000000.0f); // Time elapsed since last update

  _lastUpdate = now;

  switch(_fusion) {
    case SensorFusion::MADGWICK:
      madgwickUpdate(deltaT);
    break;
    case SensorFusion::MAHONY:
      mahoneyUpdate(deltaT);
    break;
    case SensorFusion::COMPLEMENTARY:
      complementaryUpdate(deltaT);
    break;
    case SensorFusion::FUSION:
      //  TODO:
    break;
    case SensorFusion::CLASSIC:
      updateEulerAngles(deltaT);
      classicUpdate();
      tiltCompensatedYaw();
    break;
    case SensorFusion::NONE:
      updateEulerAngles(deltaT);
      tiltCompensatedYaw();
    break;
  }
}

SensorData ReefwingAHRS::filterFormat() {
  //  Correct LSM9DS1 magnetometer x-axis mismatch
  //  and convert gyro to radians/sec.
  SensorData filterData;

  filterData.ax = _data.ax;
  filterData.ay = _data.ay;
  filterData.az = _data.az;

  filterData.gx = _data.gx * DEG_TO_RAD;
  filterData.gy = _data.gy * DEG_TO_RAD;
  filterData.gz = _data.gz * DEG_TO_RAD;

  filterData.mx = -_data.mx;
  filterData.my = _data.my;
  filterData.mz = _data.mz;

  return filterData;
}

const char* ReefwingAHRS::getBoardType() {
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

void ReefwingAHRS::setFusionGain(float g) {
  //TODO:
}

void ReefwingAHRS::setDeclination(float dec) {
  _declination = dec;
}

void ReefwingAHRS::setData(SensorData d) {
  _data = d;
}

/******************************************************************
 *
 * ReefwingAHRS - Update Methods 
 * 
 ******************************************************************/

void ReefwingAHRS::classicUpdate() {
  // Convert from force vector to angle using 3 axis formula - result in radians
  float accRollAngle  =  atan(-1 * _data.ay / sqrt(pow(_data.ax, 2) + pow(_data.az, 2)));
  float accPitchAngle =  -atan(-1 * _data.ax / sqrt(pow(_data.ay, 2) + pow(_data.az, 2)));

  //  Combine gyro and acc angles using a complementary filter
  _angles.pitch = _alpha * _angles.pitch + (1.0f - _alpha) * accPitchAngle * RAD_TO_DEG;
  _angles.roll = _alpha * _angles.roll + (1.0f - _alpha) * accRollAngle * RAD_TO_DEG;
  _angles.pitchRadians = _angles.pitch * DEG_TO_RAD;
  _angles.rollRadians = _angles.roll * DEG_TO_RAD;
}

void ReefwingAHRS::tiltCompensatedYaw() {
  //  Calculate yaw using magnetometer & derived roll and pitch
  float mag_x_compensated = _data.mx * cos(_angles.pitchRadians) + _data.mz * sin(_angles.pitchRadians);
  float mag_y_compensated = _data.mx * sin(_angles.rollRadians) * sin(_angles.pitchRadians) + _data.my * cos(_angles.rollRadians) - _data.mz * sin(_angles.rollRadians) * cos(_angles.pitchRadians);

  _angles.yawRadians = -atan2(mag_x_compensated, mag_y_compensated);
  _angles.yaw =  _angles.yawRadians * RAD_TO_DEG;    //  Yaw compensated for tilt
  //  float magYawAngle = atan2(_data.mx, _data.my) * RAD_TO_DEG;      //  Raw yaw from magnetometer, uncompensated for tilt - alternative yaw value
  
  _angles.heading = _angles.yaw - _declination;
}

void ReefwingAHRS::updateEulerAngles(float deltaT) {
  // Auxiliary variables to avoid repeated arithmetic
  float sinPHI = sin(_angles.rollRadians);
  float cosPHI = cos(_angles.rollRadians);
  float cosTHETA = cos(_angles.pitchRadians);
  float tanTHETA = tan(_angles.pitchRadians);

  //  Convert gyro rates to Euler rates (ground reference frame)
  //  Euler Roll Rate, ϕ ̇= p + sin(ϕ)tan(θ) × q + cos(ϕ)tan(θ) × r
  //  Euler Pitch Rate, θ ̇= cos(ϕ) × q - sin(ϕ) × r
  //  Euler Yaw Rate, ψ ̇= [sin(ϕ) × q]/cos(θ) + cos(ϕ)cos(θ) × r

  float eulerRollRate = _data.gy + sinPHI * tanTHETA * _data.gx + cosPHI * tanTHETA * _data.gz;
  float eulerPitchRate = cosPHI * _data.gx - sinPHI * _data.gz;
  float eulerYawRate = (sinPHI * _data.gx) / cosTHETA + cosPHI * cosTHETA * _data.gz;

  _angles.rollRadians  += eulerRollRate * deltaT;    // Angle around the X-axis
  _angles.pitchRadians += eulerPitchRate * deltaT;   // Angle around the Y-axis
  _angles.yawRadians   += eulerYawRate * deltaT;     // Angle around the Z-axis    
  _angles.roll = _angles.rollRadians * RAD_TO_DEG;
  _angles.pitch = _angles.pitchRadians * RAD_TO_DEG;
  _angles.yaw = _angles.yawRadians * RAD_TO_DEG;
}

void ReefwingAHRS::complementaryUpdate(float deltaT) {
  //  Roll (Theta) and Pitch (Phi) from accelerometer
  float rollAcc = atan2(_data.ay, _data.az);
  float pitchAcc = atan2(-_data.ax, sqrt(pow(_data.ay, 2) + pow(_data.az, 2)));

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
  _att[0] = _att[0] - _halfdT * _data.gx * _att[1] - _halfdT * _data.gy * _att[2] - _halfdT * _data.gz * _att[3];
  _att[1] = _att[1] + _halfdT * _data.gx * _att[0] - _halfdT * _data.gy * _att[3] + _halfdT * _data.gz * _att[2];
  _att[2] = _att[2] + _halfdT * _data.gx * _att[3] + _halfdT * _data.gy * _att[0] - _halfdT * _data.gz * _att[1];
  _att[3] = _att[3] - _halfdT * _data.gx * _att[2] + _halfdT * _data.gy * _att[1] + _halfdT * _data.gz * _att[0];

  //  Calculate Tilt Vector [bx by bz] and tilt adjusted yaw (Psi) using accelerometer data
  float bx = _data.mx * _cosTheta + _data.my * _sinTheta * _sinPhi + _data.mz * _sinTheta * _cosPhi;
  float by = _data.my * _cosPhi - _data.mz * _sinPhi;
  float bz = -_data.mx * _sinTheta + _data.my * _cosTheta * _sinPhi + _data.mz * _cosTheta * _cosPhi;

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

void ReefwingAHRS::madgwickUpdate(float deltaT) {
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
  norm = sqrt(_data.ax * _data.ax + _data.ay * _data.ay + _data.az * _data.az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  _data.ax *= norm;
  _data.ay *= norm;
  _data.az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(_data.mx * _data.mx + _data.my * _data.my + _data.mz * _data.mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  _data.mx *= norm;
  _data.my *= norm;
  _data.mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q0mx = 2.0f * _q.q0 * _data.mx;
  _2q0my = 2.0f * _q.q0 * _data.my;
  _2q0mz = 2.0f * _q.q0 * _data.mz;
  _2q1mx = 2.0f * _q.q1 * _data.mx;

  hx = _data.mx * q0q0 - _2q0my * _q.q3 + _2q0mz * _q.q2 + _data.mx * q1q1 + _2q1 * _data.my * _q.q2 + _2q1 * _data.mz * _q.q3 - _data.mx * q2q2 - _data.mx * q3q3;
  hy = _2q0mx * _q.q3 + _data.my * q0q0 - _2q0mz * _q.q1 + _2q1mx * _q.q2 - _data.my * q1q1 + _data.my * q2q2 + _2q2 * _data.mz * _q.q3 - _data.my * q3q3;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q0mx * _q.q2 + _2q0my * _q.q1 + _data.mz * q0q0 + _2q1mx * _q.q3 - _data.mz * q1q1 + _2q2 * _data.my * _q.q3 - _data.mz * q2q2 + _data.mz * q3q3;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - _data.ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - _data.ay) - _2bz * _q.q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - _data.mx) + (-_2bx * _q.q3 + _2bz * _q.q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - _data.my) + _2bx * _q.q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - _data.mz);
  s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - _data.ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - _data.ay) - 4.0f * _q.q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - _data.az) + _2bz * _q.q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - _data.mx) + (_2bx * _q.q2 + _2bz * _q.q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - _data.my) + (_2bx * _q.q3 - _4bz * _q.q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - _data.mz);
  s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - _data.ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - _data.ay) - 4.0f * _q.q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - _data.az) + (-_4bx * _q.q2 - _2bz * _q.q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - _data.mx) + (_2bx * _q.q1 + _2bz * _q.q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - _data.my) + (_2bx * _q.q0 - _4bz * _q.q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - _data.mz);
  s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - _data.ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - _data.ay) + (-_4bx * _q.q3 + _2bz * _q.q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - _data.mx) + (-_2bx * _q.q0 + _2bz * _q.q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - _data.my) + _2bx * _q.q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - _data.mz);
  
  norm = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);    // normalise step magnitude
  norm = 1.0f/norm;
  s0 *= norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;

  // Compute rate of change of quaternion
  qDot0 = 0.5f * (-_q.q1 * _data.gx - _q.q2 * _data.gy - _q.q3 * _data.gz) - _beta * s0;
  qDot1 = 0.5f * (_q.q0 * _data.gx + _q.q2 * _data.gz - _q.q3 * _data.gy) - _beta * s1;
  qDot2 = 0.5f * (_q.q0 * _data.gy - _q.q1 * _data.gz + _q.q3 * _data.gx) - _beta * s2;
  qDot3 = 0.5f * (_q.q0 * _data.gz + _q.q1 * _data.gy - _q.q2 * _data.gx) - _beta * s3;

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

void ReefwingAHRS::mahoneyUpdate(float deltaT) {
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
  norm = sqrtf(_data.ax * _data.ax + _data.ay * _data.ay + _data.az * _data.az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  _data.ax *= norm;
  _data.ay *= norm;
  _data.az *= norm;

  // Normalise magnetometer measurement
  norm = sqrtf(_data.mx * _data.mx + _data.my * _data.my + _data.mz * _data.mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  _data.mx *= norm;
  _data.my *= norm;
  _data.mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * _data.mx * (0.5f - q2q2 - q3q3) + 2.0f * _data.my * (q1q2 - q0q3) + 2.0f * _data.mz * (q1q3 + q0q2);
  hy = 2.0f * _data.mx * (q1q2 + q0q3) + 2.0f * _data.my * (0.5f - q1q1 - q3q3) + 2.0f * _data.mz * (q2q3 - q0q1);
  bx = sqrtf((hx * hx) + (hy * hy));
  bz = 2.0f * _data.mx * (q1q3 - q0q2) + 2.0f * _data.my * (q2q3 + q0q1) + 2.0f * _data.mz * (0.5f - q1q1 - q2q2);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q1q3 - q0q2);
  vy = 2.0f * (q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
  wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
  wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);  

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (_data.ay * vz - _data.az * vy) + (_data.my * wz - _data.mz * wy);
  ey = (_data.az * vx - _data.ax * vz) + (_data.mz * wx - _data.mx * wz);
  ez = (_data.ax * vy - _data.ay * vx) + (_data.mx * wy - _data.my * wx);
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
  _data.gx = _data.gx + _Kp * ex + _Ki * _eInt[0];
  _data.gy = _data.gy + _Kp * ey + _Ki * _eInt[1];
  _data.gz = _data.gz + _Kp * ez + _Ki * _eInt[2];

  // Integrate rate of change of quaternion
  pa = _q.q1;
  pb = _q.q2;
  pc = _q.q3;
  _q.q0 = _q.q0 + (-_q.q1 * _data.gx - _q.q2 * _data.gy - _q.q3 * _data.gz) * (0.5f * deltaT);
  _q.q1 = pa + (_q.q0 * _data.gx + pb * _data.gz - pc * _data.gy) * (0.5f * deltaT);
  _q.q2 = pb + (_q.q0 * _data.gy - pa * _data.gz + pc * _data.gx) * (0.5f * deltaT);
  _q.q3 = pc + (_q.q0 * _data.gz + pa * _data.gy - pb * _data.gx) * (0.5f * deltaT);

  // Normalise quaternion
  norm = sqrtf(_q.q0 * _q.q0 + _q.q1 * _q.q1 + _q.q2 * _q.q2 + _q.q3 * _q.q3);
  norm = 1.0f / norm;
  _q.q0 = _q.q0 * norm;
  _q.q1 = _q.q1 * norm;
  _q.q2 = _q.q2 * norm;
  _q.q3 = _q.q3 * norm;
}
