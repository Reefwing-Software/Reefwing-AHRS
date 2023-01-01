/******************************************************************
  @file       ReefwingAHRS.cpp
  @brief      Attitude and Heading Reference System (AHRS) for the Nano 33 BLE.
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

  Credit - LPS22HB Absolute Digital Barometer class 
           based on work by Adrien Chapelet for IoThings.
           ref: https://github.com/adrien3d/IO_LPS22HB
         - LSM9DS1 Class borrows heavily from the 
           Kris Winer sketch LSM9DS1_BasicAHRS_Nano33.ino
           ref: https://github.com/kriswiner/LSM9DS1
         - The C++ code for our quaternion position update 
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
 * LSM9DS1 Register Map
 * Reference: https://www.st.com/resource/en/datasheet/lsm9ds1.pdf
 *            Section 6, Page 38, Table 21.
 * 
 * Accelerometer and Gyroscope Registers
 * 
 ******************************************************************/

#define LSM9DS1XG_ACT_THS           0x04
#define LSM9DS1XG_ACT_DUR           0x05
#define LSM9DS1XG_INT_GEN_CFG_XL    0x06
#define LSM9DS1XG_INT_GEN_THS_X_XL  0x07
#define LSM9DS1XG_INT_GEN_THS_Y_XL  0x08
#define LSM9DS1XG_INT_GEN_THS_Z_XL  0x09
#define LSM9DS1XG_INT_GEN_DUR_XL    0x0A
#define LSM9DS1XG_REFERENCE_G       0x0B
#define LSM9DS1XG_INT1_CTRL         0x0C
#define LSM9DS1XG_INT2_CTRL         0x0D
#define LSM9DS1XG_WHO_AM_I          0x0F  // Returns 0x68 ref: 7.11
#define LSM9DS1XG_CTRL_REG1_G       0x10
#define LSM9DS1XG_CTRL_REG2_G       0x11
#define LSM9DS1XG_CTRL_REG3_G       0x12
#define LSM9DS1XG_ORIENT_CFG_G      0x13
#define LSM9DS1XG_INT_GEN_SRC_G     0x14
#define LSM9DS1XG_OUT_TEMP_L        0x15
#define LSM9DS1XG_OUT_TEMP_H        0x16
#define LSM9DS1XG_STATUS_REG        0x17
#define LSM9DS1XG_OUT_X_L_G         0x18
#define LSM9DS1XG_OUT_X_H_G         0x19
#define LSM9DS1XG_OUT_Y_L_G         0x1A
#define LSM9DS1XG_OUT_Y_H_G         0x1B
#define LSM9DS1XG_OUT_Z_L_G         0x1C
#define LSM9DS1XG_OUT_Z_H_G         0x1D
#define LSM9DS1XG_CTRL_REG4         0x1E
#define LSM9DS1XG_CTRL_REG5_XL      0x1F
#define LSM9DS1XG_CTRL_REG6_XL      0x20
#define LSM9DS1XG_CTRL_REG7_XL      0x21
#define LSM9DS1XG_CTRL_REG8         0x22
#define LSM9DS1XG_CTRL_REG9         0x23
#define LSM9DS1XG_CTRL_REG10        0x24
#define LSM9DS1XG_INT_GEN_SRC_XL    0x26
#define LSM9DS1XG_OUT_X_L_XL        0x28
#define LSM9DS1XG_OUT_X_H_XL        0x29
#define LSM9DS1XG_OUT_Y_L_XL        0x2A
#define LSM9DS1XG_OUT_Y_H_XL        0x2B
#define LSM9DS1XG_OUT_Z_L_XL        0x2C
#define LSM9DS1XG_OUT_Z_H_XL        0x2D
#define LSM9DS1XG_FIFO_CTRL         0x2E
#define LSM9DS1XG_FIFO_SRC          0x2F
#define LSM9DS1XG_INT_GEN_CFG_G     0x30
#define LSM9DS1XG_INT_GEN_THS_XH_G  0x31
#define LSM9DS1XG_INT_GEN_THS_XL_G  0x32
#define LSM9DS1XG_INT_GEN_THS_YH_G  0x33
#define LSM9DS1XG_INT_GEN_THS_YL_G  0x34
#define LSM9DS1XG_INT_GEN_THS_ZH_G  0x35
#define LSM9DS1XG_INT_GEN_THS_ZL_G  0x36
#define LSM9DS1XG_INT_GEN_DUR_G     0x37
#define LSM9DS1XG_WHO_AM_I_VALUE    0x68

/******************************************************************
 *
 * Magnetometer Registers 
 * 
 ******************************************************************/

#define LSM9DS1M_OFFSET_X_REG_L_M   0x05
#define LSM9DS1M_OFFSET_X_REG_H_M   0x06
#define LSM9DS1M_OFFSET_Y_REG_L_M   0x07
#define LSM9DS1M_OFFSET_Y_REG_H_M   0x08
#define LSM9DS1M_OFFSET_Z_REG_L_M   0x09
#define LSM9DS1M_OFFSET_Z_REG_H_M   0x0A
#define LSM9DS1M_WHO_AM_I           0x0F  // Returns 0x3D ref: 8.4
#define LSM9DS1M_CTRL_REG1_M        0x20
#define LSM9DS1M_CTRL_REG2_M        0x21
#define LSM9DS1M_CTRL_REG3_M        0x22
#define LSM9DS1M_CTRL_REG4_M        0x23
#define LSM9DS1M_CTRL_REG5_M        0x24
#define LSM9DS1M_STATUS_REG_M       0x27
#define LSM9DS1M_OUT_X_L_M          0x28
#define LSM9DS1M_OUT_X_H_M          0x29
#define LSM9DS1M_OUT_Y_L_M          0x2A
#define LSM9DS1M_OUT_Y_H_M          0x2B
#define LSM9DS1M_OUT_Z_L_M          0x2C
#define LSM9DS1M_OUT_Z_H_M          0x2D
#define LSM9DS1M_INT_CFG_M          0x30
#define LSM9DS1M_INT_SRC_M          0x31
#define LSM9DS1M_INT_THS_L_M        0x32
#define LSM9DS1M_INT_THS_H_M        0x33
#define LSM9DS1M_WHO_AM_I_VALUE     0x3D

/******************************************************************
 *
 * Barometer Registers -
 * 
 ******************************************************************/

#define LPS22HB_WHO_AM_I        0X0F // Who am I
#define LPS22HB_RES_CONF        0X1A // Normal (0) or Low current mode (1)
#define LPS22HB_CTRL_REG1       0X10 // Output rate and filter settings
#define LPS22HB_CTRL_REG2       0X11 // BOOT FIFO_EN STOP_ON_FTH IF_ADD_INC I2C_DIS SWRESET One_Shot
#define LPS22HB_STATUS_REG      0X27 // Temp or Press data available bits
#define LPS22HB_PRES_OUT_XL     0X28 // XLSB
#define LPS22HB_PRES_OUT_L      0X29 // LSB
#define LPS22HB_PRES_OUT_H      0X2A // MSB
#define LPS22HB_TEMP_OUT_L      0X2B // LSB
#define LPS22HB_TEMP_OUT_H      0X2C // MSB
#define LPS22HB_WHO_AM_I_VALUE  0xB1 // Expected return value of WHO_AM_I register

/******************************************************************
 *
 * I2C Device Addresses - 
 * ref: https://github.com/arduino-libraries/Arduino_LSM9DS1/blob/master/src/LSM9DS1.cpp
 * 
 ******************************************************************/

#define LSM9DS1XG_ADDRESS 0x6B  //  Address of gyroscope
#define LSM9DS1M_ADDRESS  0x1E  //  Address of magnetometer
#define LPS22HB_ADDRESS   0x5C  //  Address of barometer (on SENSE)  

/******************************************************************
 *
 * Define pressure and temperature conversion rates - 
 * 
 ******************************************************************/

#define ADC_256  0x00 
#define ADC_512  0x02
#define ADC_1024 0x04
#define ADC_2048 0x06
#define ADC_4096 0x08
#define ADC_D1   0x40
#define ADC_D2   0x50

/******************************************************************
 *
 * ENUM Type Definitions - 
 * 
 ******************************************************************/

enum Aodr {  // set of allowable gyro sample rates
  AODR_PowerDown = 0,
  AODR_10Hz,
  AODR_50Hz,
  AODR_119Hz,
  AODR_238Hz,
  AODR_476Hz,
  AODR_952Hz
};

enum Abw {  // set of allowable accewl bandwidths
  ABW_408Hz = 0,
  ABW_211Hz,
  ABW_105Hz,
  ABW_50Hz
};

enum Godr {  // set of allowable gyro sample rates
  GODR_PowerDown = 0,
  GODR_14_9Hz,
  GODR_59_5Hz,
  GODR_119Hz,
  GODR_238Hz,
  GODR_476Hz,
  GODR_952Hz
};

enum Gbw {   // set of allowable gyro data bandwidths
  GBW_low = 0,  // 14 Hz at Godr = 238 Hz,  33 Hz at Godr = 952 Hz
  GBW_med,      // 29 Hz at Godr = 238 Hz,  40 Hz at Godr = 952 Hz
  GBW_high,     // 63 Hz at Godr = 238 Hz,  58 Hz at Godr = 952 Hz
  GBW_highest   // 78 Hz at Godr = 238 Hz, 100 Hz at Godr = 952 Hz
};

enum Mmode {
  MMode_LowPower = 0, 
  MMode_MedPerformance,
  MMode_HighPerformance,
  MMode_UltraHighPerformance
};

enum Modr {  // set of allowable mag sample rates
  MODR_0_625Hz = 0,
  MODR_1_25Hz,
  MODR_2_5Hz,
  MODR_5Hz,
  MODR_10Hz,
  MODR_20Hz,
  MODR_80Hz
};

/******************************************************************
 *
 * Quarternion Implementation - 
 * 
 ******************************************************************/

Quaternion::Quaternion() {
  reset();
}

Quaternion::Quaternion(float w, float x, float y, float z) {
  q0 = w;
  q1 = x;
  q2 = y;
  q3 = z;
}

Quaternion::Quaternion(float yaw, float pitch, float roll) {
  //  Converts Euler Angles,  yaw (Z), pitch (Y), and roll (X) in radians
  //  to a quaternion.
  //  ref: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);

  q0 = cr * cp * cy + sr * sp * sy;
  q1 = sr * cp * cy - cr * sp * sy;
  q2 = cr * sp * cy + sr * cp * sy;
  q3 = cr * cp * sy - sr * sp * cy;
}

void Quaternion::reset() {
  q0 = 1.0;
  q1 = q2 = q3 = 0.0;
}

EulerAngles Quaternion::toEulerAngles(float declination) {
  //  Converts a quaternion to Euler Angles
  //  ref: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

  EulerAngles angles;

  angles.yawRadians   = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);   
  angles.pitchRadians = -asin(2.0f * (q1 * q3 - q0 * q2));
  angles.rollRadians  = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);

  angles.pitch = angles.pitchRadians * 180.0f / M_PI;
  angles.yaw   = angles.yawRadians * 180.0f / M_PI; 
  angles.roll  = angles.rollRadians * 180.0f / M_PI;

  // Convert yaw to heading (normal compass degrees)   
  if (angles.yaw < 0) angles.yaw = angles.yaw + 360.0;
  if (angles.yaw >= 360.0) angles.yaw = angles.yaw - 360.0;

  angles.heading = angles.yaw - declination; // You need to subtract a positive declination.

  return angles;
}

void Quaternion::complementaryUpdate(SensorData data, float alpha, float deltaT) {
  //  Roll (Theta) and Pitch (Phi) from accelerometer
  float rollAcc = atan2(data.ay, data.az);
  float pitchAcc = atan2(-data.ax, sqrt(pow(data.ay, 2) + pow(data.az, 2)));

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
  att[0] = att[0] - _halfdT * data.gx * att[1] - _halfdT * data.gy * att[2] - _halfdT * data.gz * att[3];
  att[1] = att[1] + _halfdT * data.gx * att[0] - _halfdT * data.gy * att[3] + _halfdT * data.gz * att[2];
  att[2] = att[2] + _halfdT * data.gx * att[3] + _halfdT * data.gy * att[0] - _halfdT * data.gz * att[1];
  att[3] = att[3] - _halfdT * data.gx * att[2] + _halfdT * data.gy * att[1] + _halfdT * data.gz * att[0];

  //  Calculate Tilt Vector [bx by bz] and tilt adjusted yaw (Psi) using accelerometer data
  float bx = data.mx * _cosTheta + data.my * _sinTheta * _sinPhi + data.mz * _sinTheta * _cosPhi;
  float by = data.my * _cosPhi - data.mz * _sinPhi;
  float bz = -data.mx * _sinTheta + data.my * _cosTheta * _sinPhi + data.mz * _cosTheta * _cosPhi;

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

  //  Fuse attitude quaternion (att) with qam using complementary filter
  q0 = alpha * att[0] + (1 - alpha) * qam[0];
  q1 = alpha * att[1] + (1 - alpha) * qam[1];
  q2 = alpha * att[2] + (1 - alpha) * qam[2];
  q3 = alpha * att[3] + (1 - alpha) * qam[3];
}

void Quaternion::madgwickUpdate(SensorData data, float beta, float deltaT) {
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
  float _2q0 = 2.0f * q0;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q0q2 = 2.0f * q0 * q2;
  float _2q2q3 = 2.0f * q2 * q3;
  float q0q0 = q0 * q0;
  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  float q0q3 = q0 * q3;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q3q3 = q3 * q3;

  // Normalise accelerometer measurement
  norm = sqrt(data.ax * data.ax + data.ay * data.ay + data.az * data.az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  data.ax *= norm;
  data.ay *= norm;
  data.az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(data.mx * data.mx + data.my * data.my + data.mz * data.mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  data.mx *= norm;
  data.my *= norm;
  data.mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q0mx = 2.0f * q0 * data.mx;
  _2q0my = 2.0f * q0 * data.my;
  _2q0mz = 2.0f * q0 * data.mz;
  _2q1mx = 2.0f * q1 * data.mx;

  hx = data.mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + data.mx * q1q1 + _2q1 * data.my * q2 + _2q1 * data.mz * q3 - data.mx * q2q2 - data.mx * q3q3;
  hy = _2q0mx * q3 + data.my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - data.my * q1q1 + data.my * q2q2 + _2q2 * data.mz * q3 - data.my * q3q3;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q0mx * q2 + _2q0my * q1 + data.mz * q0q0 + _2q1mx * q3 - data.mz * q1q1 + _2q2 * data.my * q3 - data.mz * q2q2 + data.mz * q3q3;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - data.ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - data.ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - data.mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - data.my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - data.mz);
  s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - data.ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - data.ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - data.az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - data.mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - data.my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - data.mz);
  s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - data.ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - data.ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - data.az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - data.mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - data.my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - data.mz);
  s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - data.ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - data.ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - data.mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - data.my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - data.mz);
  
  norm = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);    // normalise step magnitude
  norm = 1.0f/norm;
  s0 *= norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;

  // Compute rate of change of quaternion
  qDot0 = 0.5f * (-q1 * data.gx - q2 * data.gy - q3 * data.gz) - beta * s0;
  qDot1 = 0.5f * (q0 * data.gx + q2 * data.gz - q3 * data.gy) - beta * s1;
  qDot2 = 0.5f * (q0 * data.gy - q1 * data.gz + q3 * data.gx) - beta * s2;
  qDot3 = 0.5f * (q0 * data.gz + q1 * data.gy - q2 * data.gx) - beta * s3;

  // Integrate to yield quaternion
  q0 += qDot0 * deltaT;
  q1 += qDot1 * deltaT;
  q2 += qDot2 * deltaT;
  q3 += qDot3 * deltaT;

  norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);    // normalise quaternion
  norm = 1.0f/norm;

  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;
}

void Quaternion::mahoneyUpdate(SensorData data, float Kp, float Ki, float deltaT) {
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q0q0 = q0 * q0;
  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  float q0q3 = q0 * q3;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q3q3 = q3 * q3;   

  // Normalise accelerometer measurement
  norm = sqrtf(data.ax * data.ax + data.ay * data.ay + data.az * data.az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  data.ax *= norm;
  data.ay *= norm;
  data.az *= norm;

  // Normalise magnetometer measurement
  norm = sqrtf(data.mx * data.mx + data.my * data.my + data.mz * data.mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  data.mx *= norm;
  data.my *= norm;
  data.mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * data.mx * (0.5f - q2q2 - q3q3) + 2.0f * data.my * (q1q2 - q0q3) + 2.0f * data.mz * (q1q3 + q0q2);
  hy = 2.0f * data.mx * (q1q2 + q0q3) + 2.0f * data.my * (0.5f - q1q1 - q3q3) + 2.0f * data.mz * (q2q3 - q0q1);
  bx = sqrtf((hx * hx) + (hy * hy));
  bz = 2.0f * data.mx * (q1q3 - q0q2) + 2.0f * data.my * (q2q3 + q0q1) + 2.0f * data.mz * (0.5f - q1q1 - q2q2);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q1q3 - q0q2);
  vy = 2.0f * (q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
  wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
  wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);  

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (data.ay * vz - data.az * vy) + (data.my * wz - data.mz * wy);
  ey = (data.az * vx - data.ax * vz) + (data.mz * wx - data.mx * wz);
  ez = (data.ax * vy - data.ay * vx) + (data.mx * wy - data.my * wx);
  if (Ki > 0.0f) {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else {
    eInt[0] = 0.0f;     // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }

  // Apply feedback terms
  data.gx = data.gx + Kp * ex + Ki * eInt[0];
  data.gy = data.gy + Kp * ey + Ki * eInt[1];
  data.gz = data.gz + Kp * ez + Ki * eInt[2];

  // Integrate rate of change of quaternion
  pa = q1;
  pb = q2;
  pc = q3;
  q0 = q0 + (-q1 * data.gx - q2 * data.gy - q3 * data.gz) * (0.5f * deltaT);
  q1 = pa + (q0 * data.gx + pb * data.gz - pc * data.gy) * (0.5f * deltaT);
  q2 = pb + (q0 * data.gy - pa * data.gz + pc * data.gx) * (0.5f * deltaT);
  q3 = pc + (q0 * data.gz + pa * data.gy - pb * data.gx) * (0.5f * deltaT);

  // Normalise quaternion
  norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  norm = 1.0f / norm;
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;
}

/******************************************************************
 *
 * LSM9DS1 Implementation - 
 * 
 ******************************************************************/

LSM9DS1::LSM9DS1() { }

void LSM9DS1::begin() {
  Wire1.begin();
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x05);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, 0x0c);

  //  Default configuration
  OSR = ADC_4096;                 // set pressure amd temperature oversample rate
  Godr = GODR_238Hz;              // gyro data sample rate
  Gbw = GBW_med;                  // gyro data bandwidth
  Aodr = AODR_238Hz;              // accel data sample rate
  Abw = ABW_50Hz;                 // accel data bandwidth
  Modr = MODR_10Hz;               // mag data sample rate
  Mmode = MMode_HighPerformance;  // magnetometer operation mode

  //  Sensor Fusion Co-Efficients - see README.md
  gyroMeasError = M_PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
  alpha = 0.98; // default complementary filter coefficient
  beta = sqrt(3.0f / 4.0f) * gyroMeasError;   // compute beta
  Kp = 2.0f * 5.0f; // These are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
  Ki = 0.0f;

  //  Scale resolutions per LSB for each sensor
  //  sets aRes, gRes, mRes and aScale, gScale, mScale
  setAccResolution(Ascale::AFS_2G);
  setGyroResolution(Gscale::GFS_245DPS);
  setMagResolution(Mscale::MFS_4G);

  //  Set default sensor fusion algorithm
  setFusionAlgorithm(SensorFusion::MADGWICK);

  //  Set default magnetic declination - Sydney, NSW, AUSTRALIA
  setDeclination(12.717);
}

void LSM9DS1::start() {  
  //  Start processing sensor data
  //  enable the 3-axes of the gyroscope
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG4, 0x38);
  // configure the gyroscope
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG1_G, Godr << 5 | gScale << 3 | Gbw);
  delay(200);
  // enable the three axes of the accelerometer 
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG5_XL, 0x38);
  // configure the accelerometer-specify bandwidth selection with Abw
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG6_XL, Aodr << 5 | aScale << 3 | 0x04 | Abw);
  delay(200);
  // enable block data update, allow auto-increment during multiple byte read
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x44);
  // configure the magnetometer-enable temperature compensation of mag data
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M, 0x80 | Mmode << 5 | Modr << 2); // select x,y-axis mode
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, mScale << 5 ); // select mag full scale
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG3_M, 0x00 ); // continuous conversion mode
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG4_M, Mmode << 2 ); // select z-axis mode
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG5_M, 0x40 ); // select block update mode

  // Initialise AHRS algorithm
  FusionOffsetInitialise(&fusionOffset, sampleRate);
  FusionAhrsInitialise(&fusionAhrs); 
  FusionAhrsSetSettings(&fusionAhrs, &fusionSettings);
}

bool LSM9DS1::accelAvailable() {
  if (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_STATUS_REG) & 0x01) return true;

  return false;
}

bool LSM9DS1::gyroAvailable() {
  if (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_STATUS_REG) & 0x02) return true;

  return false;
}

bool LSM9DS1::magAvailable() {
  if (readByte(LSM9DS1M_ADDRESS, LSM9DS1M_STATUS_REG_M) & 0x08) return true;

  return false;
}

EulerAngles LSM9DS1::update() {
  int16_t accelCount[3], gyroCount[3], magCount[3];  // Stores the raw 16-bit signed accelerometer, gyro, and mag sensor output

  if (accelAvailable()) {  // check if new accel data is ready  
    readAccelData(accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    sensorData.ax = (float)accelCount[0] * aRes - accelBias[0];  // get actual g value, this depends on scale being set
    sensorData.ay = (float)accelCount[1] * aRes - accelBias[1];   
    sensorData.az = (float)accelCount[2] * aRes - accelBias[2]; 
  } 

  if (gyroAvailable()) {  // check if new gyro data is ready  
    readGyroData(gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    sensorData.gx = (float)gyroCount[0] * gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
    sensorData.gy = (float)gyroCount[1] * gRes - gyroBias[1];  
    sensorData.gz = (float)gyroCount[2] * gRes - gyroBias[2];   
  }

  if (magAvailable()) {  // check if new mag data is ready  
    readMagData(magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    sensorData.mx = (float)magCount[0] * mRes - magBias[0];  // get actual magnetometer value, this depends on scale being set
    sensorData.my = (float)magCount[1] * mRes - magBias[1];  // if offset already stored in register, no need
    sensorData.mz = (float)magCount[2] * mRes - magBias[2];  // to subtract the bias again. 
  }

  long now = micros();

  deltaT = ((now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = now;

  //  Sensor Fusion - updates quaternion & Euler Angles
  switch (fusion) {
    case SensorFusion::MADGWICK:
      quaternion.madgwickUpdate(filterFormat(), beta, deltaT);
      break;
    case SensorFusion::MAHONY:
      quaternion.mahoneyUpdate(filterFormat(), Kp, Ki, deltaT);
      break;
    case SensorFusion::COMPLEMENTARY:
      quaternion.complementaryUpdate(filterFormat(), alpha, deltaT);
      break;
    case SensorFusion::FUSION:
      return fusionEulerAngles(sensorData);
      break;
    case SensorFusion::CLASSIC:
      updateEulerAngles();
      complementaryUpdate();
      tiltCompensatedYaw();
      return eulerAngles;
      break;
    case SensorFusion::NONE:
      updateEulerAngles();
      tiltCompensatedYaw();
      return eulerAngles;
      break;
  }

  eulerAngles = quaternion.toEulerAngles(declination);
  eulerAngles = toNED();  //  Convert all angles to North East Down (NED) reference frame

  return eulerAngles;
}

EulerAngles LSM9DS1::toNED() {
  //  Adjust filter angles to be consistent with NED reference frame
  switch (fusion) {
    case SensorFusion::MADGWICK:
      eulerAngles.roll = -eulerAngles.roll;
      eulerAngles.pitch = -eulerAngles.pitch;
      eulerAngles.pitchRadians = -eulerAngles.pitchRadians;
      eulerAngles.rollRadians = -eulerAngles.rollRadians;
      break;
    case SensorFusion::MAHONY:
      eulerAngles.roll = -eulerAngles.roll;
      eulerAngles.pitch = -eulerAngles.pitch;
      eulerAngles.pitchRadians = -eulerAngles.pitchRadians;
      eulerAngles.rollRadians = -eulerAngles.rollRadians;
      break;
    case SensorFusion::COMPLEMENTARY:
      eulerAngles.yaw = -eulerAngles.yaw;
      eulerAngles.yawRadians = -eulerAngles.yawRadians;
      break;
    case SensorFusion::FUSION:
      break;
    case SensorFusion::CLASSIC:
      break;
    case SensorFusion::NONE:
      break;
  }
  return eulerAngles;
}

void LSM9DS1::complementaryUpdate() {
  // Convert from force vector to angle using 3 axis formula - result in radians
  float accRollAngle  =  atan(-1 * sensorData.ay / sqrt(pow(sensorData.ax, 2) + pow(sensorData.az, 2)));
  float accPitchAngle =  -atan(-1 * sensorData.ax / sqrt(pow(sensorData.ay, 2) + pow(sensorData.az, 2)));

  //  Combine gyro and acc angles using a complementary filter
  eulerAngles.pitch = alpha * eulerAngles.pitch + (1.0f - alpha) * accPitchAngle * RAD_TO_DEG;
  eulerAngles.roll = alpha * eulerAngles.roll + (1.0f - alpha) * accRollAngle * RAD_TO_DEG;
  eulerAngles.pitchRadians = eulerAngles.pitch * DEG_TO_RAD;
  eulerAngles.rollRadians = eulerAngles.roll * DEG_TO_RAD;
}

void LSM9DS1::tiltCompensatedYaw() {
  //  Calculate yaw using magnetometer & derived roll and pitch
  float mag_x_compensated = sensorData.mx * cos(eulerAngles.pitchRadians) + sensorData.mz * sin(eulerAngles.pitchRadians);
  float mag_y_compensated = sensorData.mx * sin(eulerAngles.rollRadians) * sin(eulerAngles.pitchRadians) + sensorData.my * cos(eulerAngles.rollRadians) - sensorData.mz * sin(eulerAngles.rollRadians) * cos(eulerAngles.pitchRadians);

  eulerAngles.yawRadians = -atan2(mag_x_compensated, mag_y_compensated);
  eulerAngles.yaw =  eulerAngles.yawRadians * RAD_TO_DEG;    //  Yaw compensated for tilt
  //  float magYawAngle = atan2(sensorData.mx, sensorData.my) * RAD_TO_DEG;      //  Raw yaw from magnetometer, uncompensated for tilt - alternative yaw value
  
  eulerAngles.heading = eulerAngles.yaw - declination;
}

void LSM9DS1::updateEulerAngles() {
  // Auxiliary variables to avoid repeated arithmetic
  float sinPHI = sin(eulerAngles.rollRadians);
  float cosPHI = cos(eulerAngles.rollRadians);
  float cosTHETA = cos(eulerAngles.pitchRadians);
  float tanTHETA = tan(eulerAngles.pitchRadians);

  //  Convert gyro rates to Euler rates (ground reference frame)
  //  Euler Roll Rate, ϕ ̇= p + sin(ϕ)tan(θ) × q + cos(ϕ)tan(θ) × r
  //  Euler Pitch Rate, θ ̇= cos(ϕ) × q - sin(ϕ) × r
  //  Euler Yaw Rate, ψ ̇= [sin(ϕ) × q]/cos(θ) + cos(ϕ)cos(θ) × r

  float eulerRollRate = sensorData.gy + sinPHI * tanTHETA * sensorData.gx + cosPHI * tanTHETA * sensorData.gz;
  float eulerPitchRate = cosPHI * sensorData.gx - sinPHI * sensorData.gz;
  float eulerYawRate = (sinPHI * sensorData.gx) / cosTHETA + cosPHI * cosTHETA * sensorData.gz;

  eulerAngles.rollRadians  += eulerRollRate * deltaT;    // Angle around the X-axis
  eulerAngles.pitchRadians += eulerPitchRate * deltaT;   // Angle around the Y-axis
  eulerAngles.yawRadians   += eulerYawRate * deltaT;     // Angle around the Z-axis    
  eulerAngles.roll = eulerAngles.rollRadians * RAD_TO_DEG;
  eulerAngles.pitch = eulerAngles.pitchRadians * RAD_TO_DEG;
  eulerAngles.yaw = eulerAngles.yawRadians * RAD_TO_DEG;
}

EulerAngles LSM9DS1::fusionEulerAngles(SensorData sensorData) {
  // Assign latest sensor data
  FusionVector gyroscope = {sensorData.gx, sensorData.gy, sensorData.gz};     // gyroscope data in degrees/s
  FusionVector accelerometer = {sensorData.ax, sensorData.ay, sensorData.az}; // accelerometer data in g
  FusionVector magnetometer = {sensorData.mx, sensorData.my, sensorData.mz};  // magnetometer data in arbitrary units

  // Apply calibration
  gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
  accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
  magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);
  
  // Update gyroscope offset correction algorithm
  gyroscope = FusionOffsetUpdate(&fusionOffset, gyroscope);

  // Update gyroscope AHRS algorithm
  FusionAhrsUpdate(&fusionAhrs, gyroscope, accelerometer, magnetometer, deltaT);

  // Algorithm outputs converted to Euler angles
  const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&fusionAhrs));
  const FusionVector earth = FusionAhrsGetEarthAcceleration(&fusionAhrs);

  //  Translate from FUSION NWU frame to Reefwing NED reference frame
  eulerAngles.roll = -euler.angle.roll;
  eulerAngles.pitch = -euler.angle.pitch;
  eulerAngles.yaw = -euler.angle.yaw;

  eulerAngles.rollRadians = eulerAngles.roll * DEG_TO_RAD;
  eulerAngles.pitchRadians = eulerAngles.pitch * DEG_TO_RAD;
  eulerAngles.yawRadians = eulerAngles.yaw * DEG_TO_RAD;

  return eulerAngles;
}

SensorData LSM9DS1::rawData() {
  return sensorData;
}

Quaternion LSM9DS1::getQuaternion() {
  return quaternion;
}

void LSM9DS1::setFusionAlgorithm(SensorFusion algo) {
  fusion = algo;
}

void LSM9DS1::setFusionPeriod(float p) {
  sampleRate = (int)(1.0/p);
  fusionSettings.rejectionTimeout = 5 * sampleRate;
  FusionAhrsSetSettings(&fusionAhrs, &fusionSettings);
}

void LSM9DS1::setDeclination(float dec) {
  declination = dec;
}

void LSM9DS1::setAlpha(float a) {
  alpha = constrain(a, 0.0, 1.0);
}

void LSM9DS1::setBeta(float b) {
  beta = b;
}

void LSM9DS1::setGyroMeasError(float gme) {
  gyroMeasError = gme;
  beta = sqrt(3.0f / 4.0f) * gyroMeasError;
}

void LSM9DS1::setKp(float p) {
  Kp = p;
}

void LSM9DS1::setKi(float i) {
  Ki = i;
}

void LSM9DS1::setFusionGain(float g) {
  fusionSettings.gain = g;
  FusionAhrsSetSettings(&fusionAhrs, &fusionSettings);
}

uint8_t LSM9DS1::whoAmIGyro() {
    // Read WHO_AM_I register for LSM9DS1 accel/gyro
    return readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_WHO_AM_I);  
}

uint8_t LSM9DS1::whoAmIMag() {
    // Read WHO_AM_I register for LSM9DS1 magnetometer
    return readByte(LSM9DS1M_ADDRESS, LSM9DS1M_WHO_AM_I);  
}

bool LSM9DS1::connected() {
  return (whoAmIGyro() == LSM9DS1XG_WHO_AM_I_VALUE &&
          whoAmIMag() == LSM9DS1M_WHO_AM_I_VALUE);
}

void LSM9DS1::resetQuaternion() {
  quaternion.reset();
}

float LSM9DS1::readGyroTemp() {
    // x/y/z gyro register data stored here
    uint8_t rawData[2];  
    // Read the two raw data registers sequentially into data array 
    readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_TEMP_L, 2, &rawData[0]);  
    // Turn the MSB and LSB into a 16-bit signed value
    int16_t rawTemp = (((int16_t)rawData[1] << 8) | rawData[0]);  
    // Gyro chip temperature in degrees Centigrade
    return ((float)rawTemp/256.0 + 25.0); 
}

void LSM9DS1::readAccelData(int16_t* destination) {
  uint8_t rawData[6];  // x/y/z accel register data stored here

  readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_XL, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}

void LSM9DS1::readGyroData(int16_t* destination) {
  uint8_t rawData[6];  // x/y/z gyro register data stored here

  readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_G, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}

void LSM9DS1::readMagData(int16_t* destination) {
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  
  readBytes(LSM9DS1M_ADDRESS, LSM9DS1M_OUT_X_L_M, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}

void LSM9DS1::setAccResolution(Ascale ascale) {
  aScale = static_cast<uint8_t>(ascale);
  switch (ascale) {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 16 Gs (01), 4 Gs (10), and 8 Gs  (11). 
    case Ascale::AFS_2G:
      aRes = 2.0/32768.0;
      break;
    case Ascale::AFS_16G:
      aRes = 16.0/32768.0;
      break;
    case Ascale::AFS_4G:
      aRes = 4.0/32768.0;
      break;
    case Ascale::AFS_8G:
      aRes = 8.0/32768.0;
      break;
  }

  accelerometerSensitivity.axis = { aRes, aRes, aRes };
}

void LSM9DS1::setGyroResolution(Gscale gscale) {
  gScale = static_cast<uint8_t>(gscale);
  switch (gscale) {
    // Possible gyro scales (and their register bit settings) are:
    // 245 DPS (00), 500 DPS (01), and 2000 DPS  (11). 
    case Gscale::GFS_245DPS:
      gRes = 245.0/32768.0;
      break;
    case Gscale::GFS_500DPS:
      gRes = 500.0/32768.0;
      break;
    case Gscale::GFS_2000DPS:
      gRes = 2000.0/32768.0;
      break;
  }

  gyroscopeSensitivity.axis = { gRes, gRes, gRes };
}

void LSM9DS1::setMagResolution(Mscale mscale) {
  mScale = static_cast<uint8_t>(mscale);
  switch (mscale) {
    // Possible magnetometer scales (and their register bit settings) are:
    // 4 Gauss (00), 8 Gauss (01), 12 Gauss (10) and 16 Gauss (11)
    case Mscale::MFS_4G:
      mRes = 4.0/32768.0;
      break;
    case Mscale::MFS_8G:
      mRes = 8.0/32768.0;
      break;
    case Mscale::MFS_12G:
      mRes = 12.0/32768.0;
      break;
    case Mscale::MFS_16G:
      mRes = 16.0/32768.0;
      break;
  }

}

float LSM9DS1::getAccResolution() {
  return (aRes * 1000.0);
}

float LSM9DS1::getGyroResolution() {
  return (gRes * 1000.0);
}

float LSM9DS1::getMagResolution() {
  return (mRes * 1000.0);
}

SensorData LSM9DS1::filterFormat() {
  //  Correct magnetometer x-axis mismatch
  //  and convert gyro to radians/sec.
  SensorData filterData;

  filterData.ax = sensorData.ax;
  filterData.ay = sensorData.ay;
  filterData.az = sensorData.az;

  filterData.gx = sensorData.gx * M_PI/180.0;
  filterData.gy = sensorData.gy * M_PI/180.0;
  filterData.gz = sensorData.gz * M_PI/180.0;

  filterData.mx = -sensorData.mx;
  filterData.my = sensorData.my;
  filterData.mz = sensorData.mz;

  return filterData;
}

SelfTestResults LSM9DS1::selfTest() {
  SelfTestResults results;
  float accel_noST[3] = {0.0, 0.0, 0.0}, accel_ST[3] = {0.0, 0.0, 0.0};
  float gyro_noST[3] = {0.0, 0.0, 0.0}, gyro_ST[3] = {0.0, 0.0, 0.0};

  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG10,   0x00); // disable self test
  setBiasOffsets(gyro_noST, accel_noST);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG10,   0x05); // enable gyro/accel self test
  setBiasOffsets(gyro_ST, accel_ST);

  results.gyrodx = (gyro_ST[0] - gyro_noST[0]);
  results.gyrody = (gyro_ST[1] - gyro_noST[1]);
  results.gyrodz = (gyro_ST[2] - gyro_noST[2]);

  results.accdx = 1000.0 * (accel_ST[0] - accel_noST[0]);
  results.accdy = 1000.0 * (accel_ST[1] - accel_noST[1]);
  results.accdz = 1000.0 * (accel_ST[2] - accel_noST[2]);

  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG10,   0x00); // disable self test
  delay(200);

  return results;
}

void LSM9DS1::calibrateAccGyro() {
  setBiasOffsets(gyroBias, accelBias);
}

void LSM9DS1::calibrateMag() {
  setMagneticBias(magBias);
}

// Function to copy 'len' elements from 'src' to 'dst'
void LSM9DS1::copyArray(float* src, float* dst, int len) {
    memcpy(dst, src, sizeof(src[0])*len);
}

BiasOffsets LSM9DS1::getBiasOffsets() {
  BiasOffsets biasOffsets;

  copyArray(accelBias, biasOffsets.accelBias, 3);
  copyArray(gyroBias, biasOffsets.gyroBias, 3);
  copyArray(magBias, biasOffsets.magBias, 3);

  return biasOffsets;
}

void LSM9DS1::loadAccBias(float axB, float ayB, float azB) {
  accelBias[0] = axB;
  accelBias[1] = ayB;
  accelBias[2] = azB;
}

void LSM9DS1::loadGyroBias(float gxB, float gyB, float gzB) {
  gyroBias[0] = gxB;
  gyroBias[1] = gyB;
  gyroBias[2] = gzB;
}

void LSM9DS1::loadMagBias(float mxB, float myB, float mzB) {
  magBias[0] = mxB;
  magBias[1] = myB;
  magBias[2] = mzB;
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void LSM9DS1::setBiasOffsets(float* dest1, float* dest2) {  
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  uint16_t samples, index;

  // enable the 3-axes of the gyroscope
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG4, 0x38);
  // configure the gyroscope
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG1_G, Godr << 5 | gScale << 3 | Gbw);
  delay(200);
  // enable the three axes of the accelerometer 
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG5_XL, 0x38);
  // configure the accelerometer-specify bandwidth selection with Abw
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG6_XL, Aodr << 5 | aScale << 3 | 0x04 |Abw);
  delay(200);
  // enable block data update, allow auto-increment during multiple byte read
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x44);

  // First get gyro bias
  byte c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c | 0x02);     // Enable gyro FIFO  
  delay(50);                                                       // Wait for change to take effect
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
  delay(1000);  // delay 1000 milliseconds to collect FIFO samples

  samples = (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_SRC) & 0x2F); // Read number of stored samples

  for (index = 0; index < samples ; index++) {            // Read the gyro data stored in the FIFO
    int16_t gyro_temp[3] = {0, 0, 0};

    readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_G, 6, &data[0]);
    gyro_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
    gyro_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
    gyro_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

    gyro_bias[0] += (int32_t) gyro_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    gyro_bias[1] += (int32_t) gyro_temp[1]; 
    gyro_bias[2] += (int32_t) gyro_temp[2]; 
  }  

  gyro_bias[0] /= samples; // average the data
  gyro_bias[1] /= samples; 
  gyro_bias[2] /= samples; 

  dest1[0] = (float)gyro_bias[0] * gRes;  // Properly scale the data to get deg/s
  dest1[1] = (float)gyro_bias[1] * gRes;
  dest1[2] = (float)gyro_bias[2] * gRes;

  c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c & ~0x02);   //Disable gyro FIFO  
  delay(50);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x00);  // Enable gyro bypass mode

  // now get the accelerometer bias
  c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c | 0x02);     // Enable accel FIFO  
  delay(50);                                                       // Wait for change to take effect
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x20 | 0x1F);  // Enable accel FIFO stream mode and set watermark at 32 samples
  delay(1000);  // delay 1000 milliseconds to collect FIFO samples

  samples = (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_SRC) & 0x2F); // Read number of stored samples

  for (index = 0; index < samples ; index++) {            // Read the accel data stored in the FIFO
    int16_t accel_temp[3] = {0, 0, 0};

    readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_XL, 6, &data[0]);
    accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
    accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1]; 
    accel_bias[2] += (int32_t) accel_temp[2]; 
  }  

  accel_bias[0] /= samples; // average the data
  accel_bias[1] /= samples; 
  accel_bias[2] /= samples; 

  if (accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) (1.0/aRes);}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) (1.0/aRes);}

  dest2[0] = (float)accel_bias[0] * aRes;  // Properly scale the data to get g
  dest2[1] = (float)accel_bias[1] * aRes;
  dest2[2] = (float)accel_bias[2] * aRes;

  c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c & ~0x02);   //Disable accel FIFO  
  delay(50);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x00);  // Enable accel bypass mode
}

void LSM9DS1::setMagneticBias(float *dest1) {
  uint8_t data[6]; // data array to hold mag x, y, z, data
  uint16_t index = 0, sample_count = 128;
  int32_t mag_bias[3] = {0, 0, 0};
  int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};

  // configure the magnetometer-enable temperature compensation of mag data
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M, 0x80 | Mmode << 5 | Modr << 2); // select x,y-axis mode
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, mScale << 5 ); // select mag full scale
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG3_M, 0x00 ); // continuous conversion mode
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG4_M, Mmode << 2 ); // select z-axis mode
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG5_M, 0x40 ); // select block update mode

  delay(4000);

  for (index = 0; index < sample_count; index++) {
    int16_t mag_temp[3] = {0, 0, 0};
    
    readBytes(LSM9DS1M_ADDRESS, LSM9DS1M_OUT_X_L_M, 6, &data[0]);  // Read the six raw data registers into data array
    mag_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;   // Form signed 16-bit integer for each sample in FIFO
    mag_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
    mag_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(105);  // at 10 Hz ODR, new mag data is available every 100 ms
  }

  mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

  dest1[0] = (float) mag_bias[0] * mRes;  // save mag biases in G for main program
  dest1[1] = (float) mag_bias[1] * mRes;   
  dest1[2] = (float) mag_bias[2] * mRes;          

  //  Write biases to magnetometer offset registers as counts);
  //  Either write offset to register or correct in update()
  //  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_X_REG_L_M, (int16_t) mag_bias[0]  & 0xFF);
  //  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_X_REG_H_M, ((int16_t)mag_bias[0] >> 8) & 0xFF);
  //  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Y_REG_L_M, (int16_t) mag_bias[1] & 0xFF);
  //  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Y_REG_H_M, ((int16_t)mag_bias[1] >> 8) & 0xFF);
  //  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Z_REG_L_M, (int16_t) mag_bias[2] & 0xFF);
  //  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Z_REG_H_M, ((int16_t)mag_bias[2] >> 8) & 0xFF);
}

/******************************************************************
 *
 * I2C Read/Write methods for the LSM9DS1 - 
 * 
 ******************************************************************/

void LSM9DS1::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire1.beginTransmission(address);  // Initialize the Tx buffer
  Wire1.write(subAddress);           // Put slave register address in Tx buffer
  Wire1.write(data);                 // Put data in Tx buffer
  Wire1.endTransmission();           // Send the Tx buffer
}

uint8_t LSM9DS1::readByte(uint8_t address, uint8_t subAddress) {
  uint8_t data; // `data` will store the register data  

  Wire1.beginTransmission(address);         // Initialize the Tx buffer
  Wire1.write(subAddress);                  // Put slave register address in Tx buffer
  Wire1.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire1.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
  data = Wire1.read();                      // Fill Rx buffer with result

  return data;                             // Return data read from slave register
}

void LSM9DS1::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {  
  uint8_t i = 0;

  Wire1.beginTransmission(address);   // Initialize the Tx buffer
  Wire1.write(subAddress);            // Put slave register address in Tx buffer
  Wire1.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  Wire1.requestFrom(address, count);  // Read bytes from slave register address 
  
  while (Wire1.available()) {
    dest[i++] = Wire1.read(); }         // Put read results in the Rx buffer
}

/******************************************************************
 *
 * LPS22HB Implementation - 
 * 
 ******************************************************************/

LPS22HB::LPS22HB() { }

void LPS22HB::begin() {
  _address = LPS22HB_ADDRESS;
  Wire1.begin();
  write(LPS22HB_RES_CONF, 0x0); // resolution: temp=32, pressure=128
  write(LPS22HB_CTRL_REG1, 0x00); // one-shot mode
}

byte LPS22HB::whoAmI() {
  Wire1.beginTransmission(_address);
  Wire1.write(LPS22HB_WHO_AM_I);
  Wire1.endTransmission();
  Wire1.requestFrom(_address, 1);
  return Wire1.read();
}

bool LPS22HB::connected() {
  return (whoAmI() == LPS22HB_WHO_AM_I_VALUE);
}

float LPS22HB::readPressure() {
  write(LPS22HB_CTRL_REG2, 0x1);

  if (status(0x1) < 0)
    return 1.23;
  //delay(50);
  uint8_t pressOutH = read(LPS22HB_PRES_OUT_H);
  uint8_t pressOutL = read(LPS22HB_PRES_OUT_L);
  uint8_t pressOutXL = read(LPS22HB_PRES_OUT_XL);

  long val = ( ((long)pressOutH << 16) | ((long)pressOutL << 8) | (long)pressOutXL );
  //if (val == 1.00) readPressure();
  return val/4096.0f;
}

uint32_t LPS22HB::readPressureRAW() {
  write(LPS22HB_CTRL_REG2, 0x1);

  if (status(0x1) < 0)
    return 123;
  //delay(50);
  uint8_t pressOutH = read(LPS22HB_PRES_OUT_H);
  uint8_t pressOutL = read(LPS22HB_PRES_OUT_L);
  uint8_t pressOutXL = read(LPS22HB_PRES_OUT_XL);

  int32_t val = ( (pressOutH << 16) | (pressOutL << 8) | pressOutXL );
  val=val+0x400000;
  //if (val == 1.00) readPressure();
  return (uint32_t)val;
}

uint32_t LPS22HB::readPressureUI() {
  write(LPS22HB_CTRL_REG2, 0x1);

  if (status(0x1) < 0)
    return 1.23;
  //delay(50);
  uint8_t pressOutH = read(LPS22HB_PRES_OUT_H);
  uint8_t pressOutL = read(LPS22HB_PRES_OUT_L);
  uint8_t pressOutXL = read(LPS22HB_PRES_OUT_XL);

  uint32_t val = ((pressOutH << 16) | (pressOutL << 8) | pressOutXL );
  //if (val == 1.00) readPressure();
  return val/4096;
}

float LPS22HB::readTemperature() {
  write(LPS22HB_CTRL_REG2, 0x1);
  if (status(0x2) < 0)
    return 4.56;

  uint8_t tempOutH = read(LPS22HB_TEMP_OUT_H);
  uint8_t tempOutL = read(LPS22HB_TEMP_OUT_L);

  int16_t val = (tempOutH << 8) | (tempOutL & 0xff);
  return ((float)val)/100.0f;
}


uint8_t LPS22HB::status(uint8_t status) {
  int count = 1000;
  uint8_t data = 0xff;
  do {
    data = read(LPS22HB_STATUS_REG);
    --count;
    if (count < 0)
      break;
  } while ((data & status) == 0);

  if (count < 0)
    return -1;
  else
    return 0;
}

uint8_t LPS22HB::read(uint8_t reg) {
  Wire1.beginTransmission(_address);
  Wire1.write(reg);
  Wire1.endTransmission();
  Wire1.requestFrom(_address, 1);
  return Wire1.read();
}

void LPS22HB::write(uint8_t reg, uint8_t data) {
  Wire1.beginTransmission(_address);
  Wire1.write(reg);
  Wire1.write(data);
  Wire1.endTransmission();
}
