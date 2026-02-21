// Copyright (c) 2026 David Such
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

/*****************************************************************************************
  Reefwing-AHRS EKF Synthetic Test (Roll/Pitch Only)

  Purpose
  - Drive ReefwingAHRS::extendedKalmanUpdate() with synthetic IMU data (no hardware IMU).
  - Lets you A/B test the existing EKF implementation vs a patched one by swapping the
    library code and re-uploading the same sketch.
  - Prints: truth roll/pitch, accel-derived roll/pitch, EKF roll/pitch, and the synthetic IMU.

  Key details (matches ReefwingAHRS.cpp EKF path)
  - Reefwing EKF expects gyro inputs d.gx, d.gy in *deg/s* (it converts to rad/s internally).
  - Accel inputs are used only through atan2() ratios, so use "g" units (1g gravity).

  Wiring
  - None. This sketch does not call ahrs.begin() and does not touch I2C.
  - It calls ahrs.reset() to initialize the EKF with the library’s default init.

  Output
  - Serial @ 115200. Use Serial Plotter or capture CSV.

******************************************************************************************/

#include <Arduino.h>
#include <ReefwingAHRS.h>

// ---------------------------- Configuration ----------------------------

// Fixed timestep (seconds). Keep constant so results are comparable run-to-run.
static constexpr float DT = 0.01f;          // 10 ms
static constexpr uint32_t DT_US = 10000;    // 10,000 us

// Optional synthetic imperfections (set to 0 for ideal sensors)
static constexpr float GYRO_BIAS_DPS_X = 0.0f;
static constexpr float GYRO_BIAS_DPS_Y = 0.0f;
static constexpr float ACC_BIAS_G_X    = 0.0f;
static constexpr float ACC_BIAS_G_Y    = 0.0f;
static constexpr float ACC_BIAS_G_Z    = 0.0f;

// Motion profile: piecewise constant commanded Euler rates (deg/s)
// We generate "truth" by integrating these rates directly (consistent with EKF’s simplified model).
struct RatesDps { float rollRate; float pitchRate; };

RatesDps commandedRates(float t) {
  // Timeline:
  // 0–2s: level, still
  // 2–4s: roll to +45 deg (22.5 dps for 2s)
  // 4–6s: hold
  // 6–8s: pitch to +30 deg (15 dps for 2s)
  // 8–10s: hold
  // 10–12s: roll back to 0 deg (-22.5 dps for 2s)
  // 12–14s: pitch back to 0 deg (-15 dps for 2s)
  // 14s+: hold

  if (t < 2.0f)  return {0.0f, 0.0f};
  if (t < 4.0f)  return {+22.5f, 0.0f};
  if (t < 6.0f)  return {0.0f, 0.0f};
  if (t < 8.0f)  return {0.0f, +15.0f};
  if (t < 10.0f) return {0.0f, 0.0f};
  if (t < 12.0f) return {-22.5f, 0.0f};
  if (t < 14.0f) return {0.0f, -15.0f};
  return {0.0f, 0.0f};
}

// ---------------------------- Helpers ----------------------------

static constexpr float PI_F = 3.14159265358979323846f;

inline float degToRad(float d) { return d * (PI_F / 180.0f); }
inline float radToDeg(float r) { return r * (180.0f / PI_F); }

// Compute gravity vector in sensor frame from roll/pitch (yaw ignored)
// Using the same tilt conventions as the library’s accel angle extraction:
//   accRoll  = atan2(ay, az)
//   accPitch = atan2(-ax, sqrt(ay^2 + az^2))
//
// A consistent "pure gravity" model is:
//   ax = -sin(pitch)
//   ay =  sin(roll) * cos(pitch)
//   az =  cos(roll) * cos(pitch)
void gravityFromRollPitch(float rollRad, float pitchRad, float &ax_g, float &ay_g, float &az_g) {
  const float sr = sinf(rollRad);
  const float cr = cosf(rollRad);
  const float sp = sinf(pitchRad);
  const float cp = cosf(pitchRad);

  ax_g = -sp;
  ay_g =  sr * cp;
  az_g =  cr * cp;
}

// ---------------------------- Global objects ----------------------------

ReefwingAHRS ahrs;

// "Truth" state (radians)
float truthRoll = 0.0f;
float truthPitch = 0.0f;

// Timing
uint32_t lastTickUs = 0;
float tSec = 0.0f;

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait on boards that need it */ }

  // Important: do NOT call ahrs.begin(); that probes hardware IMUs.
  // reset() reinitializes the EKF using the library’s default init wiring.
  ahrs.reset();
  ahrs.setFusionAlgorithm(SensorFusion::EXTENDED_KALMAN);

  lastTickUs = micros();

  Serial.println(F("t,truthRollDeg,truthPitchDeg,accRollDeg,accPitchDeg,ekfRollDeg,ekfPitchDeg,gxDps,gyDps,axG,ayG,azG"));
}

void loop() {
  // Maintain fixed-rate stepping for repeatability
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastTickUs) < DT_US) {
    return;
  }
  lastTickUs += DT_US;
  tSec += DT;

  // Commanded rates (deg/s)
  const RatesDps r = commandedRates(tSec);

  // Integrate "truth" using the same simplified assumption as the EKF control input.
  truthRoll  += degToRad(r.rollRate)  * DT;
  truthPitch += degToRad(r.pitchRate) * DT;

  // Build synthetic SensorData
  SensorData d{};
  d.mx = d.my = d.mz = 0.0f;  // unused for EKF roll/pitch
  d.gz = 0.0f;                // unused for EKF roll/pitch

  // Gyro outputs in deg/s (matches Reefwing EKF path)
  d.gx = r.rollRate  + GYRO_BIAS_DPS_X;
  d.gy = r.pitchRate + GYRO_BIAS_DPS_Y;

  // Accel from gravity only (g units)
  gravityFromRollPitch(truthRoll, truthPitch, d.ax, d.ay, d.az);
  d.ax += ACC_BIAS_G_X;
  d.ay += ACC_BIAS_G_Y;
  d.az += ACC_BIAS_G_Z;

  // Compute accel-derived roll/pitch (same equations as ReefwingAHRS::extendedKalmanUpdate)
  const float accRoll  = atan2f(d.ay, d.az);
  const float accPitch = atan2f(-d.ax, sqrtf(d.ay * d.ay + d.az * d.az));

  // Run EKF update
  ahrs.extendedKalmanUpdate(d, DT);

  // Extract EKF outputs
  const float ekfRollDeg  = ahrs.angles.roll;
  const float ekfPitchDeg = ahrs.angles.pitch;

  // Print CSV line
  Serial.print(tSec, 3); Serial.print(',');
  Serial.print(radToDeg(truthRoll), 3); Serial.print(',');
  Serial.print(radToDeg(truthPitch), 3); Serial.print(',');
  Serial.print(radToDeg(accRoll), 3); Serial.print(',');
  Serial.print(radToDeg(accPitch), 3); Serial.print(',');
  Serial.print(ekfRollDeg, 3); Serial.print(',');
  Serial.print(ekfPitchDeg, 3); Serial.print(',');
  Serial.print(d.gx, 3); Serial.print(',');
  Serial.print(d.gy, 3); Serial.print(',');
  Serial.print(d.ax, 4); Serial.print(',');
  Serial.print(d.ay, 4); Serial.print(',');
  Serial.println(d.az, 4);

  // Stop after 20 seconds (optional)
  if (tSec > 20.0f) {
    while (true) { delay(1000); }
  }
}