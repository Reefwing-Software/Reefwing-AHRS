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
  - It calls ahrs.reset() to initialize the EKF with the library's default init.

  Output
  - Serial @ 115200. Use Serial Plotter or capture CSV.

******************************************************************************************/

#include <Arduino.h>
#include <ReefwingAHRS.h>

// ---------------------------- Configuration ----------------------------

// Fixed timestep (seconds). Keep constant so results are comparable run-to-run.
static constexpr float    DT    = 0.01f;   // 10 ms — matches a typical 100 Hz IMU update rate
static constexpr uint32_t DT_US = 10000;   // Same period in microseconds, used by micros() ticker

// Optional synthetic imperfections — set any value non-zero to simulate sensor bias.
// Gyro bias shifts the commanded rate fed to the EKF's predict step; accel bias
// shifts the gravity-projected measurement fed to the EKF's update step.
static constexpr float GYRO_BIAS_DPS_X = 0.0f;
static constexpr float GYRO_BIAS_DPS_Y = 0.0f;
static constexpr float ACC_BIAS_G_X    = 0.0f;
static constexpr float ACC_BIAS_G_Y    = 0.0f;
static constexpr float ACC_BIAS_G_Z    = 0.0f;

// Motion profile: piecewise constant commanded Euler rates (deg/s)
// We generate "truth" by integrating these rates directly (consistent with EKF's simplified model).
struct RatesDps { float rollRate; float pitchRate; };

// Returns the commanded angular rates at time t.
// The profile exercises roll and pitch independently so EKF tracking can be verified
// for each axis without cross-coupling effects.
RatesDps commandedRates(float t) {
  // Timeline:
  // 0–2s:   level, still
  // 2–4s:   roll to +45 deg (22.5 dps for 2s)
  // 4–6s:   hold
  // 6–8s:   pitch to +30 deg (15 dps for 2s)
  // 8–10s:  hold
  // 10–12s: roll back to 0 deg (-22.5 dps for 2s)
  // 12–14s: pitch back to 0 deg (-15 dps for 2s)
  // 14s+:   hold (should read 0/0 if EKF has no accumulated drift)

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

// Use a float-typed PI to keep all trigonometry in single-precision arithmetic.
// Arduino's built-in PI macro is a double, which forces double promotion on AVR/ARM.
static constexpr float PI_F = 3.14159265358979323846f;

inline float degToRad(float d) { return d * (PI_F / 180.0f); }
inline float radToDeg(float r) { return r * (180.0f / PI_F); }

// Project the gravity vector (1 g down) into the sensor body frame given known
// roll and pitch angles (radians).  The resulting (ax, ay, az) values match the
// sign conventions used inside ReefwingAHRS::extendedKalmanUpdate():
//
//   accRoll  = atan2(ay, az)
//   accPitch = atan2(-ax, sqrt(ay² + az²))
//
// Consistent gravity model (standard aerospace body-frame projection):
//   ax = -sin(pitch)              — forward tilt tilts X axis away from gravity
//   ay =  sin(roll) * cos(pitch)  — right tilt tilts Y axis into gravity
//   az =  cos(roll) * cos(pitch)  — vertical component, ~1g when level
//
// With zero bias constants the accel measurement fed to the EKF is noise-free
// truth.  Add realistic noise (e.g. ±0.01 g Gaussian) to ACC_BIAS_G_* for a
// more representative test of the filter's noise-rejection behaviour.
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

// "Truth" state (radians) — integrated from commandedRates() using the same
// flat Euler step as the EKF's imuStateTransitionFunc.  This keeps model error
// at zero so only filter-tuning effects show up in the residuals.
// Note: flat Euler is a simplification; it ignores gyroscopic cross-coupling
// (i.e. no Bortz/quaternion kinematics).  It is accurate for small simultaneous
// roll+pitch angles but would accumulate error for large combined manoeuvres.
float truthRoll  = 0.0f;
float truthPitch = 0.0f;

// Timing — lastTickUs is the micros() timestamp of the previous step.
// tSec is the accumulated simulation time in seconds.
uint32_t lastTickUs = 0;
float    tSec       = 0.0f;

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait on boards that need it (e.g. Leonardo, native USB) */ }

  // Do NOT call ahrs.begin(): that probes I2C for a physical IMU and will hang
  // or misbehave without hardware attached.
  // ahrs.reset() reinitialises the EKF (state, covariance, function pointers)
  // without touching any hardware peripheral.
  ahrs.reset();
  ahrs.setFusionAlgorithm(SensorFusion::EXTENDED_KALMAN);

  lastTickUs = micros();

  // CSV header — import into Serial Plotter, Python, or Excel for visualisation.
  Serial.println(F("t,truthRollDeg,truthPitchDeg,accRollDeg,accPitchDeg,ekfRollDeg,ekfPitchDeg,gxDps,gyDps,axG,ayG,azG"));
}

void loop() {
  // Fixed-rate ticker: block until exactly DT_US microseconds have elapsed.
  // Using lastTickUs += DT_US (not lastTickUs = nowUs) means each step always
  // advances the nominal timeline by exactly DT regardless of how long the
  // Serial.print() calls took, preventing cumulative jitter on the time axis.
  // The (uint32_t) cast makes the subtraction safe across the 32-bit micros() rollover.
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastTickUs) < DT_US) {
    return;
  }
  lastTickUs += DT_US;
  tSec += DT;

  // Look up commanded angular rates for this simulation instant.
  const RatesDps r = commandedRates(tSec);

  // Integrate truth with the same flat Euler step the EKF uses internally.
  // This ensures that any residual between EKF output and truth is due to the
  // filter (noise weighting, initialisation, etc.) not model mismatch.
  truthRoll  += degToRad(r.rollRate)  * DT;
  truthPitch += degToRad(r.pitchRate) * DT;

  // Build synthetic SensorData.  Zero-init all fields so unused channels are clean.
  SensorData d{};

  // Magnetometer and yaw-gyro are unused by extendedKalmanUpdate() (EKF is 2-state:
  // roll and pitch only).  Leave them at zero to avoid influencing any code paths
  // that might inspect them.
  d.mx = d.my = d.mz = 0.0f;
  d.gz = 0.0f;

  // Gyro in deg/s — the EKF path in ReefwingAHRS.cpp multiplies by DEG_TO_RAD
  // before passing to the predict step, so supply raw deg/s here.
  d.gx = r.rollRate  + GYRO_BIAS_DPS_X;
  d.gy = r.pitchRate + GYRO_BIAS_DPS_Y;

  // Accelerometer in g-units derived from the known truth attitude.
  // With all ACC_BIAS constants at zero this is a perfect, noise-free measurement.
  gravityFromRollPitch(truthRoll, truthPitch, d.ax, d.ay, d.az);
  d.ax += ACC_BIAS_G_X;
  d.ay += ACC_BIAS_G_Y;
  d.az += ACC_BIAS_G_Z;

  // Compute accel-derived roll/pitch using the same atan2 equations as
  // ReefwingAHRS::extendedKalmanUpdate() so the CSV columns are directly comparable.
  const float accRoll  = atan2f(d.ay, d.az);
  const float accPitch = atan2f(-d.ax, sqrtf(d.ay * d.ay + d.az * d.az));

  // Run one EKF cycle: predict (gyro integration) then update (accel correction).
  // Pass the fixed DT rather than actual elapsed time so results are reproducible
  // regardless of Serial output latency.
  ahrs.extendedKalmanUpdate(d, DT);

  // EKF output is stored in ahrs.angles in degrees.
  const float ekfRollDeg  = ahrs.angles.roll;
  const float ekfPitchDeg = ahrs.angles.pitch;

  // Emit one CSV row per tick.  3 d.p. on angles is sufficient (0.001° resolution);
  // 4 d.p. on accel to capture the small off-axis components near level.
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

  // Halt after 20 s.  The motion profile is complete by 14 s; the extra 6 s lets
  // you observe steady-state EKF behaviour.
  // Note: boards with a hardware watchdog timer may reset instead of halting here.
  if (tSec > 20.0f) {
    while (true) { delay(1000); }
  }
}
